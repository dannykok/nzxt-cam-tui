//! Synchronous TUI backend for the privileged hardware service.
//!
//! The backend is owned by the existing backend worker thread. It keeps one
//! framed connection and drives it with a small, current-thread Tokio runtime.

use crate::backend::{BackendCancellation, BackendError, BackendErrorKind, HardwareBackend};
use nzxt_cam_core::{
    ChannelId, CurvePoint, DeviceId, HardwareSnapshot, HostChannelPolicy, HostControlPolicy,
    KrakenDisplayMode,
};

#[cfg(target_os = "linux")]
mod linux {
    use std::{
        future::Future,
        io,
        path::{Path, PathBuf},
        time::Duration,
    };

    use futures_util::{SinkExt, StreamExt};
    use nzxt_cam_protocol::{
        ClientHello, ErrorCode, FrameError, JsonFrameCodec, MonitoringActivationOutcome,
        MonitoringDisplaySelection, MonitoringFirmwareCurve, PROTOCOL_VERSION_V6, RejectionCode,
        Request, Response, ServerResponse, from_value,
        time_budget::{CONNECT_TIMEOUT, HANDSHAKE_TIMEOUT, SEND_TIMEOUT, response_timeout},
    };
    use tokio::{
        net::UnixStream,
        runtime::{Builder, Runtime},
        time::{MissedTickBehavior, timeout},
    };
    use tokio_util::codec::Framed;

    use super::*;

    const SOCKET_PATH: &str = "/run/nzxt-cam/hardware.sock";
    const CANCELLATION_POLL_INTERVAL: Duration = Duration::from_millis(10);

    type Connection = Framed<UnixStream, JsonFrameCodec>;

    /// Live backend connected to the socket-activated hardware service.
    pub struct IpcBackend {
        runtime: Runtime,
        connection: Option<Connection>,
        socket_path: PathBuf,
        cancellation: BackendCancellation,
    }

    impl IpcBackend {
        /// Connects to and handshakes with the production service.
        pub fn new() -> Result<Self, BackendError> {
            Self::connect_path(PathBuf::from(SOCKET_PATH))
        }

        #[cfg(test)]
        pub(crate) fn connect_to(path: impl AsRef<Path>) -> Result<Self, BackendError> {
            Self::connect_path(path.as_ref().to_owned())
        }

        fn connect_path(socket_path: PathBuf) -> Result<Self, BackendError> {
            let runtime = Builder::new_current_thread()
                .enable_io()
                .enable_time()
                .build()
                .map_err(|error| {
                    BackendError::with_kind(
                        BackendErrorKind::Other,
                        format!("could not create the hardware-service runtime: {error}"),
                    )
                })?;
            let cancellation = BackendCancellation::default();
            let connection =
                runtime.block_on(connect_and_handshake(&socket_path, &cancellation))?;
            Ok(Self {
                runtime,
                connection: Some(connection),
                socket_path,
                cancellation,
            })
        }

        fn connect(&mut self) -> Result<(), BackendError> {
            debug_assert!(self.connection.is_none());
            let result = self
                .runtime
                .block_on(connect_and_handshake(&self.socket_path, &self.cancellation));
            match result {
                Ok(connection) => {
                    self.connection = Some(connection);
                    Ok(())
                }
                Err(error) => {
                    self.connection = None;
                    Err(error)
                }
            }
        }

        fn exchange(&mut self, request: Request) -> Result<Response, ExchangeFailure> {
            let connection = self
                .connection
                .take()
                .expect("exchange requires a connected service");
            let ExchangeResult { connection, result } =
                self.runtime
                    .block_on(exchange(connection, request, &self.cancellation));
            self.connection = connection;
            result
        }

        fn discard_connection(&mut self) {
            self.connection = None;
        }

        fn refresh_once(&mut self) -> Result<HardwareSnapshot, RefreshFailure> {
            if self.connection.is_none() {
                self.connect().map_err(RefreshFailure::Final)?;
            }

            match self.exchange(Request::GetSnapshot) {
                Ok(Response::Snapshot { snapshot }) => Ok(snapshot),
                Ok(Response::Error { code, message }) => Err(RefreshFailure::Final(
                    refresh_service_error(code, message.as_str()),
                )),
                Ok(
                    Response::FirmwareCurveApplied
                    | Response::KrakenDisplaySet
                    | Response::MonitoringAutoResumeSet
                    | Response::MonitoringActivated { .. },
                ) => {
                    self.discard_connection();
                    Err(RefreshFailure::Final(
                        BackendError::with_kind(
                            BackendErrorKind::InvalidData,
                            "hardware service returned an apply response to a snapshot request",
                        )
                        .invalidating_live_curve_claims(),
                    ))
                }
                Ok(
                    Response::HostControlStarted
                    | Response::HostControlUpdated
                    | Response::HostControlStopped,
                ) => {
                    self.discard_connection();
                    Err(RefreshFailure::Final(
                        BackendError::with_kind(
                            BackendErrorKind::InvalidData,
                            "hardware service returned a host-control response to a snapshot request",
                        )
                        .invalidating_live_curve_claims(),
                    ))
                }
                Err(failure) if failure.kind == ExchangeFailureKind::Transport => {
                    Err(RefreshFailure::Transport(
                        transport_error("snapshot request", &failure.message)
                            .invalidating_live_curve_claims(),
                    ))
                }
                Err(failure) => Err(RefreshFailure::Final(
                    refresh_exchange_error(failure).invalidating_live_curve_claims(),
                )),
            }
        }
    }

    impl HardwareBackend for IpcBackend {
        fn name(&self) -> &'static str {
            "MONITORING SERVICE"
        }

        fn cancellation_handle(&self) -> Option<BackendCancellation> {
            Some(self.cancellation.clone())
        }

        fn refresh(&mut self) -> Result<HardwareSnapshot, BackendError> {
            match self.refresh_once() {
                Ok(snapshot) => Ok(snapshot),
                Err(RefreshFailure::Final(error)) => Err(error),
                Err(RefreshFailure::Transport(_first_error)) => {
                    // A snapshot is read-only, so one fresh connection and one
                    // retry cannot duplicate a hardware mutation. A successful
                    // replacement snapshot carries authoritative service state; any
                    // final failure must invalidate stale firmware claims.
                    self.discard_connection();
                    match self.refresh_once() {
                        Ok(snapshot) => Ok(snapshot),
                        Err(RefreshFailure::Final(error) | RefreshFailure::Transport(error)) => {
                            Err(error.invalidating_live_curve_claims())
                        }
                    }
                }
            }
        }

        fn activate_monitoring(
            &mut self,
            curves: &[MonitoringFirmwareCurve],
            host: Option<&HostControlPolicy>,
            display: Option<&MonitoringDisplaySelection>,
        ) -> Result<Vec<MonitoringActivationOutcome>, BackendError> {
            if self.connection.is_none() {
                self.connect()?;
            }
            match self.exchange(Request::ActivateMonitoring {
                firmware_curves: curves.to_vec(),
                host_policy: host.cloned(),
                display: display.cloned(),
            }) {
                Ok(Response::MonitoringActivated { outcomes }) => Ok(outcomes),
                Ok(Response::Error { code, message }) => {
                    Err(service_error(code, message.as_str(), true))
                }
                Ok(_) => {
                    self.discard_connection();
                    Err(unknown_monitoring_outcome(
                        "monitoring activation",
                        "mismatched service response",
                    ))
                }
                Err(failure) => {
                    self.discard_connection();
                    if failure.send_started {
                        Err(unknown_monitoring_outcome(
                            "monitoring activation",
                            &failure.message,
                        ))
                    } else {
                        Err(pre_dispatch_monitoring_error(
                            "monitoring activation",
                            failure,
                        ))
                    }
                }
            }
        }

        fn set_monitoring_auto_resume(&mut self, enabled: bool) -> Result<(), BackendError> {
            if self.connection.is_none() {
                self.connect()?;
            }
            match self.exchange(Request::SetMonitoringAutoResume { enabled }) {
                Ok(Response::MonitoringAutoResumeSet) => Ok(()),
                Ok(Response::Error { code, message }) => {
                    Err(service_error(code, message.as_str(), true))
                }
                Ok(_) => {
                    self.discard_connection();
                    Err(unknown_monitoring_outcome(
                        "monitoring auto-resume",
                        "mismatched service response",
                    ))
                }
                Err(failure) => {
                    self.discard_connection();
                    if failure.send_started {
                        Err(unknown_monitoring_outcome(
                            "monitoring auto-resume",
                            &failure.message,
                        ))
                    } else {
                        Err(pre_dispatch_monitoring_error(
                            "monitoring auto-resume",
                            failure,
                        ))
                    }
                }
            }
        }

        fn set_kraken_display(
            &mut self,
            device_id: &DeviceId,
            mode: KrakenDisplayMode,
        ) -> Result<(), BackendError> {
            if self.connection.is_none() {
                self.connect()?;
            }
            match self.exchange(Request::SetKrakenDisplay {
                device_id: device_id.clone(),
                mode,
            }) {
                Ok(Response::KrakenDisplaySet) => Ok(()),
                Ok(Response::Error { code, message }) => {
                    Err(service_error(code, message.as_str(), true))
                }
                Ok(_) => {
                    self.discard_connection();
                    Err(unknown_display_outcome(
                        "hardware service returned a mismatched response",
                    ))
                }
                Err(failure) if failure.send_started => {
                    self.discard_connection();
                    Err(unknown_display_outcome(&failure.message))
                }
                Err(failure) => {
                    self.discard_connection();
                    Err(pre_dispatch_display_error(failure))
                }
            }
        }

        fn apply_curve(
            &mut self,
            device_id: &DeviceId,
            channel_id: &ChannelId,
            points: &[CurvePoint],
        ) -> Result<(), BackendError> {
            if self.connection.is_none() {
                // Connecting and handshaking cannot have changed device state.
                self.connect()?;
            }
            let request = Request::ApplyFirmwareCurve {
                device_id: device_id.clone(),
                channel_id: channel_id.clone(),
                points: points.to_vec(),
            };
            match self.exchange(request) {
                Ok(Response::FirmwareCurveApplied) => Ok(()),
                Ok(Response::Error { code, message }) => {
                    Err(apply_service_error(code, message.as_str()))
                }
                Ok(Response::Snapshot { .. }) => {
                    self.discard_connection();
                    Err(unknown_apply_outcome(
                        "hardware service returned a snapshot response to a curve apply request",
                    )
                    .invalidating_live_curve_claims())
                }
                Ok(
                    Response::HostControlStarted
                    | Response::HostControlUpdated
                    | Response::HostControlStopped
                    | Response::KrakenDisplaySet
                    | Response::MonitoringAutoResumeSet
                    | Response::MonitoringActivated { .. },
                ) => {
                    self.discard_connection();
                    Err(unknown_apply_outcome(
                        "hardware service returned a host-control response to a curve apply request",
                    )
                    .invalidating_live_curve_claims())
                }
                Err(failure) if failure.send_started => {
                    self.discard_connection();
                    Err(unknown_apply_outcome(&failure.message).invalidating_live_curve_claims())
                }
                Err(failure) => {
                    self.discard_connection();
                    Err(pre_dispatch_apply_error(failure).invalidating_live_curve_claims())
                }
            }
        }

        fn start_host_control(&mut self, policy: &HostControlPolicy) -> Result<(), BackendError> {
            if self.connection.is_none() {
                self.connect()?;
            }
            match self.exchange(Request::StartHostControl {
                complete_policy: policy.clone(),
            }) {
                Ok(Response::HostControlStarted) => Ok(()),
                Ok(Response::Error { code, message }) => {
                    let error = host_service_error(code, message.as_str());
                    if matches!(
                        error.kind(),
                        BackendErrorKind::RestoreRequired | BackendErrorKind::UnknownOutcome
                    ) {
                        self.discard_connection();
                        Err(error.invalidating_live_curve_claims())
                    } else {
                        Err(error)
                    }
                }
                Ok(_) => {
                    self.discard_connection();
                    Err(unknown_host_outcome(
                        "start",
                        "hardware service returned a mismatched response",
                    )
                    .invalidating_live_curve_claims())
                }
                Err(failure) if failure.send_started => {
                    self.discard_connection();
                    Err(unknown_host_outcome("start", &failure.message)
                        .invalidating_live_curve_claims())
                }
                Err(failure) => {
                    self.discard_connection();
                    Err(pre_dispatch_host_error("start", failure).invalidating_live_curve_claims())
                }
            }
        }

        fn update_host_control(
            &mut self,
            channel_policies: &[HostChannelPolicy],
        ) -> Result<(), BackendError> {
            if self.connection.is_none() {
                self.connect()?;
            }
            match self.exchange(Request::UpdateHostControl {
                channel_policies: channel_policies.to_vec(),
            }) {
                Ok(Response::HostControlUpdated) => Ok(()),
                Ok(Response::Error { code, message }) => {
                    let error = host_service_error(code, message.as_str());
                    if matches!(
                        error.kind(),
                        BackendErrorKind::RestoreRequired | BackendErrorKind::UnknownOutcome
                    ) {
                        self.discard_connection();
                        Err(error.invalidating_live_curve_claims())
                    } else {
                        Err(error)
                    }
                }
                Ok(_) => {
                    self.discard_connection();
                    Err(unknown_host_outcome(
                        "update",
                        "hardware service returned a mismatched response",
                    )
                    .invalidating_live_curve_claims())
                }
                Err(failure) if failure.send_started => {
                    self.discard_connection();
                    Err(unknown_host_outcome("update", &failure.message)
                        .invalidating_live_curve_claims())
                }
                Err(failure) => {
                    self.discard_connection();
                    Err(pre_dispatch_host_error("update", failure).invalidating_live_curve_claims())
                }
            }
        }

        fn stop_host_control(&mut self) -> Result<(), BackendError> {
            if self.connection.is_none() {
                self.connect()?;
            }
            match self.exchange(Request::StopHostControl) {
                Ok(Response::HostControlStopped) => Ok(()),
                Ok(Response::Error { code, message }) => {
                    let error = host_service_error(code, message.as_str());
                    if matches!(
                        error.kind(),
                        BackendErrorKind::RestoreRequired | BackendErrorKind::UnknownOutcome
                    ) {
                        self.discard_connection();
                        Err(error.invalidating_live_curve_claims())
                    } else {
                        Err(error)
                    }
                }
                Ok(_) => {
                    self.discard_connection();
                    Err(unknown_host_outcome(
                        "stop",
                        "hardware service returned a mismatched response",
                    )
                    .invalidating_live_curve_claims())
                }
                Err(failure) if failure.send_started => {
                    self.discard_connection();
                    Err(unknown_host_outcome("stop", &failure.message)
                        .invalidating_live_curve_claims())
                }
                Err(failure) => {
                    self.discard_connection();
                    Err(pre_dispatch_host_error("stop", failure).invalidating_live_curve_claims())
                }
            }
        }
    }

    enum RefreshFailure {
        Transport(BackendError),
        Final(BackendError),
    }

    struct ExchangeResult {
        connection: Option<Connection>,
        result: Result<Response, ExchangeFailure>,
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    enum ExchangeFailureKind {
        Transport,
        Protocol,
        Timeout,
        Cancelled,
    }

    struct ExchangeFailure {
        kind: ExchangeFailureKind,
        message: String,
        send_started: bool,
    }

    async fn connect_and_handshake(
        path: &Path,
        cancellation: &BackendCancellation,
    ) -> Result<Connection, BackendError> {
        if cancellation.is_cancelled() {
            return Err(cancelled_connect_error());
        }

        let stream = cancellable_timeout(CONNECT_TIMEOUT, cancellation, UnixStream::connect(path))
            .await
            .map_err(|failure| connect_failure(path, failure))?
            .map_err(|error| connect_io_error(path, error))?;
        let connection = Framed::new(stream, JsonFrameCodec::new());
        handshake_connection(connection, path, cancellation).await
    }

    async fn handshake_connection(
        connection: Connection,
        path: &Path,
        cancellation: &BackendCancellation,
    ) -> Result<Connection, BackendError> {
        let handshake = async {
            let (mut sender, mut receiver) = connection.split();
            let (send_result, response_result) =
                tokio::join!(sender.send(ClientHello::v6()), async {
                    let value = receiver.next().await.ok_or_else(|| {
                        FrameError::Io(io::Error::new(
                            io::ErrorKind::UnexpectedEof,
                            "service closed during handshake",
                        ))
                    })??;
                    from_value::<ServerResponse>(value)
                });
            let response = response_result.map_err(|error| handshake_data_error(path, error))?;

            match response {
                ServerResponse::Rejected {
                    code: RejectionCode::ServiceBusy,
                } => Err(BackendError::with_kind(
                    BackendErrorKind::Unavailable,
                    "hardware service is busy; another nzxt-cam-tui instance is already connected",
                )),
                ServerResponse::Rejected {
                    code: RejectionCode::UnsupportedVersion,
                } => Err(BackendError::with_kind(
                    BackendErrorKind::Unsupported,
                    "hardware service does not support this nzxt-cam-tui protocol version",
                )),
                ServerResponse::Rejected { code } => Err(BackendError::with_kind(
                    BackendErrorKind::InvalidData,
                    format!("hardware service rejected the client handshake: {code:?}"),
                )),
                ServerResponse::Ready { protocol_version } => {
                    send_result.map_err(|error| handshake_data_error(path, error))?;
                    if protocol_version != PROTOCOL_VERSION_V6 {
                        return Err(BackendError::with_kind(
                            BackendErrorKind::InvalidData,
                            format!(
                                "hardware service selected unexpected protocol version {protocol_version}"
                            ),
                        ));
                    }
                    sender.reunite(receiver).map_err(|_| {
                        BackendError::with_kind(
                            BackendErrorKind::Other,
                            "could not reconstitute the hardware-service connection after handshake",
                        )
                    })
                }
            }
        };

        cancellable_timeout(HANDSHAKE_TIMEOUT, cancellation, handshake)
            .await
            .map_err(|failure| handshake_failure(path, failure))?
    }

    fn handshake_data_error(path: &Path, error: FrameError) -> BackendError {
        BackendError::with_kind(
            BackendErrorKind::InvalidData,
            format!(
                "hardware service handshake on {} failed: {error}",
                path.display()
            ),
        )
    }

    async fn exchange(
        mut connection: Connection,
        request: Request,
        cancellation: &BackendCancellation,
    ) -> ExchangeResult {
        let response_timeout = response_timeout(&request);
        if cancellation.is_cancelled() {
            return failed_exchange(
                ExchangeFailureKind::Cancelled,
                "hardware-service request was cancelled before sending",
                false,
            );
        }

        // Once this bounded send is polled, an I/O failure or timeout may mean
        // that some request bytes reached the peer. Cancellation is checked
        // above instead of racing the send so a pre-send cancellation cannot
        // be misclassified as a potentially dispatched request.
        let send_result = timeout(SEND_TIMEOUT, connection.send(request)).await;
        match send_result {
            Err(_) => {
                return failed_exchange(
                    ExchangeFailureKind::Timeout,
                    "hardware-service request timed out while sending",
                    true,
                );
            }
            Ok(Err(error)) => {
                let kind = classify_frame_error(&error);
                let send_started = matches!(&error, FrameError::Io(_));
                return failed_exchange(
                    kind,
                    format!("could not send hardware-service request: {error}"),
                    send_started,
                );
            }
            Ok(Ok(())) => {}
        }

        let read_result = tokio::select! {
            biased;
            () = cancellation_requested(cancellation) => {
                return failed_exchange(
                    ExchangeFailureKind::Cancelled,
                    "hardware-service request was cancelled while awaiting its response",
                    true,
                );
            }
            result = timeout(response_timeout, connection.next()) => result,
        };
        let value = match read_result {
            Err(_) => {
                return failed_exchange(
                    ExchangeFailureKind::Timeout,
                    "hardware-service request timed out while awaiting its response",
                    true,
                );
            }
            Ok(None) => {
                return failed_exchange(
                    ExchangeFailureKind::Transport,
                    "hardware service closed the connection before responding",
                    true,
                );
            }
            Ok(Some(Err(error))) => {
                let kind = classify_frame_error(&error);
                return failed_exchange(
                    kind,
                    format!("could not read hardware-service response: {error}"),
                    true,
                );
            }
            Ok(Some(Ok(value))) => value,
        };

        match from_value::<Response>(value) {
            Ok(response) => ExchangeResult {
                connection: Some(connection),
                result: Ok(response),
            },
            Err(error) => failed_exchange(
                ExchangeFailureKind::Protocol,
                format!("hardware service returned an invalid response: {error}"),
                true,
            ),
        }
    }

    fn failed_exchange(
        kind: ExchangeFailureKind,
        message: impl Into<String>,
        send_started: bool,
    ) -> ExchangeResult {
        ExchangeResult {
            connection: None,
            result: Err(ExchangeFailure {
                kind,
                message: message.into(),
                send_started,
            }),
        }
    }

    async fn cancellation_requested(cancellation: &BackendCancellation) {
        let mut poll = tokio::time::interval(CANCELLATION_POLL_INTERVAL);
        poll.set_missed_tick_behavior(MissedTickBehavior::Skip);
        loop {
            poll.tick().await;
            if cancellation.is_cancelled() {
                return;
            }
        }
    }

    enum TimedFailure {
        Cancelled,
        TimedOut,
    }

    async fn cancellable_timeout<F, T>(
        duration: Duration,
        cancellation: &BackendCancellation,
        future: F,
    ) -> Result<T, TimedFailure>
    where
        F: Future<Output = T>,
    {
        tokio::select! {
            biased;
            () = cancellation_requested(cancellation) => Err(TimedFailure::Cancelled),
            result = timeout(duration, future) => result.map_err(|_| TimedFailure::TimedOut),
        }
    }

    fn classify_frame_error(error: &FrameError) -> ExchangeFailureKind {
        match error {
            FrameError::Io(error) if error.kind() != io::ErrorKind::InvalidData => {
                ExchangeFailureKind::Transport
            }
            FrameError::Io(_)
            | FrameError::Json(_)
            | FrameError::EmptyPayload
            | FrameError::PayloadTooLarge { .. } => ExchangeFailureKind::Protocol,
        }
    }

    fn connect_failure(path: &Path, failure: TimedFailure) -> BackendError {
        match failure {
            TimedFailure::Cancelled => cancelled_connect_error(),
            TimedFailure::TimedOut => BackendError::with_kind(
                BackendErrorKind::Timeout,
                format!(
                    "timed out connecting to the hardware service at {}",
                    path.display()
                ),
            ),
        }
    }

    fn handshake_failure(path: &Path, failure: TimedFailure) -> BackendError {
        match failure {
            TimedFailure::Cancelled => cancelled_connect_error(),
            TimedFailure::TimedOut => BackendError::with_kind(
                BackendErrorKind::Timeout,
                format!(
                    "timed out handshaking with the hardware service at {}",
                    path.display()
                ),
            ),
        }
    }

    fn cancelled_connect_error() -> BackendError {
        BackendError::with_kind(
            BackendErrorKind::Unavailable,
            "hardware-service connection was cancelled",
        )
    }

    fn connect_io_error(path: &Path, error: io::Error) -> BackendError {
        let kind = match error.kind() {
            io::ErrorKind::PermissionDenied => BackendErrorKind::PermissionDenied,
            io::ErrorKind::TimedOut => BackendErrorKind::Timeout,
            _ => BackendErrorKind::Unavailable,
        };
        BackendError::with_kind(
            kind,
            format!(
                "could not connect to the hardware service at {}: {error}. Ensure nzxt-cam-hwd.socket is installed and running and this user belongs to nzxt-cam-control",
                path.display()
            ),
        )
    }

    fn refresh_exchange_error(failure: ExchangeFailure) -> BackendError {
        let kind = match failure.kind {
            ExchangeFailureKind::Protocol => BackendErrorKind::InvalidData,
            ExchangeFailureKind::Timeout => BackendErrorKind::Timeout,
            ExchangeFailureKind::Cancelled | ExchangeFailureKind::Transport => {
                BackendErrorKind::Unavailable
            }
        };
        BackendError::with_kind(kind, failure.message)
    }

    fn transport_error(operation: &str, message: &str) -> BackendError {
        BackendError::with_kind(
            BackendErrorKind::Unavailable,
            format!("hardware-service {operation} lost its connection: {message}"),
        )
    }

    fn pre_dispatch_display_error(failure: ExchangeFailure) -> BackendError {
        let kind = match failure.kind {
            ExchangeFailureKind::Protocol => BackendErrorKind::InvalidData,
            ExchangeFailureKind::Timeout => BackendErrorKind::Timeout,
            ExchangeFailureKind::Cancelled | ExchangeFailureKind::Transport => {
                BackendErrorKind::Unavailable
            }
        };
        BackendError::with_kind(
            kind,
            format!(
                "Kraken display selection did not start: {}",
                failure.message
            ),
        )
    }

    fn pre_dispatch_apply_error(failure: ExchangeFailure) -> BackendError {
        let kind = match failure.kind {
            ExchangeFailureKind::Protocol => BackendErrorKind::InvalidData,
            ExchangeFailureKind::Timeout => BackendErrorKind::Timeout,
            ExchangeFailureKind::Cancelled | ExchangeFailureKind::Transport => {
                BackendErrorKind::Unavailable
            }
        };
        BackendError::with_kind(
            kind,
            format!("curve apply did not start: {}", failure.message),
        )
    }

    fn unknown_display_outcome(message: &str) -> BackendError {
        BackendError::with_kind(
            BackendErrorKind::UnknownOutcome,
            format!(
                "Kraken display request sent but outcome is unknown: {message}; refresh the service snapshot"
            ),
        )
    }

    fn unknown_apply_outcome(message: &str) -> BackendError {
        BackendError::with_kind(
            BackendErrorKind::UnknownOutcome,
            format!(
                "curve apply failed after sending the request: {message}; device state is unknown"
            ),
        )
    }

    fn pre_dispatch_monitoring_error(operation: &str, failure: ExchangeFailure) -> BackendError {
        let kind = match failure.kind {
            ExchangeFailureKind::Protocol => BackendErrorKind::InvalidData,
            ExchangeFailureKind::Timeout => BackendErrorKind::Timeout,
            ExchangeFailureKind::Cancelled | ExchangeFailureKind::Transport => {
                BackendErrorKind::Unavailable
            }
        };
        BackendError::with_kind(kind, format!("{operation} not sent: {}", failure.message))
    }

    fn unknown_monitoring_outcome(operation: &str, message: &str) -> BackendError {
        BackendError::with_kind(
            BackendErrorKind::UnknownOutcome,
            format!(
                "{operation} outcome unknown after sending the request: {message}; review service snapshot before retrying"
            ),
        )
    }

    fn pre_dispatch_host_error(operation: &str, failure: ExchangeFailure) -> BackendError {
        let kind = match failure.kind {
            ExchangeFailureKind::Protocol => BackendErrorKind::InvalidData,
            ExchangeFailureKind::Timeout => BackendErrorKind::Timeout,
            ExchangeFailureKind::Cancelled | ExchangeFailureKind::Transport => {
                BackendErrorKind::Unavailable
            }
        };
        BackendError::with_kind(
            kind,
            format!(
                "host-control {operation} did not start: {}",
                failure.message
            ),
        )
    }

    fn unknown_host_outcome(operation: &str, message: &str) -> BackendError {
        BackendError::with_kind(
            BackendErrorKind::UnknownOutcome,
            format!(
                "host-control {operation} failed after sending the request: {message}; service state is unknown"
            ),
        )
    }

    fn refresh_service_error(code: ErrorCode, message: &str) -> BackendError {
        service_error(code, message, false)
    }

    fn apply_service_error(code: ErrorCode, message: &str) -> BackendError {
        service_error(code, message, true)
    }

    fn host_service_error(code: ErrorCode, message: &str) -> BackendError {
        service_error(code, message, true)
    }

    fn service_error(code: ErrorCode, message: &str, apply_operation: bool) -> BackendError {
        let kind = match code {
            ErrorCode::Unavailable => BackendErrorKind::Unavailable,
            ErrorCode::PermissionDenied => BackendErrorKind::PermissionDenied,
            ErrorCode::Unsupported => BackendErrorKind::Unsupported,
            ErrorCode::InvalidData => BackendErrorKind::InvalidData,
            ErrorCode::Timeout if apply_operation => BackendErrorKind::UnknownOutcome,
            ErrorCode::Timeout => BackendErrorKind::Timeout,
            ErrorCode::UnknownOutcome => BackendErrorKind::UnknownOutcome,
            ErrorCode::RestoreRequired => BackendErrorKind::RestoreRequired,
            ErrorCode::Internal => BackendErrorKind::Other,
        };
        BackendError::with_kind(kind, format!("hardware service: {message}"))
    }

    #[cfg(test)]
    mod tests {
        use std::{
            io::{Read, Write},
            os::unix::net::{UnixListener, UnixStream as StdUnixStream},
            path::{Path, PathBuf},
            sync::{
                Arc,
                atomic::{AtomicU64, AtomicUsize, Ordering},
                mpsc,
            },
            thread,
            time::{Duration, Instant},
        };

        use nzxt_cam_core::HostControlState;
        use nzxt_cam_protocol::{
            ErrorMessage, MAX_FRAME_LENGTH, MAX_MONITORING_FIRMWARE_CURVES,
            MonitoringActivationStatus, MonitoringActivationTarget, Request,
            time_budget::{
                ACTIVATION_RESPONSE_TIMEOUT, APPLY_RESPONSE_TIMEOUT, APPLY_USB_COMMANDS,
                DISPLAY_PREFLIGHT_USB_COMMANDS, HOST_OPERATION_RESPONSE_TIMEOUT,
                LIQUIDCTL_COMMAND_TIMEOUT, MAX_ACTIVATION_USB_COMMANDS,
                PERSISTENCE_MARGIN_PER_TARGET, SNAPSHOT_USB_COMMANDS, USB_LOCK_ACQUIRE_TIMEOUT,
            },
        };
        use serde::{Serialize, de::DeserializeOwned};

        use super::*;

        static NEXT_SOCKET: AtomicU64 = AtomicU64::new(0);

        struct TestSocket {
            path: PathBuf,
        }

        impl TestSocket {
            fn bind() -> (Self, UnixListener) {
                let id = NEXT_SOCKET.fetch_add(1, Ordering::Relaxed);
                let path = std::env::temp_dir()
                    .join(format!("nzxt-cam-tui-ipc-{}-{id}.sock", std::process::id()));
                let _ = std::fs::remove_file(&path);
                let listener = UnixListener::bind(&path).unwrap();
                (Self { path }, listener)
            }
        }

        impl Drop for TestSocket {
            fn drop(&mut self) {
                let _ = std::fs::remove_file(&self.path);
            }
        }

        fn read_frame<T: DeserializeOwned>(stream: &mut StdUnixStream) -> T {
            let mut length = [0_u8; 4];
            stream.read_exact(&mut length).unwrap();
            let mut payload = vec![0; u32::from_be_bytes(length) as usize];
            stream.read_exact(&mut payload).unwrap();
            serde_json::from_slice(&payload).unwrap()
        }

        fn write_frame<T: Serialize>(stream: &mut StdUnixStream, value: &T) {
            let payload = serde_json::to_vec(value).unwrap();
            stream
                .write_all(&u32::try_from(payload.len()).unwrap().to_be_bytes())
                .unwrap();
            stream.write_all(&payload).unwrap();
        }

        fn ready(stream: &mut StdUnixStream) {
            assert_eq!(read_frame::<ClientHello>(stream), ClientHello::v6());
            write_frame(stream, &ServerResponse::ready_v6());
        }

        fn spawn_peer(
            script: impl FnOnce(UnixListener) + Send + 'static,
        ) -> (TestSocket, thread::JoinHandle<()>) {
            let (socket, listener) = TestSocket::bind();
            let peer = thread::spawn(move || script(listener));
            (socket, peer)
        }

        fn snapshot(sequence: u64) -> HardwareSnapshot {
            HardwareSnapshot {
                devices: Vec::new(),
                sequence,
                host_control: Default::default(),
                kraken_display: Default::default(),
                monitoring: Default::default(),
            }
        }

        fn host_policy() -> HostControlPolicy {
            HostControlPolicy {
                channels: vec![nzxt_cam_core::HostChannelPolicy {
                    channel_id: ChannelId::new("case-fan"),
                    curve: nzxt_cam_core::HostCurve {
                        source: nzxt_cam_core::HostTemperatureSource::CpuGpuMax,
                        points: vec![
                            nzxt_cam_core::HostCurvePoint {
                                temperature_millidegrees: 22_000,
                                duty_percent: 30,
                            },
                            nzxt_cam_core::HostCurvePoint {
                                temperature_millidegrees: 100_000,
                                duty_percent: 100,
                            },
                        ],
                    },
                }],
            }
        }

        fn host_snapshot(sequence: u64, state: HostControlState) -> HardwareSnapshot {
            HardwareSnapshot {
                devices: Vec::new(),
                sequence,
                host_control: nzxt_cam_core::HostControlSnapshot {
                    state,
                    channels: vec![nzxt_cam_core::HostChannelCapability {
                        channel_id: ChannelId::new("case-fan"),
                        name: "Case fan".into(),
                        minimum_duty_percent: 30,
                    }],
                    active_policy: (state == HostControlState::Running).then(host_policy),
                    last_error: None,
                },
                kraken_display: Default::default(),
                monitoring: Default::default(),
            }
        }

        fn points() -> Vec<CurvePoint> {
            vec![
                CurvePoint {
                    temperature: 20,
                    duty: 40,
                },
                CurvePoint {
                    temperature: 59,
                    duty: 100,
                },
            ]
        }

        /// Uses production framing, request-to-budget selection and response
        /// cancellation. Only the fake peer's operations use virtual time.
        async fn delayed_success(
            request: Request,
            response: Response,
            commands: u32,
            persisted_targets: u32,
            host_work: bool,
        ) -> (Response, Duration) {
            let (client, peer) = UnixStream::pair().unwrap();
            let client = Framed::new(client, JsonFrameCodec::new());
            let mut peer = Framed::new(peer, JsonFrameCodec::new());
            let expected = request.clone();
            let cancellation = BackendCancellation::default();
            let started = tokio::time::Instant::now();
            let peer = async {
                let value = peer.next().await.unwrap().unwrap();
                assert_eq!(from_value::<Request>(value).unwrap(), expected);
                for _ in 0..commands {
                    tokio::time::sleep(USB_LOCK_ACQUIRE_TIMEOUT - Duration::from_millis(100)).await;
                    tokio::time::sleep(LIQUIDCTL_COMMAND_TIMEOUT - Duration::from_millis(100))
                        .await;
                }
                tokio::time::sleep(PERSISTENCE_MARGIN_PER_TARGET * persisted_targets).await;
                if host_work {
                    tokio::time::sleep(HOST_OPERATION_RESPONSE_TIMEOUT).await;
                }
                peer.send(response).await.unwrap();
            };
            let (result, ()) = tokio::join!(exchange(client, request, &cancellation), peer);
            assert!(result.connection.is_some());
            match result.result {
                Ok(response) => (response, started.elapsed()),
                Err(error) => panic!("delayed operation failed: {}", error.message),
            }
        }

        #[tokio::test(start_paused = true)]
        async fn successful_reads_apply_and_display_outlive_old_short_deadlines() {
            let expected = Response::Snapshot {
                snapshot: snapshot(17),
            };
            let (response, elapsed) = delayed_success(
                Request::GetSnapshot,
                expected.clone(),
                SNAPSHOT_USB_COMMANDS,
                0,
                false,
            )
            .await;
            assert_eq!(response, expected);
            assert!(elapsed > Duration::from_secs(12));
            let (response, elapsed) = delayed_success(
                Request::ApplyFirmwareCurve {
                    device_id: DeviceId::new("kraken"),
                    channel_id: ChannelId::new("pump"),
                    points: points(),
                },
                Response::FirmwareCurveApplied,
                APPLY_USB_COMMANDS,
                1,
                false,
            )
            .await;
            assert_eq!(response, Response::FirmwareCurveApplied);
            assert!(elapsed > Duration::from_secs(7));
            let (response, elapsed) = delayed_success(
                Request::SetKrakenDisplay {
                    device_id: DeviceId::new("kraken"),
                    mode: KrakenDisplayMode::Cpu,
                },
                Response::KrakenDisplaySet,
                DISPLAY_PREFLIGHT_USB_COMMANDS,
                1,
                false,
            )
            .await;
            assert_eq!(response, Response::KrakenDisplaySet);
            assert!(elapsed > Duration::from_secs(7));
        }

        #[tokio::test(start_paused = true)]
        async fn maximum_activation_covers_all_97_commands_persistence_and_host_work() {
            let curves: Vec<_> = (0..MAX_MONITORING_FIRMWARE_CURVES)
                .map(|n| MonitoringFirmwareCurve {
                    device_id: DeviceId::new(format!("kraken-{n}")),
                    channel_id: ChannelId::new("pump"),
                    points: (20..60)
                        .map(|temperature| CurvePoint {
                            temperature,
                            duty: if temperature == 59 { 100 } else { 40 },
                        })
                        .collect(),
                })
                .collect();
            let mut outcomes: Vec<_> = curves
                .iter()
                .map(|curve| MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::FirmwareCurve {
                        device_id: curve.device_id.clone(),
                        channel_id: curve.channel_id.clone(),
                    },
                    status: MonitoringActivationStatus::Applied,
                    error: None,
                })
                .collect();
            outcomes.push(MonitoringActivationOutcome {
                target: MonitoringActivationTarget::Display {
                    device_id: DeviceId::new("kraken-0"),
                },
                status: MonitoringActivationStatus::Pending,
                error: None,
            });
            outcomes.push(MonitoringActivationOutcome {
                target: MonitoringActivationTarget::HostControl {},
                status: MonitoringActivationStatus::Applied,
                error: None,
            });
            let expected = Response::MonitoringActivated { outcomes };
            let (response, elapsed) = delayed_success(
                Request::ActivateMonitoring {
                    firmware_curves: curves,
                    host_policy: Some(host_policy()),
                    display: Some(MonitoringDisplaySelection {
                        device_id: DeviceId::new("kraken-0"),
                        mode: KrakenDisplayMode::Cpu,
                    }),
                },
                expected.clone(),
                MAX_ACTIVATION_USB_COMMANDS,
                MAX_MONITORING_FIRMWARE_CURVES as u32 + 1,
                true,
            )
            .await;
            assert_eq!(response, expected);
            assert!(elapsed > Duration::from_secs(210));
            assert!(elapsed < ACTIVATION_RESPONSE_TIMEOUT);
        }

        #[tokio::test(start_paused = true)]
        async fn long_activation_remains_cancellable_without_resending() {
            let (client, peer) = UnixStream::pair().unwrap();
            let client = Framed::new(client, JsonFrameCodec::new());
            let mut peer = Framed::new(peer, JsonFrameCodec::new());
            let cancellation = BackendCancellation::default();
            let peer_script = async {
                let value = peer.next().await.unwrap().unwrap();
                assert!(matches!(
                    from_value::<Request>(value).unwrap(),
                    Request::ActivateMonitoring { .. }
                ));
                tokio::time::sleep(Duration::from_secs(211)).await;
                cancellation.cancel();
                assert!(peer.next().await.is_none(), "cancelled request was resent");
            };
            let (result, ()) = tokio::join!(
                exchange(
                    client,
                    Request::ActivateMonitoring {
                        firmware_curves: Vec::new(),
                        host_policy: None,
                        display: None
                    },
                    &cancellation,
                ),
                peer_script
            );
            assert!(result.connection.is_none());
            let error = result.result.err().unwrap();
            assert_eq!(error.kind, ExchangeFailureKind::Cancelled);
            assert!(error.send_started);
        }

        #[tokio::test(start_paused = true)]
        async fn unresponsive_apply_is_still_bounded_and_unknown_after_send() {
            let (client, peer) = UnixStream::pair().unwrap();
            let client = Framed::new(client, JsonFrameCodec::new());
            let mut peer = Framed::new(peer, JsonFrameCodec::new());
            let cancellation = BackendCancellation::default();
            let started = tokio::time::Instant::now();
            let peer_script = async {
                assert!(peer.next().await.is_some());
                assert!(peer.next().await.is_none());
            };
            let (result, ()) = tokio::join!(
                exchange(
                    client,
                    Request::ApplyFirmwareCurve {
                        device_id: DeviceId::new("kraken"),
                        channel_id: ChannelId::new("pump"),
                        points: points(),
                    },
                    &cancellation,
                ),
                peer_script
            );
            assert!(result.connection.is_none());
            let error = result.result.err().unwrap();
            assert_eq!(error.kind, ExchangeFailureKind::Timeout);
            assert!(error.send_started);
            assert!(started.elapsed() >= APPLY_RESPONSE_TIMEOUT);
        }

        #[test]
        fn lock_wait_timeout_service_error_is_definite_not_unknown_write() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::ApplyFirmwareCurve { .. }
                ));
                write_frame(
                    &mut stream,
                    &Response::Error {
                        code: ErrorCode::Unavailable,
                        message: ErrorMessage::new(
                            "curve write timed out waiting for the shared USB lock; command did not start (no subprocess spawned)",
                        ),
                    },
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            let error = backend
                .apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &points())
                .unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(!error.write_outcome_unknown());
            assert!(error.to_string().contains("no subprocess spawned"));
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn successful_handshake_snapshot_and_apply_preserve_exact_payloads() {
            let expected_points = points();
            let peer_points = expected_points.clone();
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(read_frame::<Request>(&mut stream), Request::GetSnapshot);
                write_frame(
                    &mut stream,
                    &Response::Snapshot {
                        snapshot: snapshot(17),
                    },
                );
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::ApplyFirmwareCurve {
                        device_id: DeviceId::new("kraken"),
                        channel_id: ChannelId::new("pump"),
                        points: peer_points,
                    }
                );
                write_frame(&mut stream, &Response::FirmwareCurveApplied);
            });

            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            assert_eq!(backend.name(), "MONITORING SERVICE");
            assert_eq!(backend.refresh().unwrap(), snapshot(17));
            backend
                .apply_curve(
                    &DeviceId::new("kraken"),
                    &ChannelId::new("pump"),
                    &expected_points,
                )
                .unwrap();
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn activation_and_future_resume_use_v6_wire_without_retrying_lost_acks() {
            use nzxt_cam_protocol::{
                MonitoringActivationOutcome, MonitoringActivationStatus, MonitoringActivationTarget,
            };
            let curves = vec![MonitoringFirmwareCurve {
                device_id: DeviceId::new("kraken"),
                channel_id: ChannelId::new("pump"),
                points: (20..60)
                    .map(|temperature| CurvePoint {
                        temperature,
                        duty: if temperature == 59 { 100 } else { 40 },
                    })
                    .collect(),
            }];
            let expected = curves.clone();
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::ActivateMonitoring {
                        firmware_curves: expected,
                        host_policy: None,
                        display: Some(MonitoringDisplaySelection {
                            device_id: DeviceId::new("kraken"),
                            mode: KrakenDisplayMode::Cpu
                        })
                    }
                );
                write_frame(
                    &mut stream,
                    &Response::MonitoringActivated {
                        outcomes: vec![MonitoringActivationOutcome {
                            target: MonitoringActivationTarget::Display {
                                device_id: DeviceId::new("kraken"),
                            },
                            status: MonitoringActivationStatus::Pending,
                            error: None,
                        }],
                    },
                );
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::SetMonitoringAutoResume { enabled: true }
                );
                write_frame(&mut stream, &Response::MonitoringAutoResumeSet);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            let outcomes = backend
                .activate_monitoring(
                    &curves,
                    None,
                    Some(&MonitoringDisplaySelection {
                        device_id: DeviceId::new("kraken"),
                        mode: KrakenDisplayMode::Cpu,
                    }),
                )
                .unwrap();
            assert_eq!(outcomes[0].status, MonitoringActivationStatus::Pending);
            backend.set_monitoring_auto_resume(true).unwrap();
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn lost_activation_reply_is_unknown_and_never_resent() {
            let requests = Arc::new(AtomicUsize::new(0));
            let seen = Arc::clone(&requests);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::ActivateMonitoring { .. }
                ));
                seen.fetch_add(1, Ordering::Relaxed);
                drop(stream);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            assert_eq!(
                backend
                    .activate_monitoring(&[], None, None)
                    .unwrap_err()
                    .kind(),
                BackendErrorKind::UnknownOutcome
            );
            assert_eq!(requests.load(Ordering::Relaxed), 1);
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn display_selection_uses_exact_service_request_and_ack() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::SetKrakenDisplay {
                        device_id: DeviceId::new("kraken"),
                        mode: KrakenDisplayMode::CpuGpuLiquid,
                    }
                );
                write_frame(&mut stream, &Response::KrakenDisplaySet);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend
                .set_kraken_display(&DeviceId::new("kraken"), KrakenDisplayMode::CpuGpuLiquid)
                .unwrap();
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn display_lost_reply_is_unknown_and_never_resent() {
            let requests = Arc::new(AtomicUsize::new(0));
            let seen = Arc::clone(&requests);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::SetKrakenDisplay { .. }
                ));
                seen.fetch_add(1, Ordering::Relaxed);
                drop(stream);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            let error = backend
                .set_kraken_display(&DeviceId::new("kraken"), KrakenDisplayMode::Cpu)
                .unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert_eq!(requests.load(Ordering::Relaxed), 1);
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn display_mismatched_ack_is_unknown_and_discards_connection() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::SetKrakenDisplay { .. }
                ));
                write_frame(&mut stream, &Response::FirmwareCurveApplied);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            let error = backend
                .set_kraken_display(&DeviceId::new("kraken"), KrakenDisplayMode::Gpu)
                .unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(backend.connection.is_none());
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn display_pre_send_cancellation_is_definite_and_sends_no_request() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend.cancellation_handle().unwrap().cancel();
            let error = backend
                .set_kraken_display(&DeviceId::new("kraken"), KrakenDisplayMode::Gpu)
                .unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(!error.write_outcome_unknown());
            assert!(error.to_string().contains("did not start"));
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn host_start_refresh_and_stop_send_no_heartbeats_even_after_many_snapshots() {
            let expected = host_policy();
            let peer_policy = expected.clone();
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::StartHostControl {
                        complete_policy: peer_policy,
                    }
                );
                write_frame(&mut stream, &Response::HostControlStarted);
                for sequence in 1..=25 {
                    assert_eq!(read_frame::<Request>(&mut stream), Request::GetSnapshot);
                    write_frame(
                        &mut stream,
                        &Response::Snapshot {
                            snapshot: host_snapshot(sequence, HostControlState::Running),
                        },
                    );
                }
                assert_eq!(read_frame::<Request>(&mut stream), Request::StopHostControl);
                write_frame(&mut stream, &Response::HostControlStopped);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend.start_host_control(&expected).unwrap();
            for sequence in 1..=25 {
                assert_eq!(
                    backend.refresh().unwrap(),
                    host_snapshot(sequence, HostControlState::Running)
                );
            }
            backend.stop_host_control().unwrap();
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn firmware_apply_while_host_running_sends_only_apply() {
            let expected = points();
            let peer_points = expected.clone();
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::StartHostControl { .. }
                ));
                write_frame(&mut stream, &Response::HostControlStarted);
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::ApplyFirmwareCurve {
                        device_id: DeviceId::new("kraken"),
                        channel_id: ChannelId::new("pump"),
                        points: peer_points,
                    }
                );
                write_frame(&mut stream, &Response::FirmwareCurveApplied);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend.start_host_control(&host_policy()).unwrap();
            backend
                .apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &expected)
                .unwrap();
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn reconnect_snapshot_reports_actual_running_policy_without_restarting_or_stopping() {
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut first, _) = listener.accept().unwrap();
                ready(&mut first);
                assert_eq!(read_frame::<Request>(&mut first), Request::GetSnapshot);
                drop(first);
                let (mut replacement, _) = listener.accept().unwrap();
                ready(&mut replacement);
                assert_eq!(
                    read_frame::<Request>(&mut replacement),
                    Request::GetSnapshot
                );
                write_frame(
                    &mut replacement,
                    &Response::Snapshot {
                        snapshot: host_snapshot(9, HostControlState::Running),
                    },
                );
                assert_eq!(
                    read_frame::<Request>(&mut replacement),
                    Request::StopHostControl
                );
                write_frame(&mut replacement, &Response::HostControlStopped);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            assert_eq!(
                backend.refresh().unwrap(),
                host_snapshot(9, HostControlState::Running)
            );
            backend.stop_host_control().unwrap();
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn host_update_sends_batch_and_rejects_mismatched_reply() {
            let mut changed = host_policy().channels;
            let mut second = changed[0].clone();
            second.channel_id = ChannelId::new("second-group");
            changed.push(second);
            let expected = changed.clone();
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::UpdateHostControl {
                        channel_policies: expected
                    }
                );
                write_frame(&mut stream, &Response::HostControlStarted);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            let error = backend.update_host_control(&changed).unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(error.invalidates_live_curve_claims());
            assert!(backend.connection.is_none());
            peer.join().unwrap();
        }

        #[test]
        fn host_update_cancelled_before_send_is_definite() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend.cancellation_handle().unwrap().cancel();
            let error = backend
                .update_host_control(&host_policy().channels[..1])
                .unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(!error.write_outcome_unknown());
            assert!(error.to_string().contains("did not start"));
            peer.join().unwrap();
        }

        #[test]
        fn host_update_lost_reply_is_unknown_without_retry() {
            let mut changed = host_policy().channels;
            let mut second = changed[0].clone();
            second.channel_id = ChannelId::new("second-group");
            changed.push(second);
            let expected = changed.clone();
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(
                    read_frame::<Request>(&mut stream),
                    Request::UpdateHostControl {
                        channel_policies: expected
                    }
                );
                drop(stream);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            let error = backend.update_host_control(&changed).unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(error.invalidates_live_curve_claims());
            peer.join().unwrap();
        }

        #[test]
        fn host_start_response_mismatch_is_unknown_and_never_retried() {
            let requests = Arc::new(AtomicUsize::new(0));
            let peer_requests = Arc::clone(&requests);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::StartHostControl { .. }
                ));
                peer_requests.fetch_add(1, Ordering::Relaxed);
                write_frame(&mut stream, &Response::HostControlStopped);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            let error = backend.start_host_control(&host_policy()).unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(error.invalidates_live_curve_claims());
            assert_eq!(requests.load(Ordering::Relaxed), 1);
            assert!(backend.connection.is_none());
            peer.join().unwrap();
        }

        #[test]
        fn host_start_cancellation_before_send_is_definite() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend.cancellation_handle().unwrap().cancel();

            let error = backend.start_host_control(&host_policy()).unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(!error.write_outcome_unknown());
            assert!(error.to_string().contains("did not start"));
            peer.join().unwrap();
        }

        #[test]
        fn blocked_host_start_cancellation_is_unknown_and_closes_the_session() {
            let (request_seen_tx, request_seen_rx) = mpsc::channel();
            let accepts = Arc::new(AtomicUsize::new(0));
            let peer_accepts = Arc::clone(&accepts);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                peer_accepts.fetch_add(1, Ordering::Relaxed);
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::StartHostControl { .. }
                ));
                request_seen_tx.send(()).unwrap();
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let backend = IpcBackend::connect_to(&socket.path).unwrap();
            let cancellation = backend.cancellation_handle().unwrap();
            let operation = thread::spawn(move || {
                let mut backend = backend;
                backend.start_host_control(&host_policy())
            });
            request_seen_rx
                .recv_timeout(Duration::from_secs(1))
                .unwrap();

            cancellation.cancel();
            let error = operation.join().unwrap().unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(error.invalidates_live_curve_claims());
            assert_eq!(accepts.load(Ordering::Relaxed), 1);
            peer.join().unwrap();
        }

        #[test]
        fn host_stop_transport_loss_is_unknown_and_never_retried() {
            let requests = Arc::new(AtomicUsize::new(0));
            let peer_requests = Arc::clone(&requests);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::StartHostControl { .. }
                ));
                write_frame(&mut stream, &Response::HostControlStarted);
                assert_eq!(read_frame::<Request>(&mut stream), Request::StopHostControl);
                peer_requests.fetch_add(1, Ordering::Relaxed);
                drop(stream);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend.start_host_control(&host_policy()).unwrap();

            let error = backend.stop_host_control().unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(error.invalidates_live_curve_claims());
            assert_eq!(requests.load(Ordering::Relaxed), 1);
            peer.join().unwrap();
        }

        #[test]
        fn startup_service_busy_mentions_another_tui_instance() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                write_frame(
                    &mut stream,
                    &ServerResponse::rejected(RejectionCode::ServiceBusy),
                );
            });

            let error = IpcBackend::connect_to(&socket.path).err().unwrap();
            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(error.to_string().contains("another nzxt-cam-tui instance"));
            peer.join().unwrap();
        }

        #[test]
        fn service_busy_wins_when_peer_closes_before_hello_is_sent() {
            let runtime = Builder::new_current_thread()
                .enable_io()
                .enable_time()
                .build()
                .unwrap();
            let cancellation = BackendCancellation::default();

            for _ in 0..32 {
                let (client, mut peer) = StdUnixStream::pair().unwrap();
                let peer = thread::spawn(move || {
                    write_frame(
                        &mut peer,
                        &ServerResponse::rejected(RejectionCode::ServiceBusy),
                    );
                    // Deliberately do not read ClientHello. Closing with unread
                    // client data makes its concurrent send fail with BrokenPipe.
                });
                peer.join().unwrap();
                client.set_nonblocking(true).unwrap();

                let error = runtime
                    .block_on(async {
                        let stream = UnixStream::from_std(client).unwrap();
                        let connection = Framed::new(stream, JsonFrameCodec::new());
                        handshake_connection(
                            connection,
                            Path::new("synchronized test socket"),
                            &cancellation,
                        )
                        .await
                    })
                    .unwrap_err();
                assert_eq!(error.kind(), BackendErrorKind::Unavailable);
                assert!(error.to_string().contains("another nzxt-cam-tui instance"));
            }
        }

        #[test]
        fn every_service_error_code_maps_exactly() {
            let mappings = [
                (ErrorCode::Unavailable, BackendErrorKind::Unavailable),
                (
                    ErrorCode::PermissionDenied,
                    BackendErrorKind::PermissionDenied,
                ),
                (ErrorCode::Unsupported, BackendErrorKind::Unsupported),
                (ErrorCode::InvalidData, BackendErrorKind::InvalidData),
                (ErrorCode::Timeout, BackendErrorKind::Timeout),
                (ErrorCode::UnknownOutcome, BackendErrorKind::UnknownOutcome),
                (
                    ErrorCode::RestoreRequired,
                    BackendErrorKind::RestoreRequired,
                ),
                (ErrorCode::Internal, BackendErrorKind::Other),
            ];
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                for (code, _) in mappings {
                    assert_eq!(read_frame::<Request>(&mut stream), Request::GetSnapshot);
                    write_frame(
                        &mut stream,
                        &Response::Error {
                            code,
                            message: ErrorMessage::new("mapped diagnostic"),
                        },
                    );
                }
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            for (code, expected) in mappings {
                let error = backend.refresh().unwrap_err();
                assert_eq!(error.kind(), expected, "{code:?}");
                assert_eq!(
                    error.write_outcome_unknown(),
                    matches!(code, ErrorCode::UnknownOutcome)
                );
                assert!(!error.invalidates_live_curve_claims());
                assert!(error.to_string().contains("mapped diagnostic"));
            }
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn refresh_reconnects_and_retries_once_after_transport_loss() {
            let requests = Arc::new(AtomicUsize::new(0));
            let peer_requests = Arc::clone(&requests);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut first, _) = listener.accept().unwrap();
                ready(&mut first);
                assert_eq!(read_frame::<Request>(&mut first), Request::GetSnapshot);
                peer_requests.fetch_add(1, Ordering::Relaxed);
                drop(first);

                let (mut second, _) = listener.accept().unwrap();
                ready(&mut second);
                assert_eq!(read_frame::<Request>(&mut second), Request::GetSnapshot);
                peer_requests.fetch_add(1, Ordering::Relaxed);
                write_frame(
                    &mut second,
                    &Response::Snapshot {
                        snapshot: snapshot(22),
                    },
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            assert_eq!(backend.refresh().unwrap(), snapshot(22));
            assert_eq!(requests.load(Ordering::Relaxed), 2);
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn failed_refresh_reconnect_invalidates_live_curve_claims() {
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut first, _) = listener.accept().unwrap();
                ready(&mut first);
                assert_eq!(read_frame::<Request>(&mut first), Request::GetSnapshot);
                drop(first);

                let (mut second, _) = listener.accept().unwrap();
                ready(&mut second);
                assert_eq!(read_frame::<Request>(&mut second), Request::GetSnapshot);
                drop(second);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            let error = backend.refresh().unwrap_err();

            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(error.invalidates_live_curve_claims());
            assert!(backend.connection.is_none());
            peer.join().unwrap();
        }

        #[test]
        fn apply_cancelled_immediately_before_send_is_pre_dispatch() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            backend.cancellation_handle().unwrap().cancel();

            let error = backend
                .apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &points())
                .unwrap_err();

            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(!error.write_outcome_unknown());
            assert!(error.invalidates_live_curve_claims());
            assert!(error.to_string().contains("cancelled before sending"));
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn oversized_apply_is_invalid_data_before_dispatch() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            let oversized_device_id = DeviceId::new("x".repeat(MAX_FRAME_LENGTH));

            let error = backend
                .apply_curve(&oversized_device_id, &ChannelId::new("pump"), &points())
                .unwrap_err();

            assert_eq!(error.kind(), BackendErrorKind::InvalidData);
            assert!(!error.write_outcome_unknown());
            assert!(error.invalidates_live_curve_claims());
            assert!(error.to_string().contains("did not start"));
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn apply_transport_loss_is_unknown_and_is_never_resent() {
            let requests = Arc::new(AtomicUsize::new(0));
            let peer_requests = Arc::clone(&requests);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::ApplyFirmwareCurve { .. }
                ));
                peer_requests.fetch_add(1, Ordering::Relaxed);
                drop(stream);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            let error = backend
                .apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &points())
                .unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(error.write_outcome_unknown());
            assert!(error.invalidates_live_curve_claims());
            assert_eq!(requests.load(Ordering::Relaxed), 1);
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn service_timeout_is_read_timeout_but_unknown_apply_outcome() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(read_frame::<Request>(&mut stream), Request::GetSnapshot);
                write_frame(
                    &mut stream,
                    &Response::Error {
                        code: ErrorCode::Timeout,
                        message: ErrorMessage::new("snapshot timed out"),
                    },
                );
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::ApplyFirmwareCurve { .. }
                ));
                write_frame(
                    &mut stream,
                    &Response::Error {
                        code: ErrorCode::Timeout,
                        message: ErrorMessage::new("liquidctl apply timed out"),
                    },
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            let refresh_error = backend.refresh().unwrap_err();
            assert_eq!(refresh_error.kind(), BackendErrorKind::Timeout);
            assert!(!refresh_error.write_outcome_unknown());
            assert!(!refresh_error.invalidates_live_curve_claims());

            let apply_error = backend
                .apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &points())
                .unwrap_err();
            assert_eq!(apply_error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(apply_error.write_outcome_unknown());
            assert!(!apply_error.invalidates_live_curve_claims());
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn apply_reconnect_handshake_timeout_is_pre_dispatch() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut first, _) = listener.accept().unwrap();
                ready(&mut first);
                let mut byte = [0_u8; 1];
                assert_eq!(first.read(&mut byte).unwrap(), 0);

                let (_second, _) = listener.accept().unwrap();
                thread::sleep(HANDSHAKE_TIMEOUT + Duration::from_millis(100));
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();
            drop(backend.connection.take());

            let error = backend
                .apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &points())
                .unwrap_err();

            assert_eq!(error.kind(), BackendErrorKind::Timeout);
            assert!(!error.write_outcome_unknown());
            assert!(!error.invalidates_live_curve_claims());
            assert!(error.to_string().contains("handshaking"));
            peer.join().unwrap();
        }

        #[test]
        fn unexpected_responses_have_operation_specific_safety() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut first, _) = listener.accept().unwrap();
                ready(&mut first);
                assert_eq!(read_frame::<Request>(&mut first), Request::GetSnapshot);
                write_frame(&mut first, &Response::FirmwareCurveApplied);
                drop(first);

                let (mut second, _) = listener.accept().unwrap();
                ready(&mut second);
                assert!(matches!(
                    read_frame::<Request>(&mut second),
                    Request::ApplyFirmwareCurve { .. }
                ));
                write_frame(
                    &mut second,
                    &Response::Snapshot {
                        snapshot: snapshot(1),
                    },
                );
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            let refresh_error = backend.refresh().unwrap_err();
            assert_eq!(refresh_error.kind(), BackendErrorKind::InvalidData);
            assert!(refresh_error.invalidates_live_curve_claims());
            let apply_error = backend
                .apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &points())
                .unwrap_err();
            assert_eq!(apply_error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(apply_error.invalidates_live_curve_claims());
            drop(backend);
            peer.join().unwrap();
        }

        #[test]
        fn malformed_response_is_invalid_data_and_discards_the_connection() {
            let (socket, peer) = spawn_peer(|listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(read_frame::<Request>(&mut stream), Request::GetSnapshot);
                write_frame(&mut stream, &serde_json::json!({"type": "future_response"}));
            });
            let mut backend = IpcBackend::connect_to(&socket.path).unwrap();

            let error = backend.refresh().unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::InvalidData);
            assert!(error.invalidates_live_curve_claims());
            assert!(backend.connection.is_none());
            peer.join().unwrap();
        }

        #[test]
        fn blocked_refresh_cancellation_wakes_promptly_and_closes_socket() {
            let (request_seen_tx, request_seen_rx) = mpsc::channel();
            let (eof_tx, eof_rx) = mpsc::channel();
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                ready(&mut stream);
                assert_eq!(read_frame::<Request>(&mut stream), Request::GetSnapshot);
                request_seen_tx.send(()).unwrap();
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
                eof_tx.send(()).unwrap();
            });
            let backend = IpcBackend::connect_to(&socket.path).unwrap();
            let cancellation = backend.cancellation_handle().unwrap();
            let started = Instant::now();
            let operation = thread::spawn(move || {
                let mut backend = backend;
                backend.refresh()
            });
            request_seen_rx
                .recv_timeout(Duration::from_secs(1))
                .unwrap();

            cancellation.cancel();
            let error = operation.join().unwrap().unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::Unavailable);
            assert!(error.invalidates_live_curve_claims());
            assert!(started.elapsed() < Duration::from_secs(1));
            eof_rx.recv_timeout(Duration::from_secs(1)).unwrap();
            peer.join().unwrap();
        }

        #[test]
        fn blocked_apply_cancellation_is_unknown_and_not_resent() {
            let (request_seen_tx, request_seen_rx) = mpsc::channel();
            let accepts = Arc::new(AtomicUsize::new(0));
            let peer_accepts = Arc::clone(&accepts);
            let (socket, peer) = spawn_peer(move |listener| {
                let (mut stream, _) = listener.accept().unwrap();
                peer_accepts.fetch_add(1, Ordering::Relaxed);
                ready(&mut stream);
                assert!(matches!(
                    read_frame::<Request>(&mut stream),
                    Request::ApplyFirmwareCurve { .. }
                ));
                request_seen_tx.send(()).unwrap();
                let mut byte = [0_u8; 1];
                assert_eq!(stream.read(&mut byte).unwrap(), 0);
                listener.set_nonblocking(true).unwrap();
                thread::sleep(Duration::from_millis(100));
                assert!(
                    matches!(listener.accept(), Err(error) if error.kind() == io::ErrorKind::WouldBlock)
                );
            });
            let backend = IpcBackend::connect_to(&socket.path).unwrap();
            let cancellation = backend.cancellation_handle().unwrap();
            let operation = thread::spawn(move || {
                let mut backend = backend;
                backend.apply_curve(&DeviceId::new("kraken"), &ChannelId::new("pump"), &points())
            });
            request_seen_rx
                .recv_timeout(Duration::from_secs(1))
                .unwrap();

            cancellation.cancel();
            let error = operation.join().unwrap().unwrap_err();
            assert_eq!(error.kind(), BackendErrorKind::UnknownOutcome);
            assert!(error.write_outcome_unknown());
            assert!(error.invalidates_live_curve_claims());
            assert_eq!(accepts.load(Ordering::Relaxed), 1);
            peer.join().unwrap();
        }

        #[test]
        fn backend_is_send() {
            fn assert_send<T: Send>() {}
            assert_send::<IpcBackend>();
        }
    }
}

#[cfg(target_os = "linux")]
pub use linux::IpcBackend;

#[cfg(not(target_os = "linux"))]
mod unsupported {
    use super::*;

    /// Placeholder used so demo mode remains portable.
    pub struct IpcBackend;

    impl IpcBackend {
        pub fn new() -> Result<Self, BackendError> {
            Err(BackendError::with_kind(
                BackendErrorKind::Unsupported,
                "the monitoring service requires Linux; use --demo on this platform",
            ))
        }
    }

    impl HardwareBackend for IpcBackend {
        fn name(&self) -> &'static str {
            "SERVICE • UNSUPPORTED"
        }

        fn refresh(&mut self) -> Result<HardwareSnapshot, BackendError> {
            Err(Self::new().unwrap_err())
        }

        fn apply_curve(
            &mut self,
            _device_id: &DeviceId,
            _channel_id: &ChannelId,
            _points: &[CurvePoint],
        ) -> Result<(), BackendError> {
            Err(Self::new().unwrap_err())
        }

        fn start_host_control(&mut self, _policy: &HostControlPolicy) -> Result<(), BackendError> {
            Err(Self::new().unwrap_err())
        }

        fn stop_host_control(&mut self) -> Result<(), BackendError> {
            Err(Self::new().unwrap_err())
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub use unsupported::IpcBackend;

#[cfg(all(test, target_os = "linux"))]
#[path = "ipc_real_server_tests.rs"]
mod real_server_tests;
