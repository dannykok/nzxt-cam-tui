//! Lean Linux Unix-socket hardware service.
//!
//! One accepted connection owns the command slot, not the host-control policy.
//! Ready clients issue sequential operations; later clients receive `ServiceBusy`.
//! Accepted host commands finish after disconnect, while service shutdown has
//! an independent signal to the continuously running host-control worker.

use std::{
    future::Future,
    io,
    sync::{Arc, Mutex, MutexGuard},
    time::Duration,
};

use futures_util::{Sink, SinkExt, Stream, StreamExt};
use nzxt_cam_protocol::{
    ClientHello, ErrorCode, ErrorMessage, FrameError, JsonFrameCodec, RejectionCode, Request,
    Response, ServerResponse, from_value, validate_frame_payload,
};
use serde_json::Value;
use tokio::{
    net::{UnixListener, UnixStream},
    sync::watch,
    task::JoinHandle,
    time::timeout,
};
use tokio_util::codec::Framed;

use crate::{
    hardware::{HardwareCancellation, HardwareError, HardwareErrorKind, HardwareOperations},
    host_control::HostControlShutdownHandle,
};

/// Fixed time allowed for receiving the complete initial client hello.
const INITIAL_HELLO_TIMEOUT: Duration = Duration::from_secs(2);
const INVALID_REQUEST_MESSAGE: &str = "invalid operation request";
const OVERSIZED_SNAPSHOT_MESSAGE: &str = "hardware snapshot exceeds the protocol frame limit";
const INITIALIZING_MESSAGE: &str = "hardware service is initializing; retry shortly";

/// Kernel-authenticated identity captured before any active-client bytes are
/// parsed. Supplementary groups are intentionally not reconstructed.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct PeerCredentials {
    pid: u32,
    uid: u32,
    gid: u32,
}

impl PeerCredentials {
    fn read_from(stream: &UnixStream) -> Option<Self> {
        let credentials = stream.peer_cred().ok()?;
        let pid = u32::try_from(credentials.pid()?)
            .ok()
            .filter(|pid| *pid != 0)?;

        Some(Self {
            pid,
            uid: credentials.uid(),
            gid: credentials.gid(),
        })
    }
}

/// Fatal listener failure. Per-connection failures release the active slot and
/// do not terminate the service.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ServiceError {
    AcceptFailed { kind: io::ErrorKind },
    InitializationPanicked,
}

impl std::fmt::Display for ServiceError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AcceptFailed { kind } => {
                write!(formatter, "accepting a client failed ({kind:?})")
            }
            Self::InitializationPanicked => write!(formatter, "hardware initialization panicked"),
        }
    }
}

impl std::error::Error for ServiceError {}

/// Serves an already-bound listener using one shared hardware manager until
/// `shutdown` resolves.
///
/// There is exactly one active connection. A successful v6 client may issue
/// sequential operations over that connection. During guarded startup, the
/// handshake is prompt but hardware requests return `Unavailable` (initializing).
/// A second accepted connection is rejected without reading its bytes. Shutdown is
/// relayed to the active connection and waits for any blocking hardware call to
/// observe cancellation and finish before returning.
pub async fn serve_until<H, F>(
    listener: UnixListener,
    hardware: H,
    shutdown: F,
) -> Result<(), ServiceError>
where
    H: HardwareOperations,
    F: Future<Output = ()>,
{
    let (initialized, _) = watch::channel(false);
    serve_with_readiness(listener, hardware, shutdown, initialized).await
}

async fn serve_with_readiness<H, F>(
    listener: UnixListener,
    hardware: H,
    shutdown: F,
    initialized: watch::Sender<bool>,
) -> Result<(), ServiceError>
where
    H: HardwareOperations,
    F: Future<Output = ()>,
{
    tokio::pin!(shutdown);
    // Capture both handles before the initializer owns the manager mutex.
    // Neither shutdown nor early control-plane traffic may acquire that lock.
    let cancellation = hardware.cancellation_handle();
    let lifetime = ServiceShutdownGuard {
        host: hardware.host_control_shutdown_handle(),
        usb: cancellation.clone(),
    };
    // Do not begin automatic resume when shutdown was already requested.
    tokio::select! {
        biased;
        _ = &mut shutdown => return Ok(()),
        _ = std::future::ready(()) => {}
    }
    let hardware = Arc::new(Mutex::new(hardware));
    // Recovery/resume runs exactly once, independently of client connections.
    let mut initialization = Some({
        let hardware = Arc::clone(&hardware);
        tokio::task::spawn_blocking(move || lock_hardware(&hardware).initialize_host_control())
    });
    let (shutdown_tx, shutdown_rx) = watch::channel(false);
    let mut active: Option<JoinHandle<Option<PeerCredentials>>> = None;

    let result = loop {
        enum Event {
            Shutdown,
            Initialized(Result<Result<(), HardwareError>, tokio::task::JoinError>),
            ActiveFinished,
            Accepted(io::Result<UnixStream>),
        }

        // Release completed slots before processing queued accepts. Startup
        // completion opens the hardware gate, not the protocol handshake.
        let event = tokio::select! {
            biased;
            _ = &mut shutdown => Event::Shutdown,
            result = async { initialization.as_mut().unwrap().await },
                if initialization.is_some() => Event::Initialized(result),
            _result = async { active.as_mut().unwrap().await },
                if active.is_some() => Event::ActiveFinished,
            accepted = listener.accept() => {
                Event::Accepted(accepted.map(|(stream, _address)| stream))
            }
        };

        match event {
            Event::Shutdown => break Ok(()),
            Event::Initialized(result) => {
                initialization = None;
                match result {
                    Err(_) => break Err(ServiceError::InitializationPanicked),
                    Ok(Err(error)) => eprintln!("hardware initialization failed: {error}"),
                    Ok(Ok(())) => {}
                }
                // Failed recovery remains queryable in snapshots and must not
                // prevent Stop or authorize another attempt at initialization.
                initialized.send_replace(true);
            }
            Event::ActiveFinished => active = None,
            Event::Accepted(Err(error)) => {
                break Err(ServiceError::AcceptFailed { kind: error.kind() });
            }
            Event::Accepted(Ok(stream)) if active.is_some() => {
                let rejection = reject_busy(stream);
                tokio::pin!(rejection);
                tokio::select! {
                    biased;
                    _ = &mut shutdown => break Ok(()),
                    _result = &mut rejection => {}
                }
            }
            Event::Accepted(Ok(stream)) => {
                active = Some(tokio::spawn(handle_active_connection(
                    stream,
                    Arc::clone(&hardware),
                    cancellation.clone(),
                    initialized.subscribe(),
                    shutdown_rx.clone(),
                )));
            }
        }
    };

    lifetime.request_shutdown();
    signal_and_drain_active(&shutdown_tx, active).await;
    if let Some(initialization) = initialization {
        let _ = initialization.await;
    }
    result
}

async fn signal_and_drain_active(
    shutdown: &watch::Sender<bool>,
    active: Option<JoinHandle<Option<PeerCredentials>>>,
) {
    let _ = shutdown.send(true);
    if let Some(active) = active {
        let _ = active.await;
    }
}

async fn handle_active_connection<H>(
    stream: UnixStream,
    hardware: Arc<Mutex<H>>,
    cancellation: HardwareCancellation,
    initialized: watch::Receiver<bool>,
    mut shutdown: watch::Receiver<bool>,
) -> Option<PeerCredentials>
where
    H: HardwareOperations,
{
    let peer_credentials = PeerCredentials::read_from(&stream)?;
    let mut framed = Framed::new(stream, JsonFrameCodec::new());

    let first_frame = tokio::select! {
        biased;
        _ = wait_for_shutdown(&mut shutdown) => return None,
        result = timeout(INITIAL_HELLO_TIMEOUT, framed.next()) => result,
    };
    let first_frame = match first_frame {
        Ok(Some(Ok(frame))) => frame,
        Ok(None | Some(Err(_))) | Err(_) => return None,
    };

    let hello = match from_value::<ClientHello>(first_frame) {
        Ok(hello) => hello,
        Err(_) => {
            let _ = send_rejection(&mut framed, RejectionCode::InvalidMessage).await;
            return None;
        }
    };

    if !hello.is_supported() {
        let _ = send_rejection(&mut framed, RejectionCode::UnsupportedVersion).await;
        return None;
    }

    let ready_sent = tokio::select! {
        biased;
        _ = wait_for_shutdown(&mut shutdown) => false,
        result = framed.send(ServerResponse::ready_v6()) => result.is_ok(),
    };
    if !ready_sent {
        return None;
    }

    // Splitting is essential: while a blocking operation owns the hardware
    // mutex, this task must still observe EOF, pipelining, and shutdown.
    let (mut writer, mut reader) = framed.split();

    loop {
        let next_frame = tokio::select! {
            biased;
            _ = wait_for_shutdown(&mut shutdown) => {
                disconnect_ready_hardware(&hardware, &initialized);
                return Some(peer_credentials);
            }
            frame = reader.next() => frame,
        };

        let request = match next_frame {
            None => {
                disconnect_ready_hardware(&hardware, &initialized);
                return Some(peer_credentials);
            }
            Some(Ok(value)) => match from_value::<Request>(value) {
                Ok(request) => request,
                Err(_) => {
                    let response = invalid_request_response();
                    if !deliver_response(&mut writer, &mut reader, response, &mut shutdown).await {
                        disconnect_ready_hardware(&hardware, &initialized);
                        return Some(peer_credentials);
                    }
                    continue;
                }
            },
            Some(Err(_)) => {
                // A broken control-plane connection never stops host cooling.
                let response = invalid_request_response();
                let _ = deliver_final_response(&mut writer, response, &mut shutdown).await;
                disconnect_ready_hardware(&hardware, &initialized);
                return Some(peer_credentials);
            }
        };

        if !*initialized.borrow() {
            let response = Response::Error {
                code: ErrorCode::Unavailable,
                message: ErrorMessage::new(INITIALIZING_MESSAGE),
            };
            if !deliver_response(&mut writer, &mut reader, response, &mut shutdown).await {
                disconnect_ready_hardware(&hardware, &initialized);
                return Some(peer_credentials);
            }
            continue;
        }
        let mut operation = spawn_operation(Arc::clone(&hardware), request, &cancellation);

        let response = tokio::select! {
            biased;
            _ = wait_for_shutdown(&mut shutdown) => {
                cancel_and_drain(&cancellation, operation).await;
                disconnect_ready_hardware(&hardware, &initialized);
                return Some(peer_credentials);
            }
            _unexpected = reader.next() => {
                cancel_and_drain(&cancellation, operation).await;
                disconnect_ready_hardware(&hardware, &initialized);
                return Some(peer_credentials);
            }
            result = &mut operation => match result {
                Ok(response) => response,
                Err(_) => {
                    disconnect_ready_hardware(&hardware, &initialized);
                    return Some(peer_credentials);
                }
            },
        };

        let response = preflight_operation_response(response);
        if !deliver_response(&mut writer, &mut reader, response, &mut shutdown).await {
            disconnect_ready_hardware(&hardware, &initialized);
            return Some(peer_credentials);
        }
    }
}

fn spawn_operation<H>(
    hardware: Arc<Mutex<H>>,
    request: Request,
    cancellation: &HardwareCancellation,
) -> JoinHandle<Response>
where
    H: HardwareOperations,
{
    // The async server owns generation changes: clear stale cancellation
    // immediately before enqueueing, never after the blocking task is queued.
    cancellation.reset();
    tokio::task::spawn_blocking(move || {
        let mut hardware = lock_hardware(&hardware);
        let result = match request {
            Request::ActivateMonitoring {
                firmware_curves,
                host_policy,
                display,
            } => hardware
                .activate_monitoring(&firmware_curves, host_policy.as_ref(), display.as_ref())
                .map(|outcomes| Response::MonitoringActivated { outcomes }),
            Request::SetMonitoringAutoResume { enabled } => hardware
                .set_monitoring_auto_resume(enabled)
                .map(|()| Response::MonitoringAutoResumeSet),
            Request::SetKrakenDisplay { device_id, mode } => hardware
                .set_kraken_display(&device_id, mode)
                .map(|()| Response::KrakenDisplaySet),
            Request::GetSnapshot => hardware
                .snapshot()
                .map(|snapshot| Response::Snapshot { snapshot }),
            Request::ApplyFirmwareCurve {
                device_id,
                channel_id,
                points,
            } => hardware
                .apply_firmware_curve(&device_id, &channel_id, &points)
                .map(|()| Response::FirmwareCurveApplied),
            Request::StartHostControl { complete_policy } => hardware
                .start_host_control(&complete_policy)
                .map(|()| Response::HostControlStarted),
            Request::UpdateHostControl { channel_policies } => hardware
                .update_host_control(&channel_policies)
                .map(|()| Response::HostControlUpdated),
            Request::StopHostControl => hardware
                .stop_host_control()
                .map(|()| Response::HostControlStopped),
        };

        result.unwrap_or_else(hardware_error_response)
    })
}

fn preflight_operation_response(response: Response) -> Response {
    match validate_frame_payload(&response) {
        Ok(()) => response,
        Err(FrameError::PayloadTooLarge { .. })
            if matches!(&response, Response::Snapshot { .. }) =>
        {
            Response::Error {
                code: ErrorCode::InvalidData,
                message: ErrorMessage::new(OVERSIZED_SNAPSHOT_MESSAGE),
            }
        }
        Err(error) => Response::Error {
            code: ErrorCode::Internal,
            message: ErrorMessage::new(format!(
                "could not serialize hardware-service response: {error}"
            )),
        },
    }
}

fn hardware_error_response(error: HardwareError) -> Response {
    let code = match error.kind() {
        HardwareErrorKind::Unavailable => ErrorCode::Unavailable,
        HardwareErrorKind::PermissionDenied => ErrorCode::PermissionDenied,
        HardwareErrorKind::Unsupported => ErrorCode::Unsupported,
        HardwareErrorKind::InvalidData => ErrorCode::InvalidData,
        HardwareErrorKind::Timeout => ErrorCode::Timeout,
        HardwareErrorKind::UnknownOutcome => ErrorCode::UnknownOutcome,
        HardwareErrorKind::RestoreRequired => ErrorCode::RestoreRequired,
        HardwareErrorKind::Internal => ErrorCode::Internal,
    };
    Response::Error {
        code,
        message: ErrorMessage::new(error.message()),
    }
}

fn invalid_request_response() -> Response {
    Response::Error {
        code: ErrorCode::InvalidData,
        message: ErrorMessage::new(INVALID_REQUEST_MESSAGE),
    }
}

/// Service lifetime only: dropping a client never signals this handle.
struct ServiceShutdownGuard {
    host: HostControlShutdownHandle,
    usb: HardwareCancellation,
}

impl ServiceShutdownGuard {
    fn request_shutdown(&self) {
        self.usb.cancel();
        self.host.request_shutdown();
    }
}

impl Drop for ServiceShutdownGuard {
    fn drop(&mut self) {
        self.request_shutdown();
    }
}

async fn cancel_and_drain(cancellation: &HardwareCancellation, operation: JoinHandle<Response>) {
    cancellation.cancel();
    let _ = operation.await;
}

async fn deliver_response<S, St>(
    writer: &mut S,
    reader: &mut St,
    response: Response,
    shutdown: &mut watch::Receiver<bool>,
) -> bool
where
    S: Sink<Response, Error = FrameError> + Unpin,
    St: Stream<Item = Result<Value, FrameError>> + Unpin,
{
    tokio::select! {
        biased;
        _ = wait_for_shutdown(shutdown) => false,
        _unexpected = reader.next() => false,
        result = writer.send(response) => result.is_ok(),
    }
}

async fn deliver_final_response<S>(
    writer: &mut S,
    response: Response,
    shutdown: &mut watch::Receiver<bool>,
) -> bool
where
    S: Sink<Response, Error = FrameError> + Unpin,
{
    tokio::select! {
        biased;
        _ = wait_for_shutdown(shutdown) => false,
        result = writer.send(response) => result.is_ok(),
    }
}

async fn wait_for_shutdown(shutdown: &mut watch::Receiver<bool>) {
    if *shutdown.borrow() {
        return;
    }
    loop {
        if shutdown.changed().await.is_err() || *shutdown.borrow() {
            return;
        }
    }
}

fn disconnect_ready_hardware<H>(hardware: &Arc<Mutex<H>>, initialized: &watch::Receiver<bool>)
where
    H: HardwareOperations,
{
    // Startup is service-owned. An early EOF or malformed frame must neither
    // cancel replay nor synchronously wait for the initializer's manager lock.
    if *initialized.borrow() {
        disconnect_hardware(hardware);
    }
}

fn disconnect_hardware<H>(hardware: &Arc<Mutex<H>>)
where
    H: HardwareOperations,
{
    match hardware.lock() {
        Ok(mut hardware) => hardware.client_disconnected(),
        Err(poisoned) => {
            // A panicking blocking operation may have changed hardware before
            // unwinding. Recover only to invalidate that unverified state.
            let mut hardware_guard = poisoned.into_inner();
            hardware_guard.client_disconnected();
            drop(hardware_guard);
            hardware.clear_poison();
        }
    }
}

fn lock_hardware<H>(hardware: &Arc<Mutex<H>>) -> MutexGuard<'_, H> {
    match hardware.lock() {
        Ok(hardware) => hardware,
        Err(_) => panic!("hardware mutex remained poisoned outside disconnect cleanup"),
    }
}

async fn reject_busy(stream: UnixStream) -> Result<(), FrameError> {
    let mut framed = Framed::new(stream, JsonFrameCodec::new());
    send_rejection(&mut framed, RejectionCode::ServiceBusy).await
}

async fn send_rejection(
    framed: &mut Framed<UnixStream, JsonFrameCodec>,
    code: RejectionCode,
) -> Result<(), FrameError> {
    framed.send(ServerResponse::rejected(code)).await?;
    SinkExt::<ServerResponse>::close(framed).await
}

#[cfg(test)]
mod tests {
    use std::{
        collections::{BTreeMap, VecDeque},
        os::{fd::AsRawFd, unix::fs::PermissionsExt},
        path::{Path, PathBuf},
        sync::{
            Condvar,
            atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
        },
        thread,
    };

    use nzxt_cam_core::{
        ChannelId, CurvePoint, Device, DeviceId, DeviceKind, HardwareSnapshot, HostChannelPolicy,
        HostControlPolicy, HostControlSnapshot, HostCurve, HostCurvePoint, HostTemperatureSource,
    };
    use nzxt_cam_protocol::{
        MAX_ERROR_MESSAGE_BYTES, MAX_FRAME_LENGTH, MonitoringActivationOutcome,
        MonitoringActivationStatus, MonitoringActivationTarget, MonitoringDisplaySelection,
        MonitoringFirmwareCurve,
    };
    use tokio::{
        sync::oneshot,
        task::{JoinHandle, yield_now},
        time,
    };

    use super::*;
    use crate::{
        config::{HostChannelConfig, HostControlConfig},
        hardware::{HardwareManager, HostStartDispatchGate},
        host_control::{
            HostControlEngine, HostControlSysfs, HostControlWorker, HostSensorSample,
            HostSensorSource, MonotonicTimeSource, RecoveryRecord, RecoveryStore, TimedTemperature,
        },
        it8689::{BOARD_NAME, BOARD_VENDOR, CHIP_ADDRESS, CHIP_NAME, PLATFORM_COMPONENT},
        monitor_intent::{
            FileMonitorIntentStore, MonitorTarget, SavedMonitorIntent, SavedTarget, WriteState,
        },
        monitor_resume::{self, Preflight, ResumeHardware},
    };

    static NEXT_SOCKET_ID: AtomicU64 = AtomicU64::new(0);

    struct SocketPath(PathBuf);

    impl SocketPath {
        fn bind() -> (Self, UnixListener) {
            let sequence = NEXT_SOCKET_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "nzxt-cam-hwd-test-{}-{sequence}.sock",
                std::process::id()
            ));
            let _ = std::fs::remove_file(&path);
            let listener = UnixListener::bind(&path).unwrap();
            (Self(path), listener)
        }

        fn as_path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for SocketPath {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    #[derive(Clone, Debug, Default)]
    struct Observed {
        snapshot_calls: usize,
        startup_preflights: Vec<DeviceId>,
        display_sets: Vec<(DeviceId, nzxt_cam_core::KrakenDisplayMode)>,
        apply_calls: Vec<(DeviceId, ChannelId, Vec<CurvePoint>)>,
        active_operations: usize,
        max_active_operations: usize,
        work_started: usize,
        cancellation_seen: bool,
        cancellation_seen_before_work: bool,
        finished_operations: usize,
        disconnects: usize,
        applied_state_valid: bool,
        host_starts: Vec<HostControlPolicy>,
        host_updates: Vec<Vec<HostChannelPolicy>>,
        host_stops: usize,
        activation_calls: Vec<(
            Vec<MonitoringFirmwareCurve>,
            Option<HostControlPolicy>,
            Option<MonitoringDisplaySelection>,
        )>,
        auto_resume_changes: Vec<bool>,
        shutdown_seen_at_disconnect: bool,
    }

    struct FakeControl {
        observed: Mutex<Observed>,
        changed: Condvar,
        block_operations: AtomicBool,
        release_after_cancel: AtomicBool,
        panic_after_apply_mutation: AtomicBool,
        host_shutdown_count: Arc<AtomicUsize>,
        initialization_calls: AtomicUsize,
        block_initialization: AtomicBool,
        fail_initialization: AtomicBool,
        panic_initialization: AtomicBool,
    }

    impl FakeControl {
        fn new(host_shutdown_count: Arc<AtomicUsize>) -> Arc<Self> {
            Arc::new(Self {
                observed: Mutex::new(Observed::default()),
                changed: Condvar::new(),
                block_operations: AtomicBool::new(false),
                release_after_cancel: AtomicBool::new(false),
                panic_after_apply_mutation: AtomicBool::new(false),
                host_shutdown_count,
                initialization_calls: AtomicUsize::new(0),
                block_initialization: AtomicBool::new(false),
                fail_initialization: AtomicBool::new(false),
                panic_initialization: AtomicBool::new(false),
            })
        }

        fn snapshot(&self) -> Observed {
            self.observed.lock().unwrap().clone()
        }

        fn block(&self) {
            self.block_operations.store(true, Ordering::Release);
        }

        fn release(&self) {
            self.block_operations.store(false, Ordering::Release);
            self.release_after_cancel.store(true, Ordering::Release);
            self.changed.notify_all();
        }

        fn unblock_without_cancellation(&self) {
            self.block_operations.store(false, Ordering::Release);
            self.changed.notify_all();
        }

        fn panic_after_apply_mutation(&self) {
            self.panic_after_apply_mutation
                .store(true, Ordering::Release);
        }
    }

    struct FakeHardware {
        control: Arc<FakeControl>,
        cancellation: HardwareCancellation,
        snapshot: HardwareSnapshot,
        queued_snapshots: VecDeque<HardwareSnapshot>,
        errors: VecDeque<HardwareError>,
        host_shutdown: HostControlShutdownHandle,
        startup_store: Option<FileMonitorIntentStore>,
    }

    impl FakeHardware {
        fn new() -> (Self, Arc<FakeControl>) {
            let (host_shutdown, count) = HostControlShutdownHandle::test_handle();
            let control = FakeControl::new(count);
            (
                Self {
                    control: Arc::clone(&control),
                    cancellation: HardwareCancellation::default(),
                    snapshot: HardwareSnapshot {
                        devices: Vec::new(),
                        sequence: 42,
                        host_control: HostControlSnapshot::disabled(),
                        kraken_display: Default::default(),
                        monitoring: Default::default(),
                    },
                    queued_snapshots: VecDeque::new(),
                    errors: VecDeque::new(),
                    host_shutdown,
                    startup_store: None,
                },
                control,
            )
        }

        fn with_errors(
            errors: impl IntoIterator<Item = HardwareError>,
        ) -> (Self, Arc<FakeControl>) {
            let (mut hardware, control) = Self::new();
            hardware.errors = errors.into_iter().collect();
            (hardware, control)
        }

        fn run_operation(&self) -> bool {
            if self.cancellation.is_cancelled() {
                let mut observed = self.control.observed.lock().unwrap();
                observed.cancellation_seen = true;
                observed.cancellation_seen_before_work = true;
                observed.finished_operations += 1;
                self.control.changed.notify_all();
                return false;
            }

            {
                let mut observed = self.control.observed.lock().unwrap();
                observed.active_operations += 1;
                observed.max_active_operations = observed
                    .max_active_operations
                    .max(observed.active_operations);
                observed.work_started += 1;
                self.control.changed.notify_all();
            }

            if self.control.block_operations.load(Ordering::Acquire) {
                while self.control.block_operations.load(Ordering::Acquire)
                    && !self.cancellation.is_cancelled()
                {
                    thread::sleep(Duration::from_millis(2));
                }
                if self.cancellation.is_cancelled() {
                    {
                        let mut observed = self.control.observed.lock().unwrap();
                        observed.cancellation_seen = true;
                        self.control.changed.notify_all();
                    }
                    let mut observed = self.control.observed.lock().unwrap();
                    while !self.control.release_after_cancel.load(Ordering::Acquire) {
                        observed = self
                            .control
                            .changed
                            .wait_timeout(observed, Duration::from_millis(5))
                            .unwrap()
                            .0;
                    }
                }
            }

            let mut observed = self.control.observed.lock().unwrap();
            observed.active_operations -= 1;
            observed.finished_operations += 1;
            self.control.changed.notify_all();
            true
        }
    }

    #[derive(Debug)]
    struct RealWorkerEffects {
        sysfs_operations: Vec<String>,
        recovery_persists: usize,
        recovery_removes: usize,
        recovery_record: Option<RecoveryRecord>,
        pwm: BTreeMap<u8, u8>,
        enable: BTreeMap<u8, u8>,
        fail_bios_channel: Option<u8>,
        cpu_millidegrees: i64,
    }

    impl Default for RealWorkerEffects {
        fn default() -> Self {
            Self {
                sysfs_operations: Vec::new(),
                recovery_persists: 0,
                recovery_removes: 0,
                recovery_record: None,
                pwm: BTreeMap::from([(3, 63), (4, 63)]),
                enable: BTreeMap::from([(3, 2), (4, 2)]),
                fail_bios_channel: None,
                cpu_millidegrees: 40_000,
            }
        }
    }

    struct RealWorkerSysfs(Arc<Mutex<RealWorkerEffects>>);

    impl HostControlSysfs for RealWorkerSysfs {
        fn discover_exact(&mut self) -> Result<(), HardwareError> {
            self.0
                .lock()
                .unwrap()
                .sysfs_operations
                .push("discover".into());
            Ok(())
        }

        fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError> {
            let mut effects = self.0.lock().unwrap();
            effects.sysfs_operations.push(format!("read_pwm:{channel}"));
            Ok(effects.pwm[&channel])
        }

        fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError> {
            self.0
                .lock()
                .unwrap()
                .sysfs_operations
                .push(format!("read_fan:{channel}"));
            Ok(900)
        }

        fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError> {
            let mut effects = self.0.lock().unwrap();
            effects
                .sysfs_operations
                .push(format!("read_enable:{channel}"));
            Ok(effects.enable[&channel])
        }

        fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            let mut effects = self.0.lock().unwrap();
            effects
                .sysfs_operations
                .push(format!("write_pwm:{channel}:{value}"));
            assert!(
                effects.recovery_record.is_some(),
                "write before recovery persistence"
            );
            assert_eq!(effects.enable[&channel], 1, "PWM write outside manual mode");
            effects.pwm.insert(channel, value);
            Ok(())
        }

        fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            let mut effects = self.0.lock().unwrap();
            effects
                .sysfs_operations
                .push(format!("write_enable:{channel}:{value}"));
            assert!(
                effects.recovery_record.is_some(),
                "write before recovery persistence"
            );
            if value == 2 && effects.fail_bios_channel == Some(channel) {
                return Err(HardwareError::new("injected BIOS restoration failure"));
            }
            effects.enable.insert(channel, value);
            Ok(())
        }
    }

    struct RealWorkerRecovery(Arc<Mutex<RealWorkerEffects>>);

    impl RecoveryStore for RealWorkerRecovery {
        fn load(&mut self) -> Result<Option<RecoveryRecord>, HardwareError> {
            Ok(self.0.lock().unwrap().recovery_record.clone())
        }

        fn persist(&mut self, record: &RecoveryRecord) -> Result<(), HardwareError> {
            let mut effects = self.0.lock().unwrap();
            effects.recovery_persists += 1;
            effects.recovery_record = Some(record.clone());
            Ok(())
        }

        fn remove(&mut self) -> Result<(), HardwareError> {
            let mut effects = self.0.lock().unwrap();
            effects.recovery_removes += 1;
            effects.recovery_record = None;
            Ok(())
        }
    }

    struct RealWorkerSensors(Arc<Mutex<RealWorkerEffects>>);

    impl HostSensorSource for RealWorkerSensors {
        fn sample(&mut self, now: Duration) -> HostSensorSample {
            HostSensorSample {
                cpu: Some(TimedTemperature {
                    millidegrees: self.0.lock().unwrap().cpu_millidegrees,
                    sampled_at: now,
                }),
                gpu: None,
            }
        }
    }

    struct RealWorkerClock;

    impl MonotonicTimeSource for RealWorkerClock {
        fn now(&mut self) -> Result<Duration, HardwareError> {
            Ok(Duration::ZERO)
        }
    }

    fn real_host_worker() -> (HostControlWorker, Arc<Mutex<RealWorkerEffects>>) {
        real_host_worker_channels(&[3])
    }

    fn real_host_worker_channels(
        channels: &[u8],
    ) -> (HostControlWorker, Arc<Mutex<RealWorkerEffects>>) {
        let effects = Arc::new(Mutex::new(RealWorkerEffects::default()));
        let config = HostControlConfig {
            board_vendor: BOARD_VENDOR.into(),
            board_name: BOARD_NAME.into(),
            chip_name: CHIP_NAME.into(),
            chip_address: CHIP_ADDRESS,
            platform_component: PLATFORM_COMPONENT.into(),
            channels: channels
                .iter()
                .map(|&channel| HostChannelConfig {
                    id: ChannelId::new(if channel == 3 {
                        "case-fan"
                    } else {
                        "case-fan4"
                    }),
                    name: format!("Case fan{channel}"),
                    pwm_channel: channel,
                    fan_channel: channel,
                    minimum_duty_percent: 30,
                })
                .collect(),
        };
        let engine = HostControlEngine::with_dependencies(
            Some(config),
            None,
            Box::new(RealWorkerSysfs(Arc::clone(&effects))),
            Box::new(RealWorkerRecovery(Arc::clone(&effects))),
            Box::new(RealWorkerSensors(Arc::clone(&effects))),
            Box::new(RealWorkerClock),
        );
        (
            HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3_600)),
            effects,
        )
    }

    impl ResumeHardware for FakeHardware {
        fn preflight(
            &mut self,
            id: &DeviceId,
            _: &ChannelId,
            _: &[CurvePoint],
        ) -> Result<Preflight, HardwareError> {
            self.control
                .observed
                .lock()
                .unwrap()
                .startup_preflights
                .push(id.clone());
            Ok(Preflight::Ready)
        }

        fn write_curve(
            &mut self,
            id: &DeviceId,
            channel: &ChannelId,
            points: &[CurvePoint],
        ) -> Result<(), HardwareError> {
            self.apply_firmware_curve(id, channel, points)
        }
    }

    impl HardwareOperations for FakeHardware {
        fn initialize_host_control(&mut self) -> Result<(), HardwareError> {
            self.control
                .initialization_calls
                .fetch_add(1, Ordering::AcqRel);
            while self.control.block_initialization.load(Ordering::Acquire)
                && self.control.host_shutdown_count.load(Ordering::Acquire) == 0
            {
                thread::sleep(Duration::from_millis(2));
            }
            assert!(
                !self.control.panic_initialization.load(Ordering::Acquire),
                "fake initialization panic"
            );
            if self.control.fail_initialization.load(Ordering::Acquire) {
                self.snapshot.host_control.state = nzxt_cam_core::HostControlState::RestoreRequired;
                Err(HardwareError::with_kind(
                    HardwareErrorKind::RestoreRequired,
                    "fake resume blocked",
                ))
            } else {
                if let Some(mut store) = self.startup_store.take() {
                    let cancellation = self.cancellation.clone();
                    let selection = monitor_resume::boot(
                        &mut store,
                        self,
                        |_| Ok(()),
                        || {},
                        || check_startup_cancellation(&cancellation),
                    )?;
                    check_startup_cancellation(&cancellation)?;
                    if let Some((id, mode)) = selection.restored_display {
                        self.set_kraken_display(&id, mode)?;
                    }
                }
                Ok(())
            }
        }

        fn snapshot(&mut self) -> Result<HardwareSnapshot, HardwareError> {
            {
                let mut observed = self.control.observed.lock().unwrap();
                observed.snapshot_calls += 1;
                self.control.changed.notify_all();
            }
            self.run_operation();
            if let Some(error) = self.errors.pop_front() {
                Err(error)
            } else {
                Ok(self
                    .queued_snapshots
                    .pop_front()
                    .unwrap_or_else(|| self.snapshot.clone()))
            }
        }

        fn activate_monitoring(
            &mut self,
            curves: &[MonitoringFirmwareCurve],
            host_policy: Option<&HostControlPolicy>,
            display: Option<&MonitoringDisplaySelection>,
        ) -> Result<Vec<MonitoringActivationOutcome>, HardwareError> {
            self.control
                .observed
                .lock()
                .unwrap()
                .activation_calls
                .push((curves.to_vec(), host_policy.cloned(), display.cloned()));
            self.snapshot.monitoring.opted_in = true;
            self.snapshot.monitoring.auto_resume = true;
            let mut outcomes = curves
                .iter()
                .map(|curve| MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::FirmwareCurve {
                        device_id: curve.device_id.clone(),
                        channel_id: curve.channel_id.clone(),
                    },
                    status: MonitoringActivationStatus::Applied,
                    error: None,
                })
                .collect::<Vec<_>>();
            if let Some(display) = display {
                outcomes.push(MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::Display {
                        device_id: display.device_id.clone(),
                    },
                    status: MonitoringActivationStatus::Pending,
                    error: None,
                });
            }
            if host_policy.is_some() {
                outcomes.push(MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::HostControl {},
                    status: MonitoringActivationStatus::Failed,
                    error: Some(ErrorMessage::new("host unavailable")),
                });
            }
            Ok(outcomes)
        }

        fn set_monitoring_auto_resume(&mut self, enabled: bool) -> Result<(), HardwareError> {
            self.control
                .observed
                .lock()
                .unwrap()
                .auto_resume_changes
                .push(enabled);
            if !self.snapshot.monitoring.opted_in {
                return Err(HardwareError::with_kind(
                    HardwareErrorKind::RestoreRequired,
                    "not opted in",
                ));
            }
            self.snapshot.monitoring.auto_resume = enabled;
            Ok(())
        }

        fn set_kraken_display(
            &mut self,
            device_id: &DeviceId,
            mode: nzxt_cam_core::KrakenDisplayMode,
        ) -> Result<(), HardwareError> {
            let mut observed = self.control.observed.lock().unwrap();
            observed.display_sets.push((device_id.clone(), mode));
            self.snapshot.kraken_display.device_id = Some(device_id.clone());
            self.snapshot.kraken_display.mode = mode;
            self.control.changed.notify_all();
            Ok(())
        }

        fn apply_firmware_curve(
            &mut self,
            device_id: &DeviceId,
            channel_id: &ChannelId,
            points: &[CurvePoint],
        ) -> Result<(), HardwareError> {
            {
                let mut observed = self.control.observed.lock().unwrap();
                observed
                    .apply_calls
                    .push((device_id.clone(), channel_id.clone(), points.to_vec()));
                self.control.changed.notify_all();
            }
            if self.run_operation() {
                self.control.observed.lock().unwrap().applied_state_valid = true;
                assert!(
                    !self
                        .control
                        .panic_after_apply_mutation
                        .load(Ordering::Acquire),
                    "fake apply panic after mutating applied state"
                );
            }
            if let Some(error) = self.errors.pop_front() {
                Err(error)
            } else {
                Ok(())
            }
        }

        fn start_host_control(&mut self, policy: &HostControlPolicy) -> Result<(), HardwareError> {
            // Accepted host commands are service-owned, not cancelled on EOF.
            let mut observed = self.control.observed.lock().unwrap();
            observed.host_starts.push(policy.clone());
            observed.finished_operations += 1;
            self.control.changed.notify_all();
            if let Some(error) = self.errors.pop_front() {
                Err(error)
            } else {
                Ok(())
            }
        }

        fn update_host_control(
            &mut self,
            channel_policies: &[HostChannelPolicy],
        ) -> Result<(), HardwareError> {
            let mut observed = self.control.observed.lock().unwrap();
            observed.host_updates.push(channel_policies.to_vec());
            observed.finished_operations += 1;
            self.control.changed.notify_all();
            self.errors.pop_front().map_or(Ok(()), Err)
        }

        fn stop_host_control(&mut self) -> Result<(), HardwareError> {
            let mut observed = self.control.observed.lock().unwrap();
            observed.host_stops += 1;
            self.control.changed.notify_all();
            if let Some(error) = self.errors.pop_front() {
                Err(error)
            } else {
                Ok(())
            }
        }

        fn host_control_snapshot(&self) -> HostControlSnapshot {
            self.snapshot.host_control.clone()
        }

        fn host_control_shutdown_handle(&self) -> HostControlShutdownHandle {
            self.host_shutdown.clone()
        }

        fn cancellation_handle(&self) -> HardwareCancellation {
            self.cancellation.clone()
        }

        fn client_disconnected(&mut self) {
            let mut observed = self.control.observed.lock().unwrap();
            observed.disconnects += 1;
            observed.applied_state_valid = false;
            observed.shutdown_seen_at_disconnect =
                self.control.host_shutdown_count.load(Ordering::Acquire) > 0;
            self.control.changed.notify_all();
        }
    }

    fn start_initializing_service<H>(
        listener: UnixListener,
        hardware: H,
    ) -> (
        oneshot::Sender<()>,
        JoinHandle<Result<(), ServiceError>>,
        watch::Receiver<bool>,
    )
    where
        H: HardwareOperations,
    {
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let (initialized_tx, initialized_rx) = watch::channel(false);
        let service = tokio::spawn(async move {
            serve_with_readiness(
                listener,
                hardware,
                async {
                    let _ = shutdown_rx.await;
                },
                initialized_tx,
            )
            .await
        });
        (shutdown_tx, service, initialized_rx)
    }

    async fn start_service<H>(
        listener: UnixListener,
        hardware: H,
    ) -> (oneshot::Sender<()>, JoinHandle<Result<(), ServiceError>>)
    where
        H: HardwareOperations,
    {
        let (shutdown, service, mut initialized) = start_initializing_service(listener, hardware);
        initialized.wait_for(|ready| *ready).await.unwrap();
        (shutdown, service)
    }

    async fn connect(path: &Path) -> Framed<UnixStream, JsonFrameCodec> {
        Framed::new(
            UnixStream::connect(path).await.unwrap(),
            JsonFrameCodec::new(),
        )
    }

    async fn receive<T>(client: &mut Framed<UnixStream, JsonFrameCodec>) -> T
    where
        T: serde::de::DeserializeOwned,
    {
        let value = client.next().await.unwrap().unwrap();
        from_value(value).unwrap()
    }

    async fn connect_ready(path: &Path) -> Framed<UnixStream, JsonFrameCodec> {
        let mut client = connect(path).await;
        client.send(ClientHello::v6()).await.unwrap();
        assert_eq!(
            receive::<ServerResponse>(&mut client).await,
            ServerResponse::ready_v6()
        );
        client
    }

    async fn reconnect_ready(path: &Path) -> Framed<UnixStream, JsonFrameCodec> {
        time::timeout(Duration::from_secs(1), async {
            loop {
                let mut candidate = connect(path).await;
                candidate.send(ClientHello::v6()).await.unwrap();
                match receive::<ServerResponse>(&mut candidate).await {
                    response if response == ServerResponse::ready_v6() => return candidate,
                    ServerResponse::Rejected {
                        code: RejectionCode::ServiceBusy,
                    } => yield_now().await,
                    response => panic!("unexpected reconnect response: {response:?}"),
                }
            }
        })
        .await
        .expect("previous control-plane connection did not drain")
    }

    async fn stop_service(
        shutdown: oneshot::Sender<()>,
        service: JoinHandle<Result<(), ServiceError>>,
    ) {
        shutdown.send(()).unwrap();
        time::timeout(Duration::from_secs(1), service)
            .await
            .unwrap()
            .unwrap()
            .unwrap();
    }

    fn check_startup_cancellation(
        cancellation: &HardwareCancellation,
    ) -> Result<(), HardwareError> {
        if cancellation.is_cancelled() {
            Err(HardwareError::service_shutdown())
        } else {
            Ok(())
        }
    }

    async fn wait_for_initialization_started(control: &FakeControl) {
        time::timeout(Duration::from_secs(1), async {
            while control.initialization_calls.load(Ordering::Acquire) == 0 {
                yield_now().await;
            }
        })
        .await
        .unwrap();
    }

    async fn wait_for_observed(
        control: &FakeControl,
        predicate: impl Fn(&Observed) -> bool,
    ) -> Observed {
        for _ in 0..500 {
            let observed = control.snapshot();
            if predicate(&observed) {
                return observed;
            }
            time::sleep(Duration::from_millis(2)).await;
        }
        panic!(
            "timed out waiting for fake hardware state: {:?}",
            control.snapshot()
        );
    }

    fn host_policy() -> HostControlPolicy {
        HostControlPolicy {
            channels: vec![HostChannelPolicy {
                channel_id: ChannelId::new("case-fan"),
                curve: HostCurve {
                    source: HostTemperatureSource::Cpu,
                    points: vec![
                        HostCurvePoint {
                            temperature_millidegrees: 20_000,
                            duty_percent: 30,
                        },
                        HostCurvePoint {
                            temperature_millidegrees: 80_000,
                            duty_percent: 100,
                        },
                    ],
                },
            }],
        }
    }

    fn apply_request() -> Request {
        Request::ApplyFirmwareCurve {
            device_id: DeviceId::new("kraken-serial"),
            channel_id: ChannelId::new("pump"),
            points: vec![
                CurvePoint {
                    temperature: 20,
                    duty: 40,
                },
                CurvePoint {
                    temperature: 59,
                    duty: 100,
                },
            ],
        }
    }

    #[tokio::test]
    async fn activation_partial_reply_and_future_auto_resume_cross_socket() {
        let (path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(path.as_path()).await;
        client
            .send(Request::SetMonitoringAutoResume { enabled: false })
            .await
            .unwrap();
        assert!(matches!(
            receive::<Response>(&mut client).await,
            Response::Error {
                code: ErrorCode::RestoreRequired,
                ..
            }
        ));
        let curve = MonitoringFirmwareCurve {
            device_id: DeviceId::new("aio"),
            channel_id: ChannelId::new("pump"),
            points: (20..60)
                .map(|temperature| CurvePoint {
                    temperature,
                    duty: if temperature == 59 { 100 } else { 45 },
                })
                .collect(),
        };
        let display = MonitoringDisplaySelection {
            device_id: DeviceId::new("lcd"),
            mode: nzxt_cam_core::KrakenDisplayMode::Cpu,
        };
        let policy = host_policy();
        client
            .send(Request::ActivateMonitoring {
                firmware_curves: vec![curve.clone()],
                host_policy: Some(policy.clone()),
                display: Some(display.clone()),
            })
            .await
            .unwrap();
        let Response::MonitoringActivated { outcomes } = receive::<Response>(&mut client).await
        else {
            panic!("missing activation response")
        };
        assert_eq!(
            outcomes.iter().map(|o| o.status).collect::<Vec<_>>(),
            vec![
                MonitoringActivationStatus::Applied,
                MonitoringActivationStatus::Pending,
                MonitoringActivationStatus::Failed
            ]
        );
        client
            .send(Request::SetMonitoringAutoResume { enabled: false })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::MonitoringAutoResumeSet
        );
        client.send(Request::GetSnapshot).await.unwrap();
        let Response::Snapshot { snapshot } = receive::<Response>(&mut client).await else {
            panic!("missing snapshot")
        };
        assert!(snapshot.monitoring.opted_in);
        assert!(!snapshot.monitoring.auto_resume);
        assert_eq!(
            control.snapshot().activation_calls,
            vec![(vec![curve], Some(policy), Some(display))]
        );
        assert_eq!(control.snapshot().auto_resume_changes, vec![false, false]);
        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn snapshot_round_trip() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        client.send(Request::GetSnapshot).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::Snapshot {
                snapshot: HardwareSnapshot {
                    devices: Vec::new(),
                    sequence: 42,
                    host_control: Default::default(),
                    kraken_display: Default::default(),
                    monitoring: Default::default(),
                }
            }
        );
        assert_eq!(control.snapshot().snapshot_calls, 1);

        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn oversized_snapshot_returns_finite_error_and_keeps_connection_usable() {
        let (socket_path, listener) = SocketPath::bind();
        let (mut hardware, control) = FakeHardware::new();
        hardware.queued_snapshots = VecDeque::from([
            HardwareSnapshot {
                devices: vec![Device {
                    id: DeviceId::new("oversized"),
                    name: "x".repeat(MAX_FRAME_LENGTH),
                    model: "test".into(),
                    kind: DeviceKind::LiquidCooler,
                    online: true,
                    readings: Vec::new(),
                    cooling_channels: Vec::new(),
                }],
                sequence: 1,
                host_control: Default::default(),
                kraken_display: Default::default(),
                monitoring: Default::default(),
            },
            HardwareSnapshot {
                devices: Vec::new(),
                sequence: 2,
                host_control: Default::default(),
                kraken_display: Default::default(),
                monitoring: Default::default(),
            },
        ]);
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        client.send(Request::GetSnapshot).await.unwrap();
        let Response::Error { code, message } = receive::<Response>(&mut client).await else {
            panic!("expected bounded oversized-snapshot error")
        };
        assert_eq!(code, ErrorCode::InvalidData);
        assert_eq!(message.as_str(), OVERSIZED_SNAPSHOT_MESSAGE);
        assert!(message.as_str().len() <= MAX_ERROR_MESSAGE_BYTES);

        client.send(Request::GetSnapshot).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::Snapshot {
                snapshot: HardwareSnapshot {
                    devices: Vec::new(),
                    sequence: 2,
                    host_control: Default::default(),
                    kraken_display: Default::default(),
                    monitoring: Default::default(),
                }
            }
        );
        let observed = control.snapshot();
        assert_eq!(observed.snapshot_calls, 2);
        assert_eq!(observed.disconnects, 0);

        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn apply_passes_exact_target_and_points_and_reports_success() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        let request = apply_request();

        client.send(request.clone()).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::FirmwareCurveApplied
        );
        let Request::ApplyFirmwareCurve {
            device_id,
            channel_id,
            points,
        } = request
        else {
            unreachable!()
        };
        assert_eq!(
            control.snapshot().apply_calls,
            vec![(device_id, channel_id, points)]
        );

        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn response_delivery_failure_does_not_request_host_shutdown() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        client.send(Request::GetSnapshot).await.unwrap();
        wait_for_observed(&control, |state| state.active_operations == 1).await;
        assert_eq!(
            unsafe { libc::shutdown(client.get_ref().as_raw_fd(), libc::SHUT_RD) },
            0
        );
        control.unblock_without_cancellation();

        let observed = wait_for_observed(&control, |state| state.disconnects == 1).await;
        assert!(!observed.shutdown_seen_at_disconnect);
        assert_eq!(control.host_shutdown_count.load(Ordering::Acquire), 0);
        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn host_control_requests_are_sequential_and_preserve_the_exact_policy() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        let policy = host_policy();

        client
            .send(Request::StartHostControl {
                complete_policy: policy.clone(),
            })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlStarted
        );
        let mut update = policy.channels[0].clone();
        update.curve.points[0].duty_percent = 50;
        let mut other = update.clone();
        other.channel_id = ChannelId::new("other-group");
        let batch = vec![update, other];
        client
            .send(Request::UpdateHostControl {
                channel_policies: batch.clone(),
            })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlUpdated
        );
        client.send(Request::GetSnapshot).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::Snapshot {
                snapshot: HardwareSnapshot {
                    devices: Vec::new(),
                    sequence: 42,
                    host_control: HostControlSnapshot::disabled(),
                    kraken_display: Default::default(),
                    monitoring: Default::default(),
                },
            }
        );
        client.send(Request::StopHostControl).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlStopped
        );

        let observed = control.snapshot();
        assert_eq!(observed.host_starts, vec![policy]);
        assert_eq!(observed.host_updates, vec![batch]);
        assert_eq!(observed.snapshot_calls, 1);
        assert_eq!(observed.host_stops, 1);
        assert_eq!(observed.max_active_operations, 1);
        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn every_hardware_error_kind_maps_exactly_and_messages_are_bounded() {
        let mappings = [
            (HardwareErrorKind::Unavailable, ErrorCode::Unavailable),
            (
                HardwareErrorKind::PermissionDenied,
                ErrorCode::PermissionDenied,
            ),
            (HardwareErrorKind::Unsupported, ErrorCode::Unsupported),
            (HardwareErrorKind::InvalidData, ErrorCode::InvalidData),
            (HardwareErrorKind::Timeout, ErrorCode::Timeout),
            (HardwareErrorKind::UnknownOutcome, ErrorCode::UnknownOutcome),
            (
                HardwareErrorKind::RestoreRequired,
                ErrorCode::RestoreRequired,
            ),
            (HardwareErrorKind::Internal, ErrorCode::Internal),
        ];
        let errors = mappings.iter().map(|(kind, _)| {
            HardwareError::with_kind(*kind, "x".repeat(MAX_ERROR_MESSAGE_BYTES + 20))
        });
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, _control) = FakeHardware::with_errors(errors);
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        for (_, expected_code) in mappings {
            client.send(Request::GetSnapshot).await.unwrap();
            let Response::Error { code, message } = receive::<Response>(&mut client).await else {
                panic!("expected operation error")
            };
            assert_eq!(code, expected_code);
            assert_eq!(message.as_str().len(), MAX_ERROR_MESSAGE_BYTES);
        }

        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn malformed_request_returns_invalid_data_without_hardware_call() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        client
            .send(serde_json::json!({"type": "get_snapshot", "unexpected": true}))
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::Error {
                code: ErrorCode::InvalidData,
                message: ErrorMessage::new(INVALID_REQUEST_MESSAGE),
            }
        );
        assert_eq!(control.snapshot().snapshot_calls, 0);

        client.send(Request::GetSnapshot).await.unwrap();
        assert!(matches!(
            receive::<Response>(&mut client).await,
            Response::Snapshot { .. }
        ));
        assert_eq!(control.snapshot().snapshot_calls, 1);

        // A syntactically invalid JSON payload receives the same operation
        // error before the codec-terminated connection is closed.
        let malformed_json = [0, 0, 0, 1, b'{'];
        client.get_mut().writable().await.unwrap();
        assert_eq!(
            client.get_mut().try_write(&malformed_json).unwrap(),
            malformed_json.len()
        );
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::Error {
                code: ErrorCode::InvalidData,
                message: ErrorMessage::new(INVALID_REQUEST_MESSAGE),
            }
        );
        assert!(client.next().await.is_none());
        let observed = wait_for_observed(&control, |state| state.disconnects == 1).await;
        assert_eq!(observed.snapshot_calls, 1);
        assert!(!observed.shutdown_seen_at_disconnect);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn operations_are_sequential_on_a_persistent_connection() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        client.send(Request::GetSnapshot).await.unwrap();
        assert!(matches!(
            receive::<Response>(&mut client).await,
            Response::Snapshot { .. }
        ));
        client.send(apply_request()).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::FirmwareCurveApplied
        );

        let observed = control.snapshot();
        assert_eq!(observed.finished_operations, 2);
        assert_eq!(observed.max_active_operations, 1);
        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn second_connection_is_busy_during_a_long_operation() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut first = connect_ready(socket_path.as_path()).await;
        first.send(apply_request()).await.unwrap();
        wait_for_observed(&control, |state| state.active_operations == 1).await;

        let mut second = connect(socket_path.as_path()).await;
        assert_eq!(
            receive::<ServerResponse>(&mut second).await,
            ServerResponse::rejected(RejectionCode::ServiceBusy)
        );
        assert!(second.next().await.is_none());

        drop(first);
        wait_for_observed(&control, |state| state.cancellation_seen).await;
        control.release();
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn eof_during_long_apply_cancels_and_drains_before_releasing_slot() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut first = connect_ready(socket_path.as_path()).await;
        first.send(apply_request()).await.unwrap();
        wait_for_observed(&control, |state| state.active_operations == 1).await;
        drop(first);
        wait_for_observed(&control, |state| state.cancellation_seen).await;
        assert_eq!(control.host_shutdown_count.load(Ordering::Acquire), 0);

        let mut while_draining = connect(socket_path.as_path()).await;
        assert_eq!(
            receive::<ServerResponse>(&mut while_draining).await,
            ServerResponse::rejected(RejectionCode::ServiceBusy)
        );
        control.release();
        wait_for_observed(&control, |state| state.disconnects == 1).await;

        let next = connect_ready(socket_path.as_path()).await;
        drop(next);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn shutdown_during_long_operation_cancels_and_drains() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block();
        let (shutdown, mut service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        client.send(apply_request()).await.unwrap();
        wait_for_observed(&control, |state| state.active_operations == 1).await;

        shutdown.send(()).unwrap();
        wait_for_observed(&control, |state| state.cancellation_seen).await;
        assert!(
            time::timeout(Duration::from_millis(25), &mut service)
                .await
                .is_err()
        );
        control.release();
        time::timeout(Duration::from_secs(1), service)
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let observed = control.snapshot();
        assert_eq!(observed.disconnects, 1);
        assert!(observed.shutdown_seen_at_disconnect);
    }

    #[tokio::test]
    async fn disconnect_after_apply_invalidates_applied_state() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        client.send(apply_request()).await.unwrap();
        wait_for_observed(&control, |state| state.active_operations == 1).await;
        drop(client);
        wait_for_observed(&control, |state| state.cancellation_seen).await;
        control.release();

        let observed = wait_for_observed(&control, |state| state.disconnects == 1).await;
        assert!(!observed.applied_state_valid);
        assert_eq!(observed.finished_operations, 1);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn pipelined_request_cancels_and_drains_the_active_operation() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        client.send(apply_request()).await.unwrap();
        wait_for_observed(&control, |state| state.active_operations == 1).await;
        client.send(Request::GetSnapshot).await.unwrap();
        wait_for_observed(&control, |state| state.cancellation_seen).await;
        control.release();

        assert!(client.next().await.is_none());
        let observed = wait_for_observed(&control, |state| state.disconnects == 1).await;
        assert_eq!(observed.snapshot_calls, 0);
        assert!(!observed.shutdown_seen_at_disconnect);
        stop_service(shutdown, service).await;
    }

    #[test]
    fn queued_operation_keeps_cancellation_until_the_blocking_worker_starts() {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .max_blocking_threads(1)
            .enable_all()
            .build()
            .unwrap();
        let (blocker_started_tx, blocker_started_rx) = std::sync::mpsc::sync_channel(0);
        let (blocker_release_tx, blocker_release_rx) = std::sync::mpsc::sync_channel(0);
        let mut blocker_release_tx = Some(blocker_release_tx);

        let result = runtime.block_on(async {
            time::timeout(Duration::from_secs(5), async {
                let (socket_path, listener) = SocketPath::bind();
                let (hardware, control) = FakeHardware::new();
                let cancellation = hardware.cancellation_handle();
                let (shutdown, service) = start_service(listener, hardware).await;
                let mut client = connect_ready(socket_path.as_path()).await;

                // Bootstrap completes before occupying the blocking pool.
                let blocker = tokio::task::spawn_blocking(move || {
                    blocker_started_tx.send(()).unwrap();
                    let _ = blocker_release_rx.recv();
                });
                blocker_started_rx
                    .recv_timeout(Duration::from_secs(1))
                    .unwrap();
                client.send(apply_request()).await.unwrap();
                client.send(Request::GetSnapshot).await.unwrap();
                time::timeout(Duration::from_secs(1), async {
                    while !cancellation.is_cancelled() {
                        yield_now().await;
                    }
                })
                .await
                .expect("pipelining did not cancel the queued operation");

                blocker_release_tx.take().unwrap().send(()).unwrap();
                blocker.await.unwrap();

                let observed =
                    wait_for_observed(&control, |state| state.finished_operations == 1).await;
                assert!(observed.cancellation_seen_before_work);
                assert_eq!(observed.work_started, 0);
                assert!(!observed.applied_state_valid);
                assert!(
                    time::timeout(Duration::from_secs(1), client.next())
                        .await
                        .expect("cancelled connection did not close")
                        .is_none()
                );
                wait_for_observed(&control, |state| state.disconnects == 1).await;
                stop_service(shutdown, service).await;
            })
            .await
        });

        if let Some(blocker_release_tx) = blocker_release_tx {
            let _ = blocker_release_tx.send(());
        }
        result.expect("queued-cancellation regression test timed out");
    }

    #[test]
    fn accepted_host_start_after_disconnect_still_reaches_the_worker() {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .max_blocking_threads(1)
            .enable_all()
            .build()
            .unwrap();
        let (blocker_started_tx, blocker_started_rx) = std::sync::mpsc::sync_channel(0);
        let (blocker_release_tx, blocker_release_rx) = std::sync::mpsc::sync_channel(0);
        let mut blocker_release_tx = Some(blocker_release_tx);

        let result = runtime.block_on(async {
            time::timeout(Duration::from_secs(5), async {
                let (socket_path, listener) = SocketPath::bind();
                let (hardware, control) = FakeHardware::new();
                let cancellation = hardware.cancellation_handle();
                let (shutdown, service) = start_service(listener, hardware).await;
                let mut client = connect_ready(socket_path.as_path()).await;

                let blocker = tokio::task::spawn_blocking(move || {
                    blocker_started_tx.send(()).unwrap();
                    let _ = blocker_release_rx.recv();
                });
                blocker_started_rx
                    .recv_timeout(Duration::from_secs(1))
                    .unwrap();
                client
                    .send(Request::StartHostControl {
                        complete_policy: host_policy(),
                    })
                    .await
                    .unwrap();
                client.send(Request::GetSnapshot).await.unwrap();
                time::timeout(Duration::from_secs(1), async {
                    while !cancellation.is_cancelled() {
                        yield_now().await;
                    }
                })
                .await
                .expect("disconnect did not cancel the queued Start");

                blocker_release_tx.take().unwrap().send(()).unwrap();
                blocker.await.unwrap();
                wait_for_observed(&control, |state| state.finished_operations == 1).await;
                let observed = control.snapshot();
                assert!(!observed.cancellation_seen_before_work);
                assert_eq!(observed.host_starts, vec![host_policy()]);
                assert_eq!(control.host_shutdown_count.load(Ordering::Acquire), 0);
                assert!(
                    time::timeout(Duration::from_secs(1), client.next())
                        .await
                        .expect("cancelled connection did not close")
                        .is_none()
                );
                stop_service(shutdown, service).await;
            })
            .await
        });

        if let Some(blocker_release_tx) = blocker_release_tx {
            let _ = blocker_release_tx.send(());
        }
        result.expect("queued Start cancellation test timed out");
    }

    #[tokio::test]
    async fn disconnected_start_finishes_before_replacement_client_can_stop_it() {
        let (socket_path, listener) = SocketPath::bind();
        let (worker, effects) = real_host_worker();
        let gate = HostStartDispatchGate::new();
        let hardware = HardwareManager::with_test_host_control(worker, Arc::clone(&gate));
        let cancellation = hardware.cancellation_handle();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut old_client = connect_ready(socket_path.as_path()).await;

        old_client
            .send(Request::StartHostControl {
                complete_policy: host_policy(),
            })
            .await
            .unwrap();
        let gate_wait = {
            let gate = Arc::clone(&gate);
            tokio::task::spawn_blocking(move || gate.wait_until_entered())
        };
        time::timeout(Duration::from_secs(1), gate_wait)
            .await
            .expect("manager Start did not reach the post-cancellation gate")
            .unwrap();

        // Tear down this client while its accepted Start waits at dispatch.
        // Transport cancellation must not cancel the service-owned command.
        old_client.send(Request::GetSnapshot).await.unwrap();
        time::timeout(Duration::from_secs(1), async {
            while !cancellation.is_cancelled() {
                yield_now().await;
            }
        })
        .await
        .expect("connection teardown did not cancel the blocked manager Start");
        {
            let effects = effects.lock().unwrap();
            assert!(effects.sysfs_operations.is_empty());
            assert_eq!(effects.recovery_persists, 0);
            assert_eq!(effects.recovery_removes, 0);
        }

        gate.release();
        assert!(
            time::timeout(Duration::from_secs(1), old_client.next())
                .await
                .expect("stale connection did not close")
                .is_none()
        );
        {
            let effects = effects.lock().unwrap();
            assert_eq!(effects.recovery_persists, 1);
            assert_eq!(effects.recovery_removes, 0);
            assert_eq!(effects.enable[&3], 1);
            assert_eq!(effects.pwm[&3], 128);
        }

        // Slot release follows the completed Start, so a replacement Stop
        // cannot be overtaken by a late command from the disconnected client.
        let mut new_client = reconnect_ready(socket_path.as_path()).await;

        new_client.send(Request::StopHostControl).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut new_client).await,
            Response::HostControlStopped
        );
        {
            let effects = effects.lock().unwrap();
            assert_eq!(effects.recovery_persists, 1);
            assert_eq!(effects.recovery_removes, 1);
            assert!(
                effects
                    .sysfs_operations
                    .iter()
                    .any(|operation| operation == "write_enable:3:1")
            );
        }

        drop(new_client);
        stop_service(shutdown, service).await;
    }

    fn two_fan_policy() -> HostControlPolicy {
        let mut policy = host_policy();
        let mut fan4 = policy.channels[0].clone();
        fan4.channel_id = ChannelId::new("case-fan4");
        policy.channels.push(fan4);
        policy
    }

    fn assert_service_kept_control(effects: &RealWorkerEffects) {
        assert_eq!(effects.recovery_removes, 0);
        assert!(effects.recovery_record.is_some());
        for channel in [3, 4] {
            assert_eq!(effects.enable[&channel], 1);
            assert_eq!(effects.pwm[&channel], 128);
        }
    }

    fn assert_gentle_restoration(effects: &RealWorkerEffects) {
        assert_eq!(effects.recovery_persists, 1);
        assert_eq!(effects.recovery_removes, 1);
        assert!(effects.recovery_record.is_none());
        for channel in [3, 4] {
            assert_eq!(effects.pwm[&channel], 63);
            assert_eq!(effects.enable[&channel], 2);
            assert!(
                effects
                    .sysfs_operations
                    .contains(&format!("write_pwm:{channel}:128"))
            );
        }
        assert!(
            !effects.sysfs_operations.iter().any(|operation| {
                operation.starts_with("write_enable:") && operation.ends_with(":0")
                    || operation.starts_with("write_pwm:") && operation.ends_with(":255")
            }),
            "unexpected full-speed transition: {:?}",
            effects.sysfs_operations
        );
    }

    #[tokio::test]
    async fn real_worker_client_exits_keep_control_and_service_exit_restores_both_fans() {
        // These are normal service connections using the real engine/worker,
        // not successful acknowledgements from FakeHardware. Only its sysfs,
        // sensors, clock and recovery storage are simulated. The test manager
        // also substitutes failed liquidctl and empty temporary telemetry roots.
        for exit in ["stop", "eof", "shutdown", "malformed-frame"] {
            let (socket_path, listener) = SocketPath::bind();
            let (worker, effects) = real_host_worker_channels(&[3, 4]);
            let gate = HostStartDispatchGate::new();
            gate.release();
            let hardware = HardwareManager::with_test_host_control(worker, gate);
            let (shutdown, service) = start_service(listener, hardware).await;
            let mut client = connect_ready(socket_path.as_path()).await;
            client
                .send(Request::StartHostControl {
                    complete_policy: two_fan_policy(),
                })
                .await
                .unwrap();
            assert_eq!(
                receive::<Response>(&mut client).await,
                Response::HostControlStarted
            );

            match exit {
                "stop" => {
                    client.send(Request::StopHostControl).await.unwrap();
                    assert_eq!(
                        receive::<Response>(&mut client).await,
                        Response::HostControlStopped
                    );
                    assert_gentle_restoration(&effects.lock().unwrap());
                    drop(client);
                    stop_service(shutdown, service).await;
                }
                "shutdown" => {
                    stop_service(shutdown, service).await;
                    assert!(
                        time::timeout(Duration::from_secs(1), client.next())
                            .await
                            .unwrap()
                            .is_none()
                    );
                }
                "malformed-frame" => {
                    let bytes = [0, 0, 0, 1, b'{'];
                    let mut sent = 0;
                    while sent < bytes.len() {
                        client.get_ref().writable().await.unwrap();
                        match client.get_ref().try_write(&bytes[sent..]) {
                            Ok(0) => panic!("socket closed during malformed-frame test"),
                            Ok(count) => sent += count,
                            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
                            Err(error) => panic!("malformed-frame write failed: {error}"),
                        }
                    }
                    assert!(matches!(
                        receive::<Response>(&mut client).await,
                        Response::Error {
                            code: ErrorCode::InvalidData,
                            ..
                        }
                    ));
                    assert!(
                        time::timeout(Duration::from_secs(1), client.next())
                            .await
                            .unwrap()
                            .is_none()
                    );
                    assert_service_kept_control(&effects.lock().unwrap());
                    stop_service(shutdown, service).await;
                }
                "eof" => {
                    drop(client);
                    let replacement = reconnect_ready(socket_path.as_path()).await;
                    assert_service_kept_control(&effects.lock().unwrap());
                    drop(replacement);
                    // Service exit, not the detached client's EOF, restores.
                    stop_service(shutdown, service).await;
                }
                _ => unreachable!(),
            }
            assert_gentle_restoration(&effects.lock().unwrap());
        }
    }

    #[tokio::test]
    async fn failed_liquidctl_cannot_hide_running_policy_or_block_a_reconnecting_stop() {
        let (socket_path, listener) = SocketPath::bind();
        let (worker, effects) = real_host_worker_channels(&[3, 4]);
        let gate = HostStartDispatchGate::new();
        gate.release();
        // This constructor uses a permanently failing fake liquidctl runner,
        // empty temporary telemetry roots, and the real host worker/manager.
        let hardware = HardwareManager::with_test_host_control(worker, gate);
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        let policy = two_fan_policy();
        client
            .send(Request::StartHostControl {
                complete_policy: policy.clone(),
            })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlStarted
        );
        for _ in 0..3 {
            client.send(Request::GetSnapshot).await.unwrap();
            let Response::Snapshot { snapshot } = receive::<Response>(&mut client).await else {
                panic!("liquidctl failure suppressed host-control status");
            };
            assert_eq!(
                snapshot.host_control.state,
                nzxt_cam_core::HostControlState::Running
            );
            assert_eq!(snapshot.host_control.active_policy, Some(policy.clone()));
            assert!(snapshot.devices.is_empty());
        }
        drop(client);
        let mut replacement = reconnect_ready(socket_path.as_path()).await;
        replacement.send(Request::GetSnapshot).await.unwrap();
        let Response::Snapshot { snapshot } = receive::<Response>(&mut replacement).await else {
            panic!("reconnected controller could not inspect running policy");
        };
        assert_eq!(snapshot.host_control.active_policy, Some(policy));
        replacement.send(Request::StopHostControl).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut replacement).await,
            Response::HostControlStopped
        );
        assert_gentle_restoration(&effects.lock().unwrap());
        drop(replacement);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn real_worker_live_update_survives_lost_reply_without_bios_or_pwm_reset() {
        let (socket_path, listener) = SocketPath::bind();
        let (worker, effects) = real_host_worker_channels(&[3, 4]);
        let gate = HostStartDispatchGate::new();
        gate.release();
        let hardware = HardwareManager::with_test_host_control(worker, gate);
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        let original = two_fan_policy();
        client
            .send(Request::StartHostControl {
                complete_policy: original.clone(),
            })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlStarted
        );
        let (record, writes) = {
            let effects = effects.lock().unwrap();
            (
                effects.recovery_record.clone(),
                effects
                    .sysfs_operations
                    .iter()
                    .filter(|operation| operation.starts_with("write_"))
                    .cloned()
                    .collect::<Vec<_>>(),
            )
        };
        let mut changed = original.channels.clone();
        for channel in &mut changed {
            for point in &mut channel.curve.points {
                point.duty_percent = 100;
            }
        }
        client
            .send(Request::UpdateHostControl {
                channel_policies: changed.clone(),
            })
            .await
            .unwrap();
        drop(client); // The accepted update must finish even without a response reader.
        let mut replacement = reconnect_ready(socket_path.as_path()).await;
        replacement.send(Request::GetSnapshot).await.unwrap();
        let Response::Snapshot { snapshot } = receive::<Response>(&mut replacement).await else {
            panic!("missing live-update snapshot");
        };
        assert_eq!(
            snapshot.host_control.state,
            nzxt_cam_core::HostControlState::Running
        );
        let active = snapshot.host_control.active_policy.unwrap();
        assert_eq!(active.channels, changed);
        {
            let effects = effects.lock().unwrap();
            assert_service_kept_control(&effects);
            assert_eq!(effects.recovery_record, record);
            assert_eq!(effects.recovery_persists, 1);
            assert_eq!(
                effects
                    .sysfs_operations
                    .iter()
                    .filter(|operation| operation.starts_with("write_"))
                    .cloned()
                    .collect::<Vec<_>>(),
                writes
            );
        }
        replacement.send(Request::StopHostControl).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut replacement).await,
            Response::HostControlStopped
        );
        assert_gentle_restoration(&effects.lock().unwrap());
        drop(replacement);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn guard_stop_reason_is_visible_and_not_lost_to_a_rejected_update() {
        let (socket_path, listener) = SocketPath::bind();
        let (worker, effects) = real_host_worker_channels(&[3, 4]);
        let gate = HostStartDispatchGate::new();
        gate.release();
        let hardware = HardwareManager::with_test_host_control(worker, gate);
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        let policy = two_fan_policy();
        client
            .send(Request::StartHostControl {
                complete_policy: policy.clone(),
            })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlStarted
        );
        effects.lock().unwrap().cpu_millidegrees = 151_000;
        client
            .send(Request::UpdateHostControl {
                channel_policies: vec![policy.channels[0].clone()],
            })
            .await
            .unwrap();
        assert!(matches!(
            receive::<Response>(&mut client).await,
            Response::Error { .. }
        ));
        for _ in 0..2 {
            client.send(Request::GetSnapshot).await.unwrap();
            let Response::Snapshot { snapshot } = receive::<Response>(&mut client).await else {
                panic!("missing fault snapshot");
            };
            assert_eq!(
                snapshot.host_control.state,
                nzxt_cam_core::HostControlState::Available
            );
            assert!(snapshot.host_control.active_policy.is_none());
            assert!(
                snapshot
                    .host_control
                    .last_error
                    .unwrap()
                    .contains("CPU sensor value is out of range")
            );
            client
                .send(Request::UpdateHostControl {
                    channel_policies: vec![policy.channels[0].clone()],
                })
                .await
                .unwrap();
            assert!(matches!(
                receive::<Response>(&mut client).await,
                Response::Error { .. }
            ));
        }
        assert_gentle_restoration(&effects.lock().unwrap());
        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn real_worker_stop_failure_keeps_record_and_restores_the_other_fan() {
        let (socket_path, listener) = SocketPath::bind();
        let (worker, effects) = real_host_worker_channels(&[3, 4]);
        let gate = HostStartDispatchGate::new();
        gate.release();
        let hardware = HardwareManager::with_test_host_control(worker, gate);
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        client
            .send(Request::StartHostControl {
                complete_policy: two_fan_policy(),
            })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlStarted
        );
        effects.lock().unwrap().fail_bios_channel = Some(3);
        client.send(Request::StopHostControl).await.unwrap();
        assert!(matches!(
            receive::<Response>(&mut client).await,
            Response::Error {
                code: ErrorCode::RestoreRequired,
                ..
            }
        ));
        {
            let mut effects = effects.lock().unwrap();
            assert!(effects.recovery_record.is_some());
            assert_eq!(effects.recovery_removes, 0);
            assert_eq!(effects.enable[&3], 1);
            assert_eq!(effects.pwm[&3], 128);
            assert_eq!(effects.enable[&4], 2);
            assert_eq!(effects.pwm[&4], 63);
            effects.fail_bios_channel = None;
        }
        // A later explicit retry may succeed, but the failed Stop must never
        // have acknowledged restoration or cleared the evidence prematurely.
        client.send(Request::StopHostControl).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::HostControlStopped
        );
        assert_gentle_restoration(&effects.lock().unwrap());
        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn panicking_hardware_operation_invalidates_state_and_closes_connection() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.panic_after_apply_mutation();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        client.send(apply_request()).await.unwrap();
        assert!(
            time::timeout(Duration::from_secs(1), client.next())
                .await
                .expect("panicking operation did not close the connection")
                .is_none()
        );
        let observed = wait_for_observed(&control, |state| state.disconnects == 1).await;
        assert_eq!(observed.finished_operations, 1);
        assert!(!observed.applied_state_valid);
        assert!(!observed.shutdown_seen_at_disconnect);

        let next = reconnect_ready(socket_path.as_path()).await;
        drop(next);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn display_selection_is_service_owned_and_survives_client_disconnect() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        let id = DeviceId::new("kraken-standard");
        client
            .send(Request::SetKrakenDisplay {
                device_id: id.clone(),
                mode: nzxt_cam_core::KrakenDisplayMode::CpuGpu,
            })
            .await
            .unwrap();
        assert_eq!(
            receive::<Response>(&mut client).await,
            Response::KrakenDisplaySet
        );
        drop(client);
        let mut client = reconnect_ready(socket_path.as_path()).await;
        client.send(Request::GetSnapshot).await.unwrap();
        let Response::Snapshot { snapshot } = receive::<Response>(&mut client).await else {
            panic!("expected snapshot")
        };
        assert_eq!(snapshot.kraken_display.device_id, Some(id.clone()));
        assert_eq!(
            snapshot.kraken_display.mode,
            nzxt_cam_core::KrakenDisplayMode::CpuGpu
        );
        assert_eq!(
            control.snapshot().display_sets,
            vec![(id, nzxt_cam_core::KrakenDisplayMode::CpuGpu)]
        );
        drop(client);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn fixed_v6_ready_handshake_keeps_connection_open() {
        let (client, service) = UnixStream::pair().unwrap();
        let (hardware, control) = FakeHardware::new();
        let cancellation = hardware.cancellation_handle();
        let hardware = Arc::new(Mutex::new(hardware));
        let (_initialized_tx, initialized_rx) = watch::channel(true);
        let (_shutdown_tx, shutdown_rx) = watch::channel(false);
        let task = tokio::spawn(handle_active_connection(
            service,
            hardware,
            cancellation,
            initialized_rx,
            shutdown_rx,
        ));
        let mut client = Framed::new(client, JsonFrameCodec::new());

        client.send(ClientHello::v6()).await.unwrap();
        assert_eq!(
            receive::<ServerResponse>(&mut client).await,
            ServerResponse::ready_v6()
        );
        assert!(
            time::timeout(Duration::from_millis(25), client.next())
                .await
                .is_err()
        );

        drop(client);
        assert!(task.await.unwrap().is_some());
        assert_eq!(control.snapshot().disconnects, 1);
    }

    #[tokio::test]
    async fn unsupported_version_is_rejected_without_invalidating_hardware() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        for protocol_version in [1, 2, 3, 4] {
            let mut unsupported = connect(socket_path.as_path()).await;
            unsupported
                .send(ClientHello { protocol_version })
                .await
                .unwrap();
            assert_eq!(
                receive::<ServerResponse>(&mut unsupported).await,
                ServerResponse::rejected(RejectionCode::UnsupportedVersion)
            );
            assert!(unsupported.next().await.is_none());
            assert_eq!(control.snapshot().disconnects, 0);
        }

        let next = connect_ready(socket_path.as_path()).await;
        drop(next);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn invalid_first_message_is_rejected_and_releases_slot() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut invalid = connect(socket_path.as_path()).await;

        invalid.send("not a client hello").await.unwrap();
        assert_eq!(
            receive::<ServerResponse>(&mut invalid).await,
            ServerResponse::rejected(RejectionCode::InvalidMessage)
        );
        assert!(invalid.next().await.is_none());
        assert_eq!(control.snapshot().disconnects, 0);

        let next = connect_ready(socket_path.as_path()).await;
        drop(next);
        stop_service(shutdown, service).await;
    }

    #[tokio::test(start_paused = true)]
    async fn initial_hello_timeout_releases_slot_without_invalidating_hardware() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut timed_out = connect(socket_path.as_path()).await;

        let mut busy = connect(socket_path.as_path()).await;
        assert_eq!(
            receive::<ServerResponse>(&mut busy).await,
            ServerResponse::rejected(RejectionCode::ServiceBusy)
        );
        time::advance(INITIAL_HELLO_TIMEOUT).await;
        yield_now().await;
        assert!(timed_out.next().await.is_none());
        assert_eq!(control.snapshot().disconnects, 0);

        let next = connect_ready(socket_path.as_path()).await;
        drop(next);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn second_connection_is_busy_while_first_is_ready_and_idle() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, _control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut first = connect_ready(socket_path.as_path()).await;
        let mut second = connect(socket_path.as_path()).await;

        assert_eq!(
            receive::<ServerResponse>(&mut second).await,
            ServerResponse::rejected(RejectionCode::ServiceBusy)
        );
        assert!(second.next().await.is_none());
        assert!(
            time::timeout(Duration::from_millis(25), first.next())
                .await
                .is_err()
        );

        drop(first);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn client_becomes_ready_after_active_client_disconnects() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, _control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let first = connect_ready(socket_path.as_path()).await;
        drop(first);

        // Closing the stream wakes the active connection task, but a new
        // accept can briefly race its final cleanup. Busy remains correct until
        // that task has fully released the single-client slot.
        let second = reconnect_ready(socket_path.as_path()).await;
        drop(second);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn actual_peer_credentials_are_captured() {
        let (client, service) = UnixStream::pair().unwrap();
        let expected = service.peer_cred().unwrap();
        let (hardware, _control) = FakeHardware::new();
        let cancellation = hardware.cancellation_handle();
        let hardware = Arc::new(Mutex::new(hardware));
        let (_initialized_tx, initialized_rx) = watch::channel(true);
        let (_shutdown_tx, shutdown_rx) = watch::channel(false);
        let task = tokio::spawn(handle_active_connection(
            service,
            hardware,
            cancellation,
            initialized_rx,
            shutdown_rx,
        ));
        let mut client = Framed::new(client, JsonFrameCodec::new());

        client.send(ClientHello::v6()).await.unwrap();
        assert_eq!(
            receive::<ServerResponse>(&mut client).await,
            ServerResponse::ready_v6()
        );
        drop(client);

        let credentials = task.await.unwrap().unwrap();
        assert_eq!(credentials.pid, std::process::id());
        assert_eq!(
            credentials.pid,
            u32::try_from(expected.pid().unwrap()).unwrap()
        );
        assert_eq!(credentials.uid, expected.uid());
        assert_eq!(credentials.gid, expected.gid());
    }

    #[tokio::test]
    async fn blocked_initialization_allows_prompt_handshake_but_no_hardware_calls_or_resets() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let cancellation = hardware.cancellation_handle();
        control.block_initialization.store(true, Ordering::Release);
        let (shutdown, service, mut initialized) = start_initializing_service(listener, hardware);
        wait_for_initialization_started(&control).await;
        let mut client =
            time::timeout(Duration::from_secs(1), connect_ready(socket_path.as_path()))
                .await
                .expect("handshake waited for the blocked initializer");
        let mut busy = connect(socket_path.as_path()).await;
        assert_eq!(
            receive::<ServerResponse>(&mut busy).await,
            ServerResponse::rejected(RejectionCode::ServiceBusy)
        );

        // This probe must survive every early request: none may reset USB
        // cancellation or queue work behind guarded startup.
        cancellation.cancel();
        for request in [
            Request::GetSnapshot,
            apply_request(),
            Request::SetKrakenDisplay {
                device_id: DeviceId::new("lcd"),
                mode: nzxt_cam_core::KrakenDisplayMode::Cpu,
            },
            Request::ActivateMonitoring {
                firmware_curves: Vec::new(),
                host_policy: None,
                display: None,
            },
            Request::SetMonitoringAutoResume { enabled: false },
            Request::StartHostControl {
                complete_policy: host_policy(),
            },
            Request::UpdateHostControl {
                channel_policies: host_policy().channels,
            },
            Request::StopHostControl,
        ] {
            client.send(request).await.unwrap();
            assert_eq!(
                time::timeout(Duration::from_secs(1), receive::<Response>(&mut client))
                    .await
                    .unwrap(),
                Response::Error {
                    code: ErrorCode::Unavailable,
                    message: ErrorMessage::new(INITIALIZING_MESSAGE)
                }
            );
            assert!(cancellation.is_cancelled());
        }
        let observed = control.snapshot();
        assert_eq!(observed.snapshot_calls, 0);
        assert!(observed.apply_calls.is_empty());
        assert!(observed.display_sets.is_empty());
        assert!(observed.activation_calls.is_empty());
        assert!(observed.auto_resume_changes.is_empty());
        assert!(observed.host_starts.is_empty());
        assert!(observed.host_updates.is_empty());
        assert_eq!(observed.host_stops, 0);
        assert_eq!(observed.disconnects, 0);
        assert!(!*initialized.borrow());

        control.block_initialization.store(false, Ordering::Release);
        initialized.wait_for(|ready| *ready).await.unwrap();
        client.send(Request::GetSnapshot).await.unwrap();
        assert!(matches!(
            receive::<Response>(&mut client).await,
            Response::Snapshot { .. }
        ));
        assert!(!cancellation.is_cancelled());
        assert_eq!(control.snapshot().snapshot_calls, 1);
        drop(client);
        let mut next = reconnect_ready(socket_path.as_path()).await;
        next.send(apply_request()).await.unwrap();
        assert_eq!(
            receive::<Response>(&mut next).await,
            Response::FirmwareCurveApplied
        );
        assert_eq!(control.initialization_calls.load(Ordering::Acquire), 1);
        drop(next);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn initialization_failure_remains_queryable_without_retry() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block_initialization.store(true, Ordering::Release);
        control.fail_initialization.store(true, Ordering::Release);
        let (shutdown, service, mut initialized) = start_initializing_service(listener, hardware);
        let mut client = connect_ready(socket_path.as_path()).await;
        control.block_initialization.store(false, Ordering::Release);
        initialized.wait_for(|ready| *ready).await.unwrap();
        client.send(Request::GetSnapshot).await.unwrap();
        let Response::Snapshot { snapshot } = receive::<Response>(&mut client).await else {
            panic!("failed bootstrap did not expose status");
        };
        assert_eq!(
            snapshot.host_control.state,
            nzxt_cam_core::HostControlState::RestoreRequired
        );
        drop(client);
        let next = reconnect_ready(socket_path.as_path()).await;
        assert_eq!(control.initialization_calls.load(Ordering::Acquire), 1);
        drop(next);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn initialization_panic_is_visible_and_closes_early_client_without_mutex_cleanup() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let cancellation = hardware.cancellation_handle();
        control.block_initialization.store(true, Ordering::Release);
        control.panic_initialization.store(true, Ordering::Release);
        let (_shutdown, service, initialized) = start_initializing_service(listener, hardware);
        wait_for_initialization_started(&control).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        control.block_initialization.store(false, Ordering::Release);
        assert_eq!(
            time::timeout(Duration::from_secs(1), service)
                .await
                .unwrap()
                .unwrap(),
            Err(ServiceError::InitializationPanicked)
        );
        assert!(client.next().await.is_none());
        assert!(!*initialized.borrow());
        assert!(cancellation.is_cancelled());
        assert!(control.host_shutdown_count.load(Ordering::Acquire) > 0);
        assert_eq!(control.initialization_calls.load(Ordering::Acquire), 1);
        assert_eq!(control.snapshot().disconnects, 0);
    }

    #[tokio::test]
    async fn early_disconnect_and_malformed_frames_do_not_cancel_guarded_startup() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let cancellation = hardware.cancellation_handle();
        control.block_initialization.store(true, Ordering::Release);
        let (shutdown, service, mut initialized) = start_initializing_service(listener, hardware);
        wait_for_initialization_started(&control).await;
        let mut client = connect_ready(socket_path.as_path()).await;
        client.send(Request::GetSnapshot).await.unwrap();
        assert!(matches!(
            receive::<Response>(&mut client).await,
            Response::Error {
                code: ErrorCode::Unavailable,
                ..
            }
        ));
        drop(client);
        let mut next = reconnect_ready(socket_path.as_path()).await;
        next.send(serde_json::json!({"type": "get_snapshot", "unexpected": true}))
            .await
            .unwrap();
        assert!(matches!(
            receive::<Response>(&mut next).await,
            Response::Error {
                code: ErrorCode::InvalidData,
                ..
            }
        ));
        let bytes = [0, 0, 0, 1, b'{'];
        let mut sent = 0;
        while sent < bytes.len() {
            next.get_ref().writable().await.unwrap();
            match next.get_ref().try_write(&bytes[sent..]) {
                Ok(0) => panic!("socket closed during malformed-frame test"),
                Ok(count) => sent += count,
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
                Err(error) => panic!("malformed-frame write failed: {error}"),
            }
        }
        assert!(matches!(
            receive::<Response>(&mut next).await,
            Response::Error {
                code: ErrorCode::InvalidData,
                ..
            }
        ));
        assert!(
            time::timeout(Duration::from_secs(1), next.next())
                .await
                .unwrap()
                .is_none()
        );
        let mut replacement = reconnect_ready(socket_path.as_path()).await;
        assert!(!cancellation.is_cancelled());
        assert_eq!(control.host_shutdown_count.load(Ordering::Acquire), 0);
        assert_eq!(control.snapshot().disconnects, 0);
        assert!(!*initialized.borrow());
        assert_eq!(control.initialization_calls.load(Ordering::Acquire), 1);
        control.block_initialization.store(false, Ordering::Release);
        initialized.wait_for(|ready| *ready).await.unwrap();
        replacement.send(Request::GetSnapshot).await.unwrap();
        assert!(matches!(
            receive::<Response>(&mut replacement).await,
            Response::Snapshot { .. }
        ));
        drop(replacement);
        stop_service(shutdown, service).await;
    }

    #[tokio::test]
    async fn shutdown_signals_blocked_initialization_without_waiting_for_manager_mutex() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        control.block_initialization.store(true, Ordering::Release);
        let cancellation = hardware.cancellation_handle();
        let (shutdown, service, initialized) = start_initializing_service(listener, hardware);
        let mut queued = connect(socket_path.as_path()).await;
        wait_for_initialization_started(&control).await;
        // Initialization holds the manager mutex until the independent
        // service-shutdown signal reaches it. A lock-first shutdown deadlocks.
        stop_service(shutdown, service).await;
        assert!(control.host_shutdown_count.load(Ordering::Acquire) > 0);
        assert!(cancellation.is_cancelled());
        assert!(!*initialized.borrow());
        assert!(matches!(queued.next().await, None | Some(Err(_))));
        assert_eq!(control.snapshot().snapshot_calls, 0);
    }

    #[tokio::test]
    async fn shutdown_during_first_replay_drains_it_and_prevents_later_targets_and_lcd() {
        let root = tempfile::tempdir().unwrap();
        std::fs::set_permissions(root.path(), std::fs::Permissions::from_mode(0o700)).unwrap();
        let store_path = root.path().join("monitor-intent.json");
        let mut store = FileMonitorIntentStore::at(store_path.clone());
        let curve = |id| MonitorTarget::AioCurve {
            device_id: DeviceId::new(id),
            channel_id: ChannelId::new("pump"),
            points: vec![CurvePoint {
                temperature: 59,
                duty: 100,
            }],
        };
        let intent = SavedMonitorIntent {
            version: 1,
            opted_in: true,
            auto_resume: true,
            // Remembering display first must not dispatch it if a later write
            // is interrupted. The real manager keeps the worker idle in boot.
            targets: [
                MonitorTarget::Display {
                    device_id: DeviceId::new("lcd"),
                    mode: nzxt_cam_core::KrakenDisplayMode::Cpu,
                },
                curve("first"),
                curve("later"),
            ]
            .into_iter()
            .map(|target| SavedTarget {
                target,
                state: WriteState::Ready,
            })
            .collect(),
        };
        store.persist(&intent).unwrap();
        let (socket_path, listener) = SocketPath::bind();
        let (mut hardware, control) = FakeHardware::new();
        hardware.startup_store = Some(store);
        let cancellation = hardware.cancellation_handle();
        control.block();
        let (shutdown, mut service, initialized) = start_initializing_service(listener, hardware);
        let client = connect_ready(socket_path.as_path()).await;
        wait_for_observed(&control, |state| state.active_operations == 1).await;
        shutdown.send(()).unwrap();
        wait_for_observed(&control, |state| state.cancellation_seen).await;
        assert!(cancellation.is_cancelled());
        assert!(control.host_shutdown_count.load(Ordering::Acquire) > 0);
        assert!(
            time::timeout(Duration::from_millis(25), &mut service)
                .await
                .is_err()
        );
        assert!(!*initialized.borrow());
        control.release();
        time::timeout(Duration::from_secs(1), service)
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let observed = control.snapshot();
        assert_eq!(observed.startup_preflights, [DeviceId::new("first")]);
        assert_eq!(observed.apply_calls.len(), 1);
        assert!(observed.display_sets.is_empty());
        assert_eq!(observed.disconnects, 0);
        assert_eq!(control.initialization_calls.load(Ordering::Acquire), 1);
        // This fake returns definite success from the interrupted first write.
        // Persist that evidence, but never start a second write or LCD upload.
        assert_eq!(
            FileMonitorIntentStore::at(store_path).load().unwrap(),
            Some(intent)
        );
        drop(client);
    }

    #[tokio::test]
    async fn already_ready_shutdown_accepts_no_queued_connection() {
        let (socket_path, listener) = SocketPath::bind();
        let mut queued = connect(socket_path.as_path()).await;
        let (hardware, control) = FakeHardware::new();
        let cancellation = hardware.cancellation_handle();

        serve_until(listener, hardware, std::future::ready(()))
            .await
            .unwrap();

        assert!(cancellation.is_cancelled());
        assert!(control.host_shutdown_count.load(Ordering::Acquire) > 0);
        assert_eq!(control.initialization_calls.load(Ordering::Acquire), 0);
        match queued.next().await {
            None | Some(Err(_)) => {}
            Some(Ok(message)) => panic!("queued connection received a response: {message}"),
        }
    }

    #[tokio::test]
    async fn shutdown_closes_ready_connection_and_invalidates_hardware() {
        let (socket_path, listener) = SocketPath::bind();
        let (hardware, control) = FakeHardware::new();
        let (shutdown, service) = start_service(listener, hardware).await;
        let mut client = connect_ready(socket_path.as_path()).await;

        stop_service(shutdown, service).await;
        assert!(client.next().await.is_none());
        assert_eq!(control.snapshot().disconnects, 1);
    }
}
