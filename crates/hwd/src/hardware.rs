use std::{
    error::Error,
    fmt,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use nzxt_cam_protocol::{
    MonitoringActivationOutcome, MonitoringDisplaySelection, MonitoringFirmwareCurve,
};

use nzxt_cam_core::{
    ChannelId, CurvePoint, Device, DeviceId, HardwareSnapshot, HostChannelPolicy,
    HostControlPolicy, HostControlSnapshot, HostControlState, KrakenDisplayMode,
    KrakenDisplaySnapshot, MonitoringActualState, MonitoringError, MonitoringSnapshot,
    MonitoringTargetIntent, MonitoringTargetSnapshot,
};

use crate::{
    config::{ConfigError, HardwareConfig},
    host_control::{HostControlEngine, HostControlShutdownHandle, HostControlWorker},
    kraken_display::KrakenDisplayWorker,
    liquidctl::LiquidctlHardware,
    monitor_intent::{FileMonitorIntentStore, MonitorTarget, WriteState},
    monitor_resume,
    monitor_service::{self, MonitoringIo},
    ownership::HostControlLock,
    telemetry::HostTelemetry,
};

const MAX_ERROR_MESSAGE_BYTES: usize = 512;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HardwareErrorKind {
    Unavailable,
    PermissionDenied,
    Unsupported,
    InvalidData,
    Timeout,
    UnknownOutcome,
    RestoreRequired,
    Internal,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct HardwareError {
    kind: HardwareErrorKind,
    message: String,
    service_shutdown: bool,
}

impl HardwareError {
    pub fn new(message: impl Into<String>) -> Self {
        Self::with_kind(HardwareErrorKind::Internal, message)
    }

    pub fn with_kind(kind: HardwareErrorKind, message: impl Into<String>) -> Self {
        let mut message = message.into();
        if message.len() > MAX_ERROR_MESSAGE_BYTES {
            let mut boundary = MAX_ERROR_MESSAGE_BYTES;
            while !message.is_char_boundary(boundary) {
                boundary -= 1;
            }
            message.truncate(boundary);
        }
        Self {
            kind,
            message,
            service_shutdown: false,
        }
    }

    pub(crate) fn service_shutdown() -> Self {
        Self {
            kind: HardwareErrorKind::Unavailable,
            message: "host-control service shutdown".into(),
            service_shutdown: true,
        }
    }

    pub(crate) const fn is_service_shutdown(&self) -> bool {
        self.service_shutdown
    }

    pub const fn kind(&self) -> HardwareErrorKind {
        self.kind
    }

    pub fn message(&self) -> &str {
        &self.message
    }

    #[cfg(test)]
    pub const fn write_outcome_unknown(&self) -> bool {
        matches!(
            self.kind,
            HardwareErrorKind::Timeout | HardwareErrorKind::UnknownOutcome
        )
    }
}

impl fmt::Display for HardwareError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl Error for HardwareError {}

#[derive(Debug)]
pub enum HardwareManagerError {
    Config(ConfigError),
    Ownership(HardwareError),
}

impl fmt::Display for HardwareManagerError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Config(error) => write!(formatter, "invalid hardware config: {error}"),
            Self::Ownership(error) => {
                write!(formatter, "host-control ownership unavailable: {error}")
            }
        }
    }
}

impl Error for HardwareManagerError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Config(error) => Some(error),
            Self::Ownership(error) => Some(error),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct HardwareCancellation {
    cancelled: Arc<AtomicBool>,
}

impl HardwareCancellation {
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
    }

    pub fn reset(&self) {
        self.cancelled.store(false, Ordering::Release);
    }

    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::Acquire)
    }
}

pub trait HardwareOperations: Send + 'static {
    /// Complete service-owned recovery/resume before dispatching hardware commands.
    /// The control-plane handshake may finish while this is still running.
    fn initialize_host_control(&mut self) -> Result<(), HardwareError> {
        Ok(())
    }

    fn snapshot(&mut self) -> Result<HardwareSnapshot, HardwareError>;

    fn activate_monitoring(
        &mut self,
        _curves: &[MonitoringFirmwareCurve],
        _host_policy: Option<&HostControlPolicy>,
        _display: Option<&MonitoringDisplaySelection>,
    ) -> Result<Vec<MonitoringActivationOutcome>, HardwareError> {
        Err(HardwareError::with_kind(
            HardwareErrorKind::Unsupported,
            "monitoring activation unsupported",
        ))
    }

    fn set_monitoring_auto_resume(&mut self, _enabled: bool) -> Result<(), HardwareError> {
        Err(HardwareError::with_kind(
            HardwareErrorKind::Unsupported,
            "monitoring auto-resume unsupported",
        ))
    }

    fn set_kraken_display(
        &mut self,
        _device_id: &DeviceId,
        _mode: KrakenDisplayMode,
    ) -> Result<(), HardwareError> {
        Err(HardwareError::with_kind(
            HardwareErrorKind::Unsupported,
            "Kraken display is unsupported",
        ))
    }

    fn apply_firmware_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError>;

    fn start_host_control(&mut self, policy: &HostControlPolicy) -> Result<(), HardwareError>;

    fn update_host_control(
        &mut self,
        channel_policies: &[HostChannelPolicy],
    ) -> Result<(), HardwareError>;

    fn stop_host_control(&mut self) -> Result<(), HardwareError>;

    fn host_control_shutdown_handle(&self) -> HostControlShutdownHandle;

    fn cancellation_handle(&self) -> HardwareCancellation;

    fn client_disconnected(&mut self);
}

pub struct HardwareManager {
    hardware: LiquidctlHardware,
    telemetry: HostTelemetry,
    host_control: HostControlWorker,
    kraken_display: KrakenDisplayWorker,
    monitor_store: FileMonitorIntentStore,
    monitor_actual: Vec<(
        MonitorTarget,
        MonitoringActualState,
        Option<MonitoringError>,
    )>,
    #[cfg(test)]
    host_start_dispatch_gate: Option<Arc<HostStartDispatchGate>>,
    #[cfg(test)]
    _test_telemetry_root: Option<tempfile::TempDir>,
    // Declared last so it is dropped only after the worker and all other
    // manager fields have completed their own drops.
    _host_control_lock: Option<HostControlLock>,
}

#[cfg(test)]
pub(crate) struct HostStartDispatchGate {
    state: std::sync::Mutex<HostStartDispatchGateState>,
    changed: std::sync::Condvar,
}

#[cfg(test)]
#[derive(Default)]
struct HostStartDispatchGateState {
    entered: bool,
    released: bool,
}

#[cfg(test)]
impl HostStartDispatchGate {
    pub(crate) fn new() -> Arc<Self> {
        Arc::new(Self {
            state: std::sync::Mutex::new(HostStartDispatchGateState::default()),
            changed: std::sync::Condvar::new(),
        })
    }

    fn block_once(&self) {
        let mut state = self.state.lock().unwrap();
        if state.released {
            return;
        }
        state.entered = true;
        self.changed.notify_all();
        while !state.released {
            state = self.changed.wait(state).unwrap();
        }
    }

    pub(crate) fn wait_until_entered(&self) {
        let mut state = self.state.lock().unwrap();
        while !state.entered {
            state = self.changed.wait(state).unwrap();
        }
    }

    pub(crate) fn release(&self) {
        let mut state = self.state.lock().unwrap();
        state.released = true;
        self.changed.notify_all();
    }
}

impl HardwareManager {
    /// Acquires exclusive process ownership, then reads and validates the
    /// root-owned service configuration. Construction does not query
    /// liquidctl, sysfs, DMI, NVML, or any physical device.
    pub fn new() -> Result<Self, HardwareManagerError> {
        // This must precede construction of the production engine because its
        // constructor inspects recovery state.
        let host_control_lock =
            HostControlLock::acquire_service().map_err(HardwareManagerError::Ownership)?;
        let config = HardwareConfig::load_default().map_err(HardwareManagerError::Config)?;
        Ok(Self::from_config(config, Some(host_control_lock)))
    }

    fn from_config(config: HardwareConfig, host_control_lock: Option<HostControlLock>) -> Self {
        let io_lock = Arc::new(std::sync::Mutex::new(()));
        Self {
            hardware: LiquidctlHardware::with_io_lock(io_lock.clone()),
            telemetry: HostTelemetry::new(config.clone()),
            host_control: HostControlWorker::new(HostControlEngine::new(config.clone())),
            kraken_display: KrakenDisplayWorker::new(config, io_lock),
            monitor_store: FileMonitorIntentStore::new(),
            monitor_actual: Vec::new(),
            #[cfg(test)]
            host_start_dispatch_gate: None,
            #[cfg(test)]
            _test_telemetry_root: None,
            _host_control_lock: host_control_lock,
        }
    }

    #[cfg(test)]
    pub(crate) fn with_test_host_control(
        host_control: HostControlWorker,
        host_start_dispatch_gate: Arc<HostStartDispatchGate>,
    ) -> Self {
        let root = tempfile::TempDir::new().unwrap();
        Self {
            hardware: LiquidctlHardware::unavailable_for_test(),
            telemetry: HostTelemetry::at_empty_test_root(root.path()),
            host_control,
            kraken_display: KrakenDisplayWorker::with_sources(
                LiquidctlHardware::unavailable_for_test(),
                HostTelemetry::at_empty_test_root(root.path()),
            ),
            monitor_store: FileMonitorIntentStore::at(root.path().join("monitor-intent.json")),
            monitor_actual: Vec::new(),
            host_start_dispatch_gate: Some(host_start_dispatch_gate),
            _test_telemetry_root: Some(root),
            _host_control_lock: None,
        }
    }
    fn monitoring_snapshot(
        &self,
        snapshot: &HardwareSnapshot,
    ) -> Result<MonitoringSnapshot, HardwareError> {
        let Some(intent) = self.monitor_store.load()? else {
            return Ok(MonitoringSnapshot::default());
        };
        let targets = intent
            .targets
            .into_iter()
            .map(|saved| {
                let current = self
                    .monitor_actual
                    .iter()
                    .find(|(target, _, _)| target == &saved.target);
                let (mut actual_state, mut last_error) = current.map_or(
                    (MonitoringActualState::Pending, None),
                    |(_, state, error)| (*state, error.clone()),
                );
                if saved.state == WriteState::InFlightUnknown {
                    actual_state = MonitoringActualState::ReviewRequired;
                    if last_error.is_none() {
                        last_error = Some(MonitoringError::new("write outcome requires review"));
                    }
                } else if let MonitorTarget::Display { device_id, mode } = &saved.target {
                    if self.kraken_display.applied_selection(device_id, *mode) {
                        actual_state = MonitoringActualState::Applied;
                        last_error = None;
                    } else if snapshot.kraken_display.device_id.as_ref() != Some(device_id) {
                        actual_state = MonitoringActualState::Unavailable;
                    } else {
                        let worker = self.kraken_display.snapshot();
                        if worker.device_id.as_ref() == Some(device_id)
                            && worker.mode == *mode
                            && let Some(error) = worker.last_error
                        {
                            actual_state = MonitoringActualState::Unavailable;
                            last_error = Some(MonitoringError::new(error));
                        }
                    }
                } else if let MonitorTarget::AioCurve {
                    device_id,
                    channel_id,
                    ..
                } = &saved.target
                    && !snapshot.devices.iter().any(|device| {
                        &device.id == device_id
                            && device.online
                            && device
                                .cooling_channels
                                .iter()
                                .any(|channel| &channel.id == channel_id)
                    })
                {
                    actual_state = MonitoringActualState::Unavailable;
                }
                MonitoringTargetSnapshot {
                    intent: match saved.target {
                        MonitorTarget::AioCurve {
                            device_id,
                            channel_id,
                            points,
                        } => MonitoringTargetIntent::AioCurve {
                            device_id,
                            channel_id,
                            points,
                        },
                        MonitorTarget::Display { device_id, mode } => {
                            MonitoringTargetIntent::Display { device_id, mode }
                        }
                    },
                    actual_state,
                    last_error,
                }
            })
            .collect();
        Ok(MonitoringSnapshot {
            opted_in: intent.opted_in,
            auto_resume: intent.auto_resume,
            targets,
        })
    }
}

fn restore_startup_display(
    display: &KrakenDisplayWorker,
    selection: Option<(DeviceId, KrakenDisplayMode)>,
    cancellation: &HardwareCancellation,
) -> Result<(), HardwareError> {
    // Boot can finish just as shutdown arrives. The last check belongs at the
    // worker handoff, not only inside the replay loop.
    if cancellation.is_cancelled() {
        return Err(HardwareError::service_shutdown());
    }
    if let Some((id, mode)) = selection {
        display.select_restored(id, mode);
    }
    Ok(())
}

/// Advertising an eligible cooler is not selecting it. Once a selection is
/// owned by the worker, an offline identity never turns into a replacement.
fn advertised_display(
    mut display: KrakenDisplaySnapshot,
    candidate: Option<DeviceId>,
) -> KrakenDisplaySnapshot {
    if let Some(selected) = &display.device_id {
        display.device_id = candidate.filter(|id| id == selected);
    } else {
        if candidate.is_some() {
            display.last_error = Some("LCD not selected or applied".into());
        }
        display.device_id = candidate;
    }
    display
}

fn invalidate_offline_monitor_claims(
    actual: &mut [(
        MonitorTarget,
        MonitoringActualState,
        Option<MonitoringError>,
    )],
    devices: &[Device],
) {
    for (target, state, error) in actual {
        let MonitorTarget::AioCurve {
            device_id,
            channel_id,
            ..
        } = target
        else {
            continue;
        };
        if *state == MonitoringActualState::Applied
            && !devices.iter().any(|device| {
                device.online
                    && &device.id == device_id
                    && device
                        .cooling_channels
                        .iter()
                        .any(|channel| &channel.id == channel_id)
            })
        {
            *state = MonitoringActualState::Unavailable;
            *error = Some(MonitoringError::new(
                "cooler disconnected; previous curve application is no longer verified",
            ));
        }
    }
}

fn append_service_state(
    snapshot: &mut HardwareSnapshot,
    host: Option<Device>,
    host_control: HostControlSnapshot,
) {
    if let Some(host) = host {
        snapshot.devices.push(host);
    }
    snapshot.host_control = host_control;
}

struct ManagerMonitoringIo<'a> {
    hardware: &'a mut LiquidctlHardware,
    host: &'a mut HostControlWorker,
    display: &'a KrakenDisplayWorker,
}

impl MonitoringIo for ManagerMonitoringIo<'_> {
    fn preflight_curve(
        &mut self,
        id: &DeviceId,
        channel: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<bool, HardwareError> {
        self.hardware.preflight_resume_curve(id, channel, points)
    }
    fn write_curve(
        &mut self,
        id: &DeviceId,
        channel: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError> {
        self.hardware.apply_firmware_curve(id, channel, points)
    }
    fn preflight_display(&mut self, id: &DeviceId) -> Result<(), HardwareError> {
        self.hardware.display_serial(id).map(|_| ())
    }
    fn select_display(&mut self, id: DeviceId, mode: KrakenDisplayMode) {
        self.display.select(id, mode);
    }
    fn clear_display(&mut self) {
        self.display.clear_selection();
    }
    fn host_state(&self) -> HostControlState {
        self.host.snapshot().state
    }
    fn start_host(&mut self, policy: &HostControlPolicy) -> Result<(), HardwareError> {
        self.host.start(policy)
    }
}

impl HardwareOperations for HardwareManager {
    fn initialize_host_control(&mut self) -> Result<(), HardwareError> {
        // Load intent before host boot; recovery is unconditional even if the
        // store is corrupt. The LCD worker remains idle until recovery and
        // startup curve replay have finished.
        let cancellation = self.hardware.cancellation_handle();
        let check = || {
            if cancellation.is_cancelled() {
                Err(HardwareError::service_shutdown())
            } else {
                Ok(())
            }
        };
        let selection = monitor_resume::boot(
            &mut self.monitor_store,
            &mut self.hardware,
            |resume| self.host_control.initialize_with_resume(resume),
            || monitor_resume::startup_pause(&cancellation),
            check,
        )?;
        for target in selection.applied_curves {
            self.monitor_actual
                .push((target, MonitoringActualState::Applied, None));
        }
        restore_startup_display(
            &self.kraken_display,
            selection.restored_display,
            &cancellation,
        )
    }

    fn snapshot(&mut self) -> Result<HardwareSnapshot, HardwareError> {
        let mut snapshot = match self.hardware.snapshot() {
            Ok(snapshot) => snapshot,
            Err(error) => {
                // The service owns independent USB and motherboard sources.
                // A USB failure must not hide a running policy or its Stop UI.
                eprintln!("liquidctl snapshot unavailable: {error}");
                self.hardware.offline_snapshot()
            }
        };
        let host = self.telemetry.sample_device();
        append_service_state(&mut snapshot, host, self.host_control.snapshot());
        let display = self.kraken_display.snapshot();
        let candidate = self.hardware.display_candidate(display.device_id.as_ref());
        snapshot.kraken_display = advertised_display(display, candidate);
        invalidate_offline_monitor_claims(&mut self.monitor_actual, &snapshot.devices);
        snapshot.monitoring = match self.monitoring_snapshot(&snapshot) {
            Ok(monitoring) => monitoring,
            Err(error) => {
                // Intent corruption blocks mutations, but cannot conceal Stop.
                eprintln!("monitor intent unavailable: {error}");
                MonitoringSnapshot::default()
            }
        };
        Ok(snapshot)
    }

    fn set_kraken_display(
        &mut self,
        device_id: &DeviceId,
        mode: KrakenDisplayMode,
    ) -> Result<(), HardwareError> {
        self.hardware.display_serial(device_id)?;
        let target = MonitorTarget::Display {
            device_id: device_id.clone(),
            mode,
        };
        monitor_service::update_selected(&mut self.monitor_store, target.clone())?;
        self.monitor_actual
            .retain(|(saved, _, _)| !monitor_service::same_key(saved, &target));
        self.kraken_display.select(device_id.clone(), mode);
        Ok(())
    }

    fn apply_firmware_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError> {
        if !self
            .hardware
            .preflight_resume_curve(device_id, channel_id, points)?
        {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                "cooler offline",
            ));
        }
        let target = MonitorTarget::AioCurve {
            device_id: device_id.clone(),
            channel_id: channel_id.clone(),
            points: points.to_vec(),
        };
        monitor_service::update_selected(&mut self.monitor_store, target.clone())?;
        let result = monitor_service::write_curve(
            &mut self.monitor_store,
            &target,
            |id, channel, points| self.hardware.apply_firmware_curve(id, channel, points),
        );
        monitor_service::record_write(&mut self.monitor_actual, &target, &result);
        result
    }

    fn activate_monitoring(
        &mut self,
        curves: &[MonitoringFirmwareCurve],
        host_policy: Option<&HostControlPolicy>,
        display: Option<&MonitoringDisplaySelection>,
    ) -> Result<Vec<MonitoringActivationOutcome>, HardwareError> {
        let mut io = ManagerMonitoringIo {
            hardware: &mut self.hardware,
            host: &mut self.host_control,
            display: &self.kraken_display,
        };
        monitor_service::activate(
            &mut self.monitor_store,
            &mut io,
            &mut self.monitor_actual,
            curves,
            host_policy,
            display,
        )
    }

    fn set_monitoring_auto_resume(&mut self, enabled: bool) -> Result<(), HardwareError> {
        monitor_service::set_auto_resume(&mut self.monitor_store, enabled)
    }

    fn start_host_control(&mut self, policy: &HostControlPolicy) -> Result<(), HardwareError> {
        // Once accepted by the service, host mutations finish independently
        // of the client's transport. Service shutdown has its own signal.
        #[cfg(test)]
        if let Some(gate) = &self.host_start_dispatch_gate {
            gate.block_once();
        }
        self.host_control.start(policy)
    }

    fn update_host_control(
        &mut self,
        channel_policies: &[HostChannelPolicy],
    ) -> Result<(), HardwareError> {
        self.host_control.update(channel_policies)
    }

    fn stop_host_control(&mut self) -> Result<(), HardwareError> {
        self.host_control.stop()
    }

    fn host_control_shutdown_handle(&self) -> HostControlShutdownHandle {
        self.host_control.shutdown_handle()
    }

    fn cancellation_handle(&self) -> HardwareCancellation {
        self.hardware.cancellation_handle()
    }

    fn client_disconnected(&mut self) {
        // Disconnect invalidates client-owned firmware claims only. The
        // service-owned motherboard policy keeps running without this client.
        self.hardware.cancellation_handle().cancel();
        self.hardware.client_disconnected();
        // Accepted service-owned monitoring writes persist beyond the TUI's
        // connection; only client-owned firmware claims become unverified.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn errors_are_cloneable_bounded_and_keep_unknown_outcome_semantics() {
        let error = HardwareError::with_kind(
            HardwareErrorKind::UnknownOutcome,
            format!("{}é", "x".repeat(MAX_ERROR_MESSAGE_BYTES - 1)),
        );

        assert_eq!(error.message().len(), MAX_ERROR_MESSAGE_BYTES - 1);
        assert_eq!(error.clone().kind(), HardwareErrorKind::UnknownOutcome);
        assert!(error.write_outcome_unknown());
        assert!(
            HardwareError::with_kind(HardwareErrorKind::Timeout, "timeout").write_outcome_unknown()
        );
        assert!(!HardwareError::new("internal").write_outcome_unknown());
        assert!(
            !HardwareError::with_kind(HardwareErrorKind::RestoreRequired, "restore required")
                .write_outcome_unknown()
        );
    }

    #[test]
    fn startup_cancelled_after_replay_never_dispatches_restored_lcd() {
        let root = tempfile::tempdir().unwrap();
        let display = KrakenDisplayWorker::with_sources(
            LiquidctlHardware::unavailable_for_test(),
            HostTelemetry::at_empty_test_root(root.path()),
        );
        let cancellation = HardwareCancellation::default();
        cancellation.cancel();
        let error = restore_startup_display(
            &display,
            Some((DeviceId::new("lcd"), KrakenDisplayMode::Cpu)),
            &cancellation,
        )
        .unwrap_err();
        assert!(error.is_service_shutdown());
        assert_eq!(display.snapshot(), KrakenDisplaySnapshot::default());
    }

    #[test]
    fn display_advertisement_never_selects_a_replacement_or_inherits_a_preset() {
        let old = DeviceId::new("lcd-old");
        let replacement = DeviceId::new("lcd-new");
        let idle = advertised_display(KrakenDisplaySnapshot::default(), Some(replacement.clone()));
        assert_eq!(idle.device_id, Some(replacement.clone()));
        assert_eq!(idle.mode, KrakenDisplayMode::BuiltinLiquid);
        assert!(idle.last_error.is_some()); // Eligible is not selected or applied.

        let selected = KrakenDisplaySnapshot {
            device_id: Some(old.clone()),
            mode: KrakenDisplayMode::Gpu,
            last_error: Some("LCD restore pending upload".into()),
        };
        let offline = advertised_display(selected.clone(), Some(replacement));
        assert_eq!(offline.device_id, None);
        assert_eq!(offline.mode, KrakenDisplayMode::Gpu);
        assert_eq!(offline.last_error, selected.last_error);
        assert_eq!(
            advertised_display(selected, Some(old.clone())).device_id,
            Some(old)
        );
    }

    #[test]
    fn disconnected_cooler_cannot_regain_a_stale_applied_claim_on_reconnect() {
        let id = DeviceId::new("aio");
        let channel = ChannelId::new("pump");
        let target = MonitorTarget::AioCurve {
            device_id: id.clone(),
            channel_id: channel,
            points: Vec::new(),
        };
        let mut actual = vec![(target, MonitoringActualState::Applied, None)];
        invalidate_offline_monitor_claims(&mut actual, &[]);
        assert_eq!(actual[0].1, MonitoringActualState::Unavailable);
        let reconnected = Device {
            id,
            name: "Kraken".into(),
            model: "Kraken".into(),
            kind: nzxt_cam_core::DeviceKind::LiquidCooler,
            online: true,
            readings: Vec::new(),
            cooling_channels: vec![nzxt_cam_core::CoolingChannel::unverified_firmware_curve(
                "pump",
                "Pump",
                nzxt_cam_core::TemperatureSource::Liquid,
                40,
                std::iter::repeat_n(50, 40),
            )],
        };
        invalidate_offline_monitor_claims(&mut actual, &[reconnected]);
        assert_eq!(actual[0].1, MonitoringActualState::Unavailable);
    }

    #[test]
    fn service_shutdown_is_a_typed_cause_not_diagnostic_text() {
        let ordinary = HardwareError::with_kind(
            HardwareErrorKind::Unavailable,
            "host-control service shutdown",
        );
        assert!(!ordinary.is_service_shutdown());
        let shutdown = HardwareError::service_shutdown();
        assert!(shutdown.clone().is_service_shutdown());
        assert_eq!(shutdown.kind(), HardwareErrorKind::Unavailable);
    }

    #[test]
    fn cancellation_clones_share_cancel_and_reset_state() {
        let cancellation = HardwareCancellation::default();
        let clone = cancellation.clone();

        clone.cancel();
        assert!(cancellation.is_cancelled());
        cancellation.reset();
        assert!(!clone.is_cancelled());
    }

    #[test]
    fn host_device_is_appended_without_reordering_or_changing_sequence() {
        use nzxt_cam_core::{DeviceKind, Reading, ReadingKind};

        let liquid = Device {
            id: DeviceId::new("liquid-first"),
            name: "Liquid".into(),
            model: "Model".into(),
            kind: DeviceKind::LiquidCooler,
            online: true,
            readings: Vec::new(),
            cooling_channels: Vec::new(),
        };
        let host = Device {
            id: DeviceId::new("host-telemetry"),
            name: "Host telemetry".into(),
            model: "Read-only host sensors".into(),
            kind: DeviceKind::FanController,
            online: true,
            readings: vec![Reading::new(
                "CPU Tctl",
                40.0,
                "°C",
                ReadingKind::Temperature,
            )],
            cooling_channels: Vec::new(),
        };
        let mut snapshot = HardwareSnapshot {
            devices: vec![liquid],
            sequence: 91,
            host_control: Default::default(),
            kraken_display: Default::default(),
            monitoring: Default::default(),
        };

        let host_control = HostControlSnapshot {
            state: nzxt_cam_core::HostControlState::Available,
            active_policy: None,
            last_error: None,
            channels: vec![nzxt_cam_core::HostChannelCapability {
                channel_id: ChannelId::new("front"),
                name: "Front".into(),
                minimum_duty_percent: 30,
            }],
        };
        append_service_state(&mut snapshot, Some(host), host_control.clone());

        assert_eq!(snapshot.sequence, 91);
        assert_eq!(snapshot.devices[0].id, DeviceId::new("liquid-first"));
        assert_eq!(snapshot.devices[1].id, DeviceId::new("host-telemetry"));
        assert!(snapshot.devices[1].cooling_channels.is_empty());
        assert_eq!(snapshot.host_control, host_control);
    }
}
