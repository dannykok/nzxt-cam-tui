//! End-to-end IPC tests: TUI backend -> real server -> real host worker/engine.
//! Every hardware and persistence boundary below is an in-memory fake.

use std::{
    collections::BTreeMap,
    os::unix::net::UnixListener as StdUnixListener,
    path::PathBuf,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

use nzxt_cam_hwd::{
    config::{HostChannelConfig, HostControlConfig},
    hardware::{HardwareCancellation, HardwareError, HardwareErrorKind, HardwareOperations},
    host_control::{
        HostControlEngine, HostControlShutdownHandle, HostControlSysfs, HostControlWorker,
        HostSensorSample, HostSensorSource, MonotonicTimeSource, RecoveryRecord, RecoveryStore,
        SystemMonotonicTime, TimedTemperature,
    },
    it8689::{BOARD_NAME, BOARD_VENDOR, CHIP_ADDRESS, CHIP_NAME, PLATFORM_COMPONENT},
    server::serve_until,
};
use tokio::{net::UnixListener, sync::oneshot};

use super::linux::IpcBackend;
use crate::{
    app::{App, AppCommand, StatusKind},
    backend::{BackendErrorKind, HardwareBackend},
    model::{
        ChannelId, CurvePoint, DeviceId, HardwareSnapshot, HostChannelPolicy, HostControlPolicy,
        HostControlSnapshot, HostControlState, HostCurve, HostCurvePoint, HostTemperatureSource,
    },
};

#[derive(Clone, Debug, PartialEq, Eq)]
enum Action {
    Discover,
    ReadFan(u8),
    ReadPwm(u8),
    ReadEnable(u8),
    WritePwm(u8, u8),
    WriteEnable(u8, u8),
    Load,
    Persist,
    Remove,
}

#[derive(Debug)]
struct Effects {
    actions: Vec<Action>,
    pwm: BTreeMap<u8, u8>,
    mode: BTreeMap<u8, u8>,
    rpm: BTreeMap<u8, u64>,
    record: Option<RecoveryRecord>,
    fail_bios_channel: Option<u8>,
}

impl Default for Effects {
    fn default() -> Self {
        Self {
            actions: Vec::new(),
            pwm: [(3, 63), (4, 63)].into(),
            mode: [(3, 2), (4, 2)].into(),
            rpm: [(3, 900), (4, 900)].into(),
            record: None,
            fail_bios_channel: None,
        }
    }
}

struct FakeSysfs(Arc<Mutex<Effects>>);
impl HostControlSysfs for FakeSysfs {
    fn discover_exact(&mut self) -> Result<(), HardwareError> {
        self.0.lock().unwrap().actions.push(Action::Discover);
        Ok(())
    }
    fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError> {
        let mut state = self.0.lock().unwrap();
        state.actions.push(Action::ReadPwm(channel));
        Ok(state.pwm[&channel])
    }
    fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError> {
        let mut state = self.0.lock().unwrap();
        state.actions.push(Action::ReadEnable(channel));
        Ok(state.mode[&channel])
    }
    fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError> {
        let mut state = self.0.lock().unwrap();
        state.actions.push(Action::ReadFan(channel));
        Ok(state.rpm[&channel])
    }
    fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
        let mut state = self.0.lock().unwrap();
        assert!(
            state.record.is_some(),
            "PWM write before recovery persistence"
        );
        assert_eq!(state.mode[&channel], 1, "PWM write outside manual mode");
        state.actions.push(Action::WritePwm(channel, value));
        state.pwm.insert(channel, value);
        Ok(())
    }
    fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
        let mut state = self.0.lock().unwrap();
        assert!(
            state.record.is_some(),
            "mode write before recovery persistence"
        );
        state.actions.push(Action::WriteEnable(channel, value));
        if value == 2 && state.fail_bios_channel == Some(channel) {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                "fake BIOS mode write failure",
            ));
        }
        state.mode.insert(channel, value);
        Ok(())
    }
}

struct FakeRecovery(Arc<Mutex<Effects>>);
impl RecoveryStore for FakeRecovery {
    fn load(&mut self) -> Result<Option<RecoveryRecord>, HardwareError> {
        let mut state = self.0.lock().unwrap();
        state.actions.push(Action::Load);
        Ok(state.record.clone())
    }
    fn persist(&mut self, record: &RecoveryRecord) -> Result<(), HardwareError> {
        let mut state = self.0.lock().unwrap();
        state.actions.push(Action::Persist);
        assert!(state.record.is_none());
        state.record = Some(record.clone());
        Ok(())
    }
    fn remove(&mut self) -> Result<(), HardwareError> {
        let mut state = self.0.lock().unwrap();
        state.actions.push(Action::Remove);
        state.record = None;
        Ok(())
    }
}

struct FakeClock(Arc<AtomicU64>);
impl MonotonicTimeSource for FakeClock {
    fn now(&mut self) -> Result<Duration, HardwareError> {
        Ok(Duration::from_secs(self.0.load(Ordering::Acquire)))
    }
}

struct FakeSensors;
impl HostSensorSource for FakeSensors {
    fn sample(&mut self, now: Duration) -> HostSensorSample {
        HostSensorSample {
            cpu: Some(TimedTemperature {
                millidegrees: 40_000,
                sampled_at: now,
            }),
            gpu: None,
        }
    }
}

#[derive(Default)]
struct StartupState {
    blocked: AtomicBool,
    calls: AtomicUsize,
    disconnects: AtomicUsize,
    cancellation: HardwareCancellation,
}

// No HardwareManager: neither liquidctl nor production telemetry is instantiated.
struct WorkerHardware {
    worker: HostControlWorker,
    cancellation: HardwareCancellation,
    startup: Arc<StartupState>,
}
impl HardwareOperations for WorkerHardware {
    fn snapshot(&mut self) -> Result<HardwareSnapshot, HardwareError> {
        Ok(HardwareSnapshot {
            devices: Vec::new(),
            sequence: 1,
            host_control: self.worker.snapshot(),
            kraken_display: Default::default(),
            monitoring: Default::default(),
        })
    }
    fn apply_firmware_curve(
        &mut self,
        _device: &DeviceId,
        _channel: &ChannelId,
        _points: &[CurvePoint],
    ) -> Result<(), HardwareError> {
        Err(HardwareError::with_kind(
            HardwareErrorKind::Unsupported,
            "no firmware fake",
        ))
    }
    fn initialize_host_control(&mut self) -> Result<(), HardwareError> {
        self.startup.calls.fetch_add(1, Ordering::AcqRel);
        while self.startup.blocked.load(Ordering::Acquire) && !self.cancellation.is_cancelled() {
            thread::sleep(Duration::from_millis(2));
        }
        if self.cancellation.is_cancelled() {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                "test service shutdown",
            ));
        }
        self.worker.initialize()
    }
    fn start_host_control(&mut self, policy: &HostControlPolicy) -> Result<(), HardwareError> {
        self.worker.start(policy)
    }
    fn update_host_control(&mut self, policies: &[HostChannelPolicy]) -> Result<(), HardwareError> {
        self.worker.update(policies)
    }
    fn stop_host_control(&mut self) -> Result<(), HardwareError> {
        self.worker.stop()
    }
    fn host_control_snapshot(&self) -> HostControlSnapshot {
        self.worker.snapshot()
    }
    fn host_control_shutdown_handle(&self) -> HostControlShutdownHandle {
        self.worker.shutdown_handle()
    }
    fn cancellation_handle(&self) -> HardwareCancellation {
        self.cancellation.clone()
    }
    fn client_disconnected(&mut self) {
        self.startup.disconnects.fetch_add(1, Ordering::AcqRel);
        self.cancellation.cancel();
        // The host worker belongs to the service, not this connection.
    }
}

static NEXT_SOCKET: AtomicU64 = AtomicU64::new(0);
struct Service {
    path: PathBuf,
    shutdown: Option<oneshot::Sender<()>>,
    join: Option<thread::JoinHandle<()>>,
    effects: Arc<Mutex<Effects>>,
    clock: Arc<AtomicU64>,
}
impl Service {
    fn new(channels: &[u8]) -> Self {
        Self::with_clock(channels, None)
    }

    fn with_clock(channels: &[u8], source: Option<Box<dyn MonotonicTimeSource>>) -> Self {
        Self::with_startup(channels, source, Arc::new(StartupState::default()))
    }

    fn with_startup(
        channels: &[u8],
        source: Option<Box<dyn MonotonicTimeSource>>,
        startup: Arc<StartupState>,
    ) -> Self {
        let path = std::env::temp_dir().join(format!(
            "nzxt-tui-real-host-{}-{}.sock",
            std::process::id(),
            NEXT_SOCKET.fetch_add(1, Ordering::Relaxed)
        ));
        let listener = StdUnixListener::bind(&path).unwrap();
        listener.set_nonblocking(true).unwrap();
        let effects = Arc::new(Mutex::new(Effects::default()));
        let clock = Arc::new(AtomicU64::new(0));
        let engine = HostControlEngine::with_dependencies(
            Some(HostControlConfig {
                board_vendor: BOARD_VENDOR.into(),
                board_name: BOARD_NAME.into(),
                chip_name: CHIP_NAME.into(),
                chip_address: CHIP_ADDRESS,
                platform_component: PLATFORM_COMPONENT.into(),
                channels: channels
                    .iter()
                    .map(|&number| HostChannelConfig {
                        id: ChannelId::new(format!("fan{number}")),
                        name: format!("Fan {number}"),
                        pwm_channel: number,
                        fan_channel: number,
                        minimum_duty_percent: 30,
                    })
                    .collect(),
            }),
            None,
            Box::new(FakeSysfs(Arc::clone(&effects))),
            Box::new(FakeRecovery(Arc::clone(&effects))),
            Box::new(FakeSensors),
            source.unwrap_or_else(|| Box::new(FakeClock(Arc::clone(&clock)))),
        );
        let hardware = WorkerHardware {
            worker: HostControlWorker::new(engine),
            cancellation: startup.cancellation.clone(),
            startup,
        };
        let (shutdown, receiver) = oneshot::channel();
        let join = thread::spawn(move || {
            let runtime = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap();
            runtime.block_on(async {
                serve_until(UnixListener::from_std(listener).unwrap(), hardware, async {
                    let _ = receiver.await;
                })
                .await
                .unwrap();
            });
        });
        Self {
            path,
            shutdown: Some(shutdown),
            join: Some(join),
            effects,
            clock,
        }
    }
    fn connect(&self) -> IpcBackend {
        let mut client = IpcBackend::connect_to(&self.path).unwrap();
        Self::ready_snapshot(&mut client);
        client
    }
    fn ready_snapshot(client: &mut IpcBackend) -> HardwareSnapshot {
        let deadline = Instant::now() + Duration::from_secs(4);
        loop {
            match client.refresh() {
                Ok(snapshot) => return snapshot,
                Err(error)
                    if error.kind() == BackendErrorKind::Unavailable
                        && error.to_string().contains("initializing")
                        && Instant::now() < deadline =>
                {
                    thread::sleep(Duration::from_millis(2))
                }
                Err(error) => panic!("service did not finish initialization: {error}"),
            }
        }
    }
    fn connect_after_disconnect(&self) -> IpcBackend {
        let deadline = Instant::now() + Duration::from_secs(4);
        loop {
            match IpcBackend::connect_to(&self.path) {
                Ok(client) => return client,
                Err(error) if Instant::now() < deadline => {
                    assert_eq!(error.kind(), BackendErrorKind::Unavailable);
                    thread::sleep(Duration::from_millis(10));
                }
                Err(error) => panic!("could not reconnect after disconnect: {error}"),
            }
        }
    }
    fn state(&self) -> std::sync::MutexGuard<'_, Effects> {
        self.effects.lock().unwrap()
    }
    fn wait_for(&self, predicate: impl Fn(&Effects) -> bool) {
        let deadline = Instant::now() + Duration::from_secs(4);
        while !predicate(&self.state()) {
            assert!(
                Instant::now() < deadline,
                "worker did not settle: {:?}",
                self.state()
            );
            thread::sleep(Duration::from_millis(10));
        }
    }
}
impl Drop for Service {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(join) = self.join.take() {
            join.join().unwrap();
        }
        std::fs::remove_file(&self.path).unwrap();
    }
}

fn policy(channels: &[u8]) -> HostControlPolicy {
    HostControlPolicy {
        channels: channels
            .iter()
            .map(|number| HostChannelPolicy {
                channel_id: ChannelId::new(format!("fan{number}")),
                curve: HostCurve {
                    source: HostTemperatureSource::Cpu,
                    // 50% at the safe 40C sensor; a full-range 100% endpoint remains editable.
                    points: (0..40)
                        .map(|index| HostCurvePoint {
                            temperature_millidegrees: 22_000 + index * 2_000,
                            duty_percent: if index == 39 { 100 } else { 50 },
                        })
                        .collect(),
                },
            })
            .collect(),
    }
}

fn assert_gentle_writes(state: &Effects, channels: &[u8], removed: bool) {
    let persisted = state
        .actions
        .iter()
        .position(|action| *action == Action::Persist)
        .unwrap();
    assert!(
        state.actions[..persisted]
            .iter()
            .all(|action| !matches!(action, Action::WritePwm(..) | Action::WriteEnable(..)))
    );
    for number in [3, 4] {
        assert!(
            state.actions[persisted..].contains(&Action::ReadFan(number)),
            "fan{number} RPM was not independently checked after persistence"
        );
    }
    for &channel in channels {
        let pwm_writes = state
            .actions
            .iter()
            .filter_map(|action| match action {
                Action::WritePwm(number, value) if *number == channel => Some(*value),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert!(
            pwm_writes.contains(&128),
            "missing gentle manual entry for {channel}: {pwm_writes:?}"
        );
        assert!(
            pwm_writes.contains(&63),
            "missing original PWM for {channel}: {pwm_writes:?}"
        );
        assert!(state.actions.contains(&Action::WriteEnable(channel, 1)));
    }
    assert!(
        !state
            .actions
            .iter()
            .any(|action| matches!(action, Action::WriteEnable(_, 0) | Action::WritePwm(_, 255)))
    );
    assert_eq!(state.actions.contains(&Action::Remove), removed);
}

#[test]
fn prompt_ipc_handshake_exposes_initializing_without_cancelling_or_repeating_startup() {
    let startup = Arc::new(StartupState {
        blocked: AtomicBool::new(true),
        ..Default::default()
    });
    let service = Service::with_startup(&[3], None, Arc::clone(&startup));
    let deadline = Instant::now() + Duration::from_secs(4);
    while startup.calls.load(Ordering::Acquire) == 0 {
        assert!(Instant::now() < deadline, "initializer did not start");
        thread::sleep(Duration::from_millis(2));
    }
    let initial_actions = service.state().actions.clone();
    let before = Instant::now();
    let mut client = IpcBackend::connect_to(&service.path).unwrap();
    assert!(before.elapsed() < Duration::from_secs(1));
    for error in [
        client.refresh().unwrap_err(),
        client.start_host_control(&policy(&[3])).unwrap_err(),
    ] {
        assert_eq!(error.kind(), BackendErrorKind::Unavailable);
        assert!(error.to_string().contains("initializing"));
    }
    assert_eq!(service.state().actions, initial_actions);
    assert!(!startup.cancellation.is_cancelled());
    drop(client);
    let mut replacement = service.connect_after_disconnect();
    assert_eq!(
        replacement.refresh().unwrap_err().kind(),
        BackendErrorKind::Unavailable
    );
    assert!(!startup.cancellation.is_cancelled());
    assert_eq!(startup.disconnects.load(Ordering::Acquire), 0);
    startup.blocked.store(false, Ordering::Release);
    assert_eq!(
        Service::ready_snapshot(&mut replacement).host_control.state,
        HostControlState::Available
    );
    replacement.start_host_control(&policy(&[3])).unwrap();
    assert_eq!(
        replacement.refresh().unwrap().host_control.state,
        HostControlState::Running
    );
    replacement.stop_host_control().unwrap();
    assert_eq!(startup.calls.load(Ordering::Acquire), 1);
}

#[test]
fn normal_dual_start_stop_restores_both_and_acknowledges_only_verified_success() {
    let service = Service::new(&[3, 4]);
    let mut client = service.connect();
    let snapshot = client.refresh().unwrap();
    assert_eq!(snapshot.host_control.state, HostControlState::Available);
    let mut app = App::new(snapshot, client.name());
    let policy = policy(&[3, 4]);
    client.start_host_control(&policy).unwrap();
    app.host_start_succeeded(policy);
    {
        let state = service.state();
        let record = serde_json::to_value(state.record.as_ref().unwrap()).unwrap();
        assert_ne!(
            record["version"], 2,
            "normal recovery must not use legacy v2"
        );
        assert!(
            record["channels"]
                .as_array()
                .unwrap()
                .iter()
                .all(|channel| channel["original_pwm"] == 63)
        );
    }
    assert_eq!(
        client.refresh().unwrap().host_control.state,
        HostControlState::Running
    );
    client.stop_host_control().unwrap();
    app.host_stop_succeeded();
    assert_eq!(app.status.kind, StatusKind::Success);
    assert_eq!(app.status.text, "Fan control stopped");
    assert_eq!(
        client.refresh().unwrap().host_control.state,
        HostControlState::Available
    );
    let state = service.state();
    assert_gentle_writes(&state, &[3, 4], true);
    assert_eq!(state.pwm[&3], 63);
    assert_eq!(state.pwm[&4], 63);
    assert_eq!(state.mode[&3], 2);
    assert_eq!(state.mode[&4], 2);
    assert!(state.record.is_none());
}

#[test]
fn live_update_preserves_bios_session_other_group_and_recovery_record_until_stop() {
    let service = Service::new(&[3, 4]);
    let mut client = service.connect();
    let original = policy(&[3, 4]);
    client.start_host_control(&original).unwrap();
    let before = service.state();
    let old_record = serde_json::to_value(before.record.as_ref().unwrap()).unwrap();
    let old_actions = before.actions.len();
    drop(before);
    let mut change = original.channels[0].clone();
    change
        .curve
        .points
        .iter_mut()
        .take(39)
        .for_each(|point| point.duty_percent = 60);
    client.update_host_control(&[change.clone()]).unwrap();
    let snapshot = client.refresh().unwrap();
    assert_eq!(snapshot.host_control.state, HostControlState::Running);
    assert_eq!(snapshot.host_control.last_error, None);
    let running = snapshot.host_control.active_policy.unwrap();
    assert_eq!(running.channels[0], change);
    assert_eq!(running.channels[1], original.channels[1]);
    let after = service.state();
    assert_eq!(
        serde_json::to_value(after.record.as_ref().unwrap()).unwrap(),
        old_record
    );
    assert!(!after.actions[old_actions..].iter().any(|action| matches!(
        action,
        Action::Persist
            | Action::Remove
            | Action::WriteEnable(_, 2)
            | Action::WriteEnable(_, 1)
            | Action::WritePwm(_, 255)
            | Action::WritePwm(_, 128)
    )));
    assert_eq!(after.mode[&3], 1);
    assert_eq!(after.mode[&4], 1);
    drop(after);
    // The accepted curve takes effect in full on the next worker tick; the
    // other group's unchanged target does not move and entry is not repeated.
    service.clock.store(1, Ordering::Release);
    service.wait_for(|state| state.actions.contains(&Action::WritePwm(3, 153)));
    assert_eq!(service.state().pwm[&4], 128);
    for point in change.curve.points.iter_mut().take(39) {
        point.duty_percent = 30;
    }
    client.update_host_control(&[change.clone()]).unwrap();
    service.clock.store(2, Ordering::Release);
    service.wait_for(|state| state.actions.contains(&Action::WritePwm(3, 77)));
    {
        let state = service.state();
        assert_eq!(state.pwm[&4], 128);
        assert_eq!(
            serde_json::to_value(state.record.as_ref().unwrap()).unwrap(),
            old_record
        );
        let writes: Vec<_> = state
            .actions
            .iter()
            .filter_map(|action| match action {
                Action::WritePwm(3, value) => Some(*value),
                _ => None,
            })
            .collect();
        assert_eq!(writes, [128, 153, 77]);
    }
    client.stop_host_control().unwrap();
    assert_eq!(client.refresh().unwrap().host_control.last_error, None);
    assert_gentle_writes(&service.state(), &[3, 4], true);
}

#[test]
fn disconnect_and_forced_quit_keep_single_channel_running_until_explicit_stop() {
    for force in [false, true] {
        let service = Service::new(&[3]);
        let mut client = service.connect();
        let expected = policy(&[3]);
        client.start_host_control(&expected).unwrap();
        if force {
            client.cancellation_handle().unwrap().cancel();
        }
        drop(client);
        // Server cleanup is asynchronous: a new connection's snapshot ensures
        // the previous session has finished without assuming wall-clock timing.
        let mut replacement = service.connect_after_disconnect();
        let snapshot = replacement.refresh().unwrap();
        assert_eq!(snapshot.host_control.state, HostControlState::Running);
        assert_eq!(snapshot.host_control.active_policy, Some(expected.clone()));
        assert!(
            !service
                .state()
                .actions
                .iter()
                .any(|action| matches!(action, Action::Remove | Action::WriteEnable(3, 2)))
        );
        let mut snapshot = snapshot;
        snapshot.monitoring.opted_in = true;
        let mut app = App::new(snapshot, replacement.name());
        app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
        assert_eq!(app.host_editor.policy(), expected);
        assert_eq!(
            {
                app.handle_key_event(crossterm::event::KeyCode::Char('o').into());
                app.handle_key_event(crossterm::event::KeyCode::Char('t').into())
            },
            Some(crate::app::AppCommand::StopHostControl)
        );
        replacement.stop_host_control().unwrap();
        app.host_stop_succeeded();
        let state = service.state();
        assert_gentle_writes(&state, &[3], true);
        assert_eq!(state.mode[&3], 2);
        assert_eq!(state.pwm[&4], 63);
    }
}

#[test]
fn explicit_service_shutdown_restores_even_without_a_connected_tui() {
    let mut service = Service::new(&[3]);
    let mut client = service.connect();
    client.start_host_control(&policy(&[3])).unwrap();
    drop(client);
    service.shutdown.take().unwrap().send(()).unwrap();
    service.join.take().unwrap().join().unwrap();
    let state = service.state();
    assert_gentle_writes(&state, &[3], true);
    assert_eq!(state.mode[&3], 2);
    assert!(state.record.is_none());
}

#[test]
fn all_scope_real_ipc_is_one_atomic_batch_and_rejects_invalid_second_without_partial_apply() {
    let service = Service::new(&[3, 4]);
    let mut client = service.connect();
    let original = policy(&[3, 4]);
    client.start_host_control(&original).unwrap();
    let before = service.state();
    let old_record = serde_json::to_value(before.record.as_ref().unwrap()).unwrap();
    let action_count = before.actions.len();
    drop(before);
    let mut snapshot = client.refresh().unwrap();
    snapshot.monitoring.opted_in = true;
    let mut app = App::new(snapshot, "LIVE");
    app.handle_key_event(crossterm::event::KeyCode::BackTab.into());
    for point in &mut app.host_editor.channels[0].curve.points[..39] {
        point.duty_percent = 60;
    }
    // The second group starts with a different draft; the chooser must use the selected curve.
    app.host_editor.channels[1].curve.points[0].duty_percent = 90;
    assert!(
        app.handle_key_event(crossterm::event::KeyCode::Enter.into())
            .is_none()
    );
    app.handle_key_event(crossterm::event::KeyCode::End.into());
    let Some(AppCommand::UpdateHostControl { channel_policies }) =
        app.handle_key_event(crossterm::event::KeyCode::Enter.into())
    else {
        panic!("expected one atomic batch")
    };
    assert_eq!(channel_policies.len(), 2);
    assert_eq!(channel_policies[0].curve, channel_policies[1].curve);
    let mut invalid = channel_policies.clone();
    invalid[1].curve.points[0].duty_percent = 20;
    assert!(client.update_host_control(&invalid).is_err());
    assert_eq!(
        client.refresh().unwrap().host_control.active_policy,
        Some(original.clone())
    );
    assert_eq!(service.state().actions.len(), action_count);
    client.update_host_control(&channel_policies).unwrap();
    let running = client
        .refresh()
        .unwrap()
        .host_control
        .active_policy
        .unwrap();
    assert_eq!(running.channels, channel_policies);
    {
        let state = service.state();
        assert_eq!(
            serde_json::to_value(state.record.as_ref().unwrap()).unwrap(),
            old_record
        );
        assert!(!state.actions[action_count..].iter().any(|a| matches!(
            a,
            Action::Persist | Action::Remove | Action::WriteEnable(_, _) | Action::WritePwm(_, _)
        )));
    }
    service.clock.store(1, Ordering::Release);
    service.wait_for(|state| {
        state.actions.contains(&Action::WritePwm(3, 153))
            && state.actions.contains(&Action::WritePwm(4, 153))
    });
    client.stop_host_control().unwrap();
}

#[test]
fn requested_full_curve_applies_on_next_tick_and_can_drop_directly_to_minimum() {
    let service = Service::new(&[3]);
    let mut client = service.connect();
    let mut full_target = policy(&[3]);
    for point in &mut full_target.channels[0].curve.points {
        point.duty_percent = 100;
    }
    client.start_host_control(&full_target).unwrap();
    service.clock.store(1, Ordering::Release);
    service.wait_for(|state| state.actions.contains(&Action::WritePwm(3, 255)));
    let mut minimum = full_target.channels[0].clone();
    for point in minimum.curve.points.iter_mut().take(39) {
        point.duty_percent = 30;
    }
    client.update_host_control(&[minimum.clone()]).unwrap();
    service.clock.store(2, Ordering::Release);
    service.wait_for(|state| state.actions.contains(&Action::WritePwm(3, 77)));
    assert_eq!(
        client.refresh().unwrap().host_control.state,
        HostControlState::Running
    );
    client.stop_host_control().unwrap();
    let state = service.state();
    let writes = state
        .actions
        .iter()
        .filter_map(|action| match action {
            Action::WritePwm(3, value) => Some(*value),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(writes, [128, 255, 77, 63]);
    assert!(!state.actions.contains(&Action::WriteEnable(3, 0)));
    assert_eq!(state.mode[&3], 2);
    assert_eq!(state.pwm[&4], 63);
    assert!(state.record.is_none());
}

#[test]
fn advancing_monotonic_clock_keeps_the_real_worker_running_after_sensor_sampling() {
    // Only the clock is real; all fan, sensor, persistence and socket paths are
    // fake/local. A clock frozen within a tick hides future-timestamp bugs.
    let service = Service::with_clock(&[3, 4], Some(Box::new(SystemMonotonicTime::new())));
    let mut client = service.connect();
    let mut target = policy(&[3, 4]);
    for channel in &mut target.channels {
        for point in &mut channel.curve.points {
            point.duty_percent = 100;
        }
    }
    client.start_host_control(&target).unwrap();
    service.wait_for(|state| {
        state.actions.contains(&Action::WritePwm(3, 255))
            && state.actions.contains(&Action::WritePwm(4, 255))
    });
    assert_eq!(
        client.refresh().unwrap().host_control.state,
        HostControlState::Running
    );
    client.stop_host_control().unwrap();
    let state = service.state();
    for number in [3, 4] {
        let writes: Vec<_> = state
            .actions
            .iter()
            .filter_map(|action| match action {
                Action::WritePwm(channel, value) if *channel == number => Some(*value),
                _ => None,
            })
            .collect();
        assert_eq!(writes, [128, 255, 63]);
        assert_eq!(state.mode[&number], 2);
        assert!(!state.actions.contains(&Action::WriteEnable(number, 0)));
    }
    assert!(state.record.is_none());
}

#[test]
fn independent_fan_guard_prevents_any_mutation_when_unconfigured_fan_is_stalled() {
    let service = Service::new(&[3]);
    service.effects.lock().unwrap().rpm.insert(4, 0);
    let mut client = service.connect();
    assert!(client.start_host_control(&policy(&[3])).is_err());
    let state = service.state();
    assert!(state.actions.contains(&Action::ReadFan(4)));
    assert!(!state.actions.iter().any(|action| matches!(
        action,
        Action::Persist | Action::WritePwm(..) | Action::WriteEnable(..)
    )));
    assert!(state.record.is_none());
}

#[test]
fn partial_bios_failure_requires_restoration_and_retains_record_without_false_stop_success() {
    let service = Service::new(&[3, 4]);
    service.effects.lock().unwrap().fail_bios_channel = Some(4);
    let mut client = service.connect();
    let mut app = App::new(client.refresh().unwrap(), client.name());
    let policy = policy(&[3, 4]);
    client.start_host_control(&policy).unwrap();
    app.host_start_succeeded(policy);
    let error = client.stop_host_control().unwrap_err();
    app.host_operation_failed(&error);
    assert_eq!(error.kind(), BackendErrorKind::RestoreRequired);
    assert_eq!(app.host_control_state, HostControlState::RestoreRequired);
    assert_eq!(app.status.kind, StatusKind::Error);
    assert!(!app.status.text.contains("Fan control stopped"));
    // RestoreRequired discards the connection. Wait for its command slot to
    // drain before checking an authoritative snapshot through a new client.
    drop(client);
    let mut client = service.connect_after_disconnect();
    assert_eq!(
        client.refresh().unwrap().host_control.state,
        HostControlState::RestoreRequired
    );
    let state = service.state();
    assert_gentle_writes(&state, &[3, 4], false);
    assert_eq!(state.mode[&3], 2);
    assert_eq!(state.pwm[&3], 63);
    assert!(state.record.is_some());
}
