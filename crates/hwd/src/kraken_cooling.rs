//! Service-owned CPU cooling. Every write retains an autonomous coolant curve.
use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex, mpsc},
    thread::{self, JoinHandle},
    time::Duration,
};

use nzxt_cam_core::{
    ChannelId, CoolingChannel, CurvePoint, CurveState, DeviceId, MonitoringActualState,
    TemperatureSource, curve::valid_kraken_curve,
};

use crate::{
    config::HardwareConfig,
    hardware::{HardwareError, HardwareErrorKind, HardwareOperations},
    liquidctl::LiquidctlHardware,
    monitor_intent::{FileMonitorIntentStore, MonitorTarget},
    telemetry::HostTelemetry,
};

type Key = (DeviceId, ChannelId);
pub(crate) type IntentStore = Arc<Mutex<FileMonitorIntentStore>>;
const TICK: Duration = Duration::from_secs(2);

#[derive(Clone)]
pub(crate) struct Selection {
    pub channel: CoolingChannel,
    pub state: MonitoringActualState,
    pub error: Option<String>,
    liquid: Vec<CurvePoint>,
    duty: Option<u8>,
    restore_pending: Option<String>,
}

enum Command {
    Preflight(
        Key,
        TemperatureSource,
        Vec<CurvePoint>,
        mpsc::Sender<Result<bool, HardwareError>>,
    ),
    Apply(
        Key,
        TemperatureSource,
        Vec<CurvePoint>,
        mpsc::Sender<Result<(), HardwareError>>,
    ),
    Shutdown,
}

pub(crate) struct KrakenCoolingWorker {
    commands: mpsc::Sender<Command>,
    selections: Arc<Mutex<BTreeMap<Key, Selection>>>,
    thread: Option<JoinHandle<()>>,
}

impl KrakenCoolingWorker {
    pub fn new(config: HardwareConfig, io_lock: Arc<Mutex<()>>, store: IntentStore) -> Self {
        Self::with_sources(
            LiquidctlHardware::with_io_lock(io_lock),
            HostTelemetry::new(config),
            store,
        )
    }

    pub(crate) fn with_sources(
        mut hardware: LiquidctlHardware,
        mut host: HostTelemetry,
        store: IntentStore,
    ) -> Self {
        let (commands, receive) = mpsc::channel();
        let selections = Arc::new(Mutex::new(BTreeMap::<Key, Selection>::new()));
        let shared = selections.clone();
        let thread = thread::Builder::new()
            .name("kraken-cooling".into())
            .spawn(move || {
                let mut active = BTreeMap::new();
                let mut pending = None;
                loop {
                    let next = pending
                        .take()
                        .map(Ok)
                        .unwrap_or_else(|| receive.recv_timeout(TICK));
                    match next {
                        Ok(Command::Preflight(key, source, points, reply)) => {
                            let result = preflight(&mut hardware, &mut host, &key, source, &points);
                            let _ = reply.send(result);
                        }
                        Ok(Command::Apply(key, source, points, reply)) => {
                            let result =
                                apply(&mut hardware, &mut host, &mut active, &key, source, points);
                            *shared.lock().unwrap() = active.clone();
                            let _ = reply.send(result);
                        }
                        Ok(Command::Shutdown) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                            // The envelope remains safe even if shutdown is interrupted.
                            for (key, selection) in &mut active {
                                if selection.channel.source == TemperatureSource::Cpu
                                    && (selection.error.is_none()
                                        || selection.restore_pending.is_some())
                                {
                                    restore(
                                        &mut hardware,
                                        &store,
                                        key,
                                        selection,
                                        "service stopped; liquid control restored",
                                    );
                                }
                            }
                            break;
                        }
                        Err(mpsc::RecvTimeoutError::Timeout) => {
                            for (key, selection) in &mut active {
                                if selection.channel.source != TemperatureSource::Cpu
                                    || (selection.error.is_some()
                                        && selection.restore_pending.is_none())
                                {
                                    continue;
                                }
                                tick(&mut hardware, &mut host, &store, key, selection);
                                if let Ok(command) = receive.try_recv() {
                                    pending = Some(command);
                                    break;
                                }
                            }
                            *shared.lock().unwrap() = active.clone();
                        }
                    }
                }
            })
            .expect("could not start Kraken cooling worker");
        Self {
            commands,
            selections,
            thread: Some(thread),
        }
    }

    pub fn preflight(
        &self,
        id: &DeviceId,
        channel: &ChannelId,
        source: TemperatureSource,
        points: &[CurvePoint],
    ) -> Result<bool, HardwareError> {
        let (reply, receive) = mpsc::channel();
        self.commands
            .send(Command::Preflight(
                (id.clone(), channel.clone()),
                source,
                points.to_vec(),
                reply,
            ))
            .map_err(|_| stopped())?;
        receive.recv().map_err(|_| stopped())?
    }

    pub fn apply(
        &self,
        id: &DeviceId,
        channel: &ChannelId,
        source: TemperatureSource,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError> {
        let (reply, receive) = mpsc::channel();
        self.commands
            .send(Command::Apply(
                (id.clone(), channel.clone()),
                source,
                points.to_vec(),
                reply,
            ))
            .map_err(|_| stopped())?;
        receive.recv().map_err(|_| stopped())?
    }

    pub fn selections(&self) -> BTreeMap<Key, Selection> {
        let mut selections = self.selections.lock().unwrap().clone();
        if self
            .thread
            .as_ref()
            .is_some_and(|thread| thread.is_finished())
        {
            for selection in selections
                .values_mut()
                .filter(|s| s.channel.source == TemperatureSource::Cpu)
            {
                selection.channel.curve_state = CurveState::Unverified;
                selection.state = MonitoringActualState::ReviewRequired;
                selection.error =
                    Some("Kraken CPU cooling worker stopped; review cooling settings".into());
            }
        }
        selections
    }
}

impl Drop for KrakenCoolingWorker {
    fn drop(&mut self) {
        let _ = self.commands.send(Command::Shutdown);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn stopped() -> HardwareError {
    HardwareError::with_kind(
        HardwareErrorKind::Unavailable,
        "Kraken cooling worker stopped",
    )
}
fn invalid() -> HardwareError {
    HardwareError::with_kind(
        HardwareErrorKind::InvalidData,
        "invalid Kraken source or 40-point curve",
    )
}

fn cpu(host: &mut HostTelemetry) -> Result<f64, HardwareError> {
    host.cpu_temperature_celsius()
        .filter(|t| t.is_finite() && (0.0..=150.0).contains(t))
        .ok_or_else(|| {
            HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                "CPU temperature unavailable",
            )
        })
}

fn preflight(
    hardware: &mut LiquidctlHardware,
    host: &mut HostTelemetry,
    key: &Key,
    source: TemperatureSource,
    points: &[CurvePoint],
) -> Result<bool, HardwareError> {
    if !valid_kraken_curve(source, points, 0) {
        return Err(invalid());
    }
    let liquid = if source == TemperatureSource::Liquid {
        points.to_vec()
    } else {
        safety_curve(0, &[])
    };
    if !hardware.preflight_resume_curve(&key.0, &key.1, &liquid)? {
        return Ok(false);
    }
    let channel = hardware
        .cooling_channel(&key.0, &key.1)
        .ok_or_else(stopped)?;
    if !valid_kraken_curve(source, points, channel.min_duty) {
        return Err(invalid());
    }
    if source == TemperatureSource::Cpu {
        cpu(host)?;
    }
    Ok(true)
}

fn apply(
    hardware: &mut LiquidctlHardware,
    host: &mut HostTelemetry,
    active: &mut BTreeMap<Key, Selection>,
    key: &Key,
    source: TemperatureSource,
    points: Vec<CurvePoint>,
) -> Result<(), HardwareError> {
    let mut channel = hardware
        .cooling_channel(&key.0, &key.1)
        .ok_or_else(stopped)?;
    if !valid_kraken_curve(source, &points, channel.min_duty) {
        return Err(invalid());
    }
    let previous = active.remove(key);
    let liquid = if source == TemperatureSource::Liquid {
        points.clone()
    } else {
        safety_curve(
            channel.min_duty,
            previous.as_ref().map_or(&channel.points, |s| &s.liquid),
        )
    };
    channel.source = source;
    channel.points = points;
    let mut selection = Selection {
        channel,
        liquid,
        duty: None,
        restore_pending: None,
        state: MonitoringActualState::Pending,
        error: None,
    };
    let result = (|| {
        let duty = if source == TemperatureSource::Cpu {
            Some(cpu_duty(&selection.channel.points, cpu(host)?, None))
        } else {
            None
        };
        let curve = duty.map_or_else(
            || selection.channel.points.clone(),
            |d| envelope(&selection.liquid, d),
        );
        hardware.apply_firmware_curve(&key.0, &key.1, &curve)?;
        selection.duty = duty;
        Ok(())
    })();
    selection.channel.curve_state = if result.is_ok() {
        CurveState::Applied
    } else {
        CurveState::Unverified
    };
    selection.state = if result.is_ok() {
        MonitoringActualState::Applied
    } else {
        MonitoringActualState::ReviewRequired
    };
    selection.error = result
        .as_ref()
        .err()
        .map(|e: &HardwareError| e.message().to_owned());
    active.insert(key.clone(), selection);
    result
}

fn target(key: &Key, selection: &Selection) -> MonitorTarget {
    MonitorTarget::AioCurve {
        source: TemperatureSource::Cpu,
        device_id: key.0.clone(),
        channel_id: key.1.clone(),
        points: selection.channel.points.clone(),
    }
}

fn durable_write(
    hardware: &mut LiquidctlHardware,
    store: &IntentStore,
    key: &Key,
    selection: &Selection,
    points: &[CurvePoint],
) -> Result<bool, HardwareError> {
    // An interactive transaction may own the store while waiting for this worker.
    // Never block the worker on that mutex: skip this background update instead.
    let Ok(mut store) = store.try_lock() else {
        return Ok(false);
    };
    let target = target(key, selection);
    store.mark_in_flight(&target)?;
    hardware.apply_firmware_curve(&key.0, &key.1, points)?;
    if let Err(error) = store.mark_ready(&target) {
        let _ = store.mark_in_flight(&target);
        return Err(error);
    }
    Ok(true)
}

fn restore(
    hardware: &mut LiquidctlHardware,
    store: &IntentStore,
    key: &Key,
    selection: &mut Selection,
    reason: &str,
) {
    let result = hardware
        .preflight_resume_curve(&key.0, &key.1, &selection.liquid)
        .and_then(|online| {
            if online {
                durable_write(hardware, store, key, selection, &selection.liquid)
            } else {
                Err(stopped())
            }
        });
    record_restore(selection, result, reason);
}

fn record_restore(selection: &mut Selection, result: Result<bool, HardwareError>, reason: &str) {
    selection.restore_pending = matches!(result, Ok(false)).then(|| reason.to_owned());
    selection.channel.curve_state = CurveState::Unverified;
    selection.state = if matches!(result, Ok(true)) {
        MonitoringActualState::Unavailable
    } else {
        MonitoringActualState::ReviewRequired
    };
    selection.error = Some(match result {
        Ok(true) => reason.into(),
        Ok(false) => format!("{reason}; liquid restore deferred"),
        Err(e) => format!("{reason}; liquid restore failed: {e}"),
    });
}

fn tick(
    hardware: &mut LiquidctlHardware,
    host: &mut HostTelemetry,
    store: &IntentStore,
    key: &Key,
    selection: &mut Selection,
) {
    // Do not start USB work while an interactive transaction owns the store.
    if store.try_lock().is_err() {
        return;
    }
    if let Some(reason) = selection.restore_pending.clone() {
        restore(hardware, store, key, selection, &reason);
        return;
    }
    if selection.error.is_some() {
        return;
    }
    match cpu(host) {
        Ok(_) => {}
        Err(_) => {
            restore(
                hardware,
                store,
                key,
                selection,
                "CPU temperature unavailable; liquid control restored",
            );
            return;
        }
    }
    // Fresh USB identity checks on every tick, including unchanged duties.
    let preflight = hardware.preflight_resume_curve(&key.0, &key.1, &selection.liquid);
    let result = match preflight {
        Ok(true) => {
            // Resample after potentially slow USB work; never use a cached temperature.
            let temperature = match cpu(host) {
                Ok(temperature) => temperature,
                Err(_) => {
                    let result = durable_write(hardware, store, key, selection, &selection.liquid);
                    record_restore(
                        selection,
                        result,
                        "CPU temperature unavailable; liquid control restored",
                    );
                    return;
                }
            };
            let duty = cpu_duty(&selection.channel.points, temperature, selection.duty);
            if selection.duty == Some(duty) {
                return;
            }
            durable_write(
                hardware,
                store,
                key,
                selection,
                &envelope(&selection.liquid, duty),
            )
            .inspect(|written| {
                if *written {
                    selection.duty = Some(duty);
                }
            })
        }
        Ok(false) => Err(HardwareError::with_kind(
            HardwareErrorKind::Unavailable,
            "selected Kraken unavailable",
        )),
        Err(error) => Err(error),
    };
    if let Err(error) = result {
        selection.channel.curve_state = CurveState::Unverified;
        selection.state = MonitoringActualState::ReviewRequired;
        selection.error = Some(format!(
            "CPU cooling stopped: {error}; review cooling settings"
        ));
    }
}

/// Conservative coolant guard, never weakened by an earlier user curve.
fn safety_curve(minimum: u8, previous: &[CurvePoint]) -> Vec<CurvePoint> {
    let stops = [(20, 40), (30, 50), (35, 65), (40, 85), (45, 100), (59, 100)];
    let prior_valid = valid_kraken_curve(TemperatureSource::Liquid, previous, minimum);
    (20..60)
        .enumerate()
        .map(|(index, temperature)| CurvePoint {
            temperature,
            duty: interpolate(&stops, f64::from(temperature))
                .max(minimum)
                .max(if prior_valid { previous[index].duty } else { 0 }),
        })
        .collect()
}

fn envelope(liquid: &[CurvePoint], duty: u8) -> Vec<CurvePoint> {
    liquid
        .iter()
        .map(|point| CurvePoint {
            temperature: point.temperature,
            duty: point.duty.max(duty),
        })
        .collect()
}

fn cpu_duty(points: &[CurvePoint], temperature: f64, previous: Option<u8>) -> u8 {
    // Immediate upward response; avoid small downward oscillations. Emergency
    // response bypasses both the curve and hysteresis.
    if temperature >= 95.0 {
        return 100;
    }
    let stops: Vec<_> = points.iter().map(|p| (p.temperature, p.duty)).collect();
    let duty = interpolate(&stops, temperature);
    match previous {
        Some(old) if duty < old && old - duty < 3 => old,
        _ => duty,
    }
}

fn interpolate(stops: &[(u8, u8)], temperature: f64) -> u8 {
    if temperature <= f64::from(stops[0].0) {
        return stops[0].1;
    }
    for pair in stops.windows(2) {
        let (lo, hi) = (pair[0], pair[1]);
        if temperature <= f64::from(hi.0) {
            return (f64::from(lo.1)
                + f64::from(hi.1 - lo.1) * (temperature - f64::from(lo.0)) / f64::from(hi.0 - lo.0))
            .ceil() as u8;
        }
    }
    stops.last().unwrap().1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        liquidctl::CoolingTestState,
        monitor_intent::{SavedMonitorIntent, SavedTarget, WriteState},
    };
    use std::{fs, os::unix::fs::PermissionsExt, time::Instant};

    struct Fixture {
        root: tempfile::TempDir,
        hardware: LiquidctlHardware,
        host: HostTelemetry,
        store: IntentStore,
        device: Arc<Mutex<CoolingTestState>>,
        key: Key,
        points: Vec<CurvePoint>,
        active: BTreeMap<Key, Selection>,
    }
    impl Fixture {
        fn new() -> Self {
            let root = tempfile::tempdir().unwrap();
            fs::set_permissions(root.path(), fs::Permissions::from_mode(0o700)).unwrap();
            let hwmon = root.path().join("hwmon/hwmon0");
            fs::create_dir_all(&hwmon).unwrap();
            fs::write(hwmon.join("name"), "k10temp").unwrap();
            fs::write(hwmon.join("temp1_label"), "Tctl").unwrap();
            fs::write(hwmon.join("temp1_input"), "50000").unwrap();
            let device = Arc::new(Mutex::new(CoolingTestState::default()));
            let mut hardware = LiquidctlHardware::cooling_test_adapter(device.clone());
            let snapshot = hardware.snapshot().unwrap();
            let key = (snapshot.devices[0].id.clone(), ChannelId::new("pump"));
            let points: Vec<_> = (0..40usize)
                .map(|index| CurvePoint {
                    temperature: 22 + 2 * index as u8,
                    duty: 30 + (70 * index / 39) as u8,
                })
                .collect();
            let store = Arc::new(Mutex::new(FileMonitorIntentStore::at(
                root.path().join("monitor-intent.json"),
            )));
            store
                .lock()
                .unwrap()
                .persist(&SavedMonitorIntent {
                    version: 1,
                    opted_in: true,
                    auto_resume: true,
                    targets: vec![SavedTarget {
                        target: MonitorTarget::AioCurve {
                            source: TemperatureSource::Cpu,
                            device_id: key.0.clone(),
                            channel_id: key.1.clone(),
                            points: points.clone(),
                        },
                        state: WriteState::Ready,
                    }],
                })
                .unwrap();
            let host = HostTelemetry::at_empty_test_root(root.path());
            Self {
                root,
                hardware,
                host,
                store,
                device,
                key,
                points,
                active: BTreeMap::new(),
            }
        }
        fn temperature(&self, value: &str) {
            fs::write(self.root.path().join("hwmon/hwmon0/temp1_input"), value).unwrap();
        }
        fn start(&mut self) {
            assert!(
                preflight(
                    &mut self.hardware,
                    &mut self.host,
                    &self.key,
                    TemperatureSource::Cpu,
                    &self.points
                )
                .unwrap()
            );
            apply(
                &mut self.hardware,
                &mut self.host,
                &mut self.active,
                &self.key,
                TemperatureSource::Cpu,
                self.points.clone(),
            )
            .unwrap();
        }
        fn tick(&mut self) {
            tick(
                &mut self.hardware,
                &mut self.host,
                &self.store,
                &self.key,
                self.active.get_mut(&self.key).unwrap(),
            );
        }
        fn last_curve(&self) -> Vec<CurvePoint> {
            self.device.lock().unwrap().writes.last().unwrap()[5..]
                .as_chunks::<2>()
                .0
                .iter()
                .map(|p| CurvePoint {
                    temperature: p[0].parse().unwrap(),
                    duty: p[1].parse().unwrap(),
                })
                .collect()
        }
    }

    #[test]
    fn sensor_loss_restores_liquid_and_does_not_reuse_cached_cpu_values() {
        let mut f = Fixture::new();
        f.start();
        let original = f.last_curve();
        f.tick();
        assert_eq!(f.device.lock().unwrap().writes.len(), 1); // unchanged duty: no write
        f.temperature("95000");
        f.tick();
        assert!(f.last_curve().iter().all(|p| p.duty == 100));
        f.temperature("broken");
        f.tick();
        assert_eq!(f.last_curve(), f.active[&f.key].liquid);
        assert!(f.last_curve()[0].duty < original[0].duty);
        assert_eq!(f.active[&f.key].state, MonitoringActualState::Unavailable);
        assert!(
            f.active[&f.key]
                .error
                .as_ref()
                .unwrap()
                .contains("CPU temperature unavailable")
        );
        assert_eq!(
            f.store.lock().unwrap().load().unwrap().unwrap().targets[0].state,
            WriteState::Ready
        );
        let count = f.device.lock().unwrap().writes.len();
        f.temperature("99000");
        f.tick();
        assert_eq!(f.device.lock().unwrap().writes.len(), count); // explicit reapply needed
    }

    #[test]
    fn sensor_lost_during_usb_preflight_restores_instead_of_using_the_earlier_sample() {
        let mut f = Fixture::new();
        f.start();
        f.temperature("95000");
        f.device.lock().unwrap().after_status = Some((
            f.root.path().join("hwmon/hwmon0/temp1_input"),
            "invalid".into(),
        ));
        f.tick();
        assert_eq!(f.last_curve(), f.active[&f.key].liquid);
        assert_eq!(f.active[&f.key].state, MonitoringActualState::Unavailable);
        assert_eq!(f.device.lock().unwrap().writes.len(), 2);
    }

    #[test]
    fn deferred_sensor_fault_restore_retries_only_the_liquid_curve() {
        let mut f = Fixture::new();
        f.start();
        let store = f.store.clone();
        let guard = store.lock().unwrap();
        restore(
            &mut f.hardware,
            &f.store,
            &f.key,
            f.active.get_mut(&f.key).unwrap(),
            "sensor fault",
        );
        assert!(f.active[&f.key].restore_pending.is_some());
        assert_eq!(f.device.lock().unwrap().writes.len(), 1);
        drop(guard);
        f.temperature("95000");
        f.tick();
        assert_eq!(f.last_curve(), f.active[&f.key].liquid);
        assert!(f.active[&f.key].restore_pending.is_none());
        assert_eq!(f.active[&f.key].state, MonitoringActualState::Unavailable);
    }

    #[test]
    fn uncertain_background_write_blocks_replay_and_is_not_retried() {
        let mut f = Fixture::new();
        f.start();
        f.temperature("95000");
        f.device.lock().unwrap().fail_write = true;
        f.tick();
        assert_eq!(
            f.active[&f.key].state,
            MonitoringActualState::ReviewRequired
        );
        assert_eq!(
            f.store.lock().unwrap().load().unwrap().unwrap().targets[0].state,
            WriteState::InFlightUnknown
        );
        let count = f.device.lock().unwrap().writes.len();
        f.tick();
        assert_eq!(f.device.lock().unwrap().writes.len(), count);
    }

    #[test]
    fn unavailable_identity_or_busy_intent_store_never_writes() {
        let mut f = Fixture::new();
        f.start();
        f.temperature("95000");
        let store = f.store.clone();
        let guard = store.lock().unwrap();
        f.tick();
        assert_eq!(f.device.lock().unwrap().writes.len(), 1);
        drop(guard);
        f.device.lock().unwrap().offline = true;
        f.tick();
        assert_eq!(f.device.lock().unwrap().writes.len(), 1);
        assert_ne!(f.active[&f.key].state, MonitoringActualState::Applied);
    }

    #[test]
    fn switching_to_liquid_stops_cpu_writes_and_keeps_channels_independent() {
        let mut f = Fixture::new();
        f.start();
        let liquid = safety_curve(20, &[]);
        apply(
            &mut f.hardware,
            &mut f.host,
            &mut f.active,
            &f.key,
            TemperatureSource::Liquid,
            liquid.clone(),
        )
        .unwrap();
        assert_eq!(f.last_curve(), liquid);
        assert_eq!(f.active[&f.key].channel.source, TemperatureSource::Liquid);
        let fan = (f.key.0.clone(), ChannelId::new("fan"));
        assert!(
            preflight(
                &mut f.hardware,
                &mut f.host,
                &fan,
                TemperatureSource::Cpu,
                &f.points
            )
            .unwrap()
        );
        apply(
            &mut f.hardware,
            &mut f.host,
            &mut f.active,
            &fan,
            TemperatureSource::Cpu,
            f.points.clone(),
        )
        .unwrap();
        assert_eq!(f.active[&f.key].channel.source, TemperatureSource::Liquid);
        assert_eq!(f.active[&fan].channel.source, TemperatureSource::Cpu);
    }

    #[test]
    fn worker_runs_without_a_client_and_clean_shutdown_restores_liquid() {
        let f = Fixture::new();
        let hardware = LiquidctlHardware::cooling_test_adapter(f.device.clone());
        let host = HostTelemetry::at_empty_test_root(f.root.path());
        let worker = KrakenCoolingWorker::with_sources(hardware, host, f.store.clone());
        assert!(
            worker
                .preflight(&f.key.0, &f.key.1, TemperatureSource::Cpu, &f.points)
                .unwrap()
        );
        worker
            .apply(&f.key.0, &f.key.1, TemperatureSource::Cpu, &f.points)
            .unwrap();
        f.temperature("95000");
        let started = Instant::now();
        while f.device.lock().unwrap().writes.len() < 2
            && started.elapsed() < Duration::from_secs(4)
        {
            thread::sleep(Duration::from_millis(5));
        }
        assert!(f.last_curve().iter().all(|p| p.duty == 100));
        drop(worker);
        assert_eq!(f.last_curve()[0].duty, 40);
        assert_eq!(
            f.store.lock().unwrap().load().unwrap().unwrap().targets[0].state,
            WriteState::Ready
        );
    }

    #[test]
    fn cpu_preflight_rejects_missing_sensor_invalid_curve_and_gpu_source_without_writes() {
        let mut f = Fixture::new();
        f.temperature("broken");
        assert!(
            preflight(
                &mut f.hardware,
                &mut f.host,
                &f.key,
                TemperatureSource::Cpu,
                &f.points
            )
            .is_err()
        );
        assert!(
            preflight(
                &mut f.hardware,
                &mut f.host,
                &f.key,
                TemperatureSource::Gpu,
                &f.points
            )
            .is_err()
        );
        f.points[0].temperature = 20;
        assert!(
            preflight(
                &mut f.hardware,
                &mut f.host,
                &f.key,
                TemperatureSource::Cpu,
                &f.points
            )
            .is_err()
        );
        assert!(f.device.lock().unwrap().writes.is_empty());
    }

    #[test]
    fn envelope_retains_autonomous_liquid_protection_at_every_cpu_duty() {
        let safety = safety_curve(20, &[]);
        for duty in [20, 40, 80, 100] {
            let curve = envelope(&safety, duty);
            assert!(valid_kraken_curve(TemperatureSource::Liquid, &curve, 20));
            assert!(
                curve
                    .iter()
                    .zip(&safety)
                    .all(|(actual, guard)| actual.duty >= duty && actual.duty >= guard.duty)
            );
            assert_eq!(curve[25].duty, 100);
        }
        assert_eq!(safety_curve(20, &envelope(&safety, 90))[0].duty, 90);
    }
    #[test]
    fn cpu_curve_interpolates_and_responds_immediately_to_heat() {
        let points = vec![
            CurvePoint {
                temperature: 22,
                duty: 30,
            },
            CurvePoint {
                temperature: 60,
                duty: 70,
            },
            CurvePoint {
                temperature: 100,
                duty: 100,
            },
        ];
        assert_eq!(cpu_duty(&points, 41.0, None), 50);
        assert_eq!(cpu_duty(&points, 41.0, Some(52)), 52);
        assert_eq!(cpu_duty(&points, 41.0, Some(60)), 50);
        assert_eq!(cpu_duty(&points, 80.0, Some(40)), 85);
        assert_eq!(cpu_duty(&points, 95.0, Some(30)), 100);
    }
}
