use std::{
    collections::{BTreeMap, HashMap},
    io::{self, Read},
    path::PathBuf,
    process::{Command, Stdio},
    sync::{Arc, Mutex, MutexGuard, TryLockError},
    thread,
    time::{Duration, Instant},
};

#[cfg(unix)]
use std::os::unix::process::CommandExt;

use nzxt_cam_core::{
    ChannelId, CoolingChannel, CurvePoint, CurveState, Device, DeviceId, DeviceKind,
    HardwareSnapshot, HostChannelPolicy, HostControlPolicy, Reading, ReadingKind,
    TemperatureSource,
};
use nzxt_cam_protocol::{
    MONITORING_FIRMWARE_CURVE_POINTS,
    time_budget::{LIQUIDCTL_COMMAND_TIMEOUT, USB_LOCK_ACQUIRE_TIMEOUT, USB_LOCK_POLL_INTERVAL},
};
use serde::Deserialize;
use serde_json::Value;

use crate::{
    hardware::{HardwareCancellation, HardwareError, HardwareErrorKind, HardwareOperations},
    host_control::HostControlShutdownHandle,
};

const LIQUIDCTL_PATH: &str = "/usr/bin/liquidctl";
const DISCOVERY_TTL: Duration = Duration::from_secs(5);
const COMMAND_TIMEOUT: Duration = LIQUIDCTL_COMMAND_TIMEOUT;
const PROCESS_POLL_INTERVAL: Duration = Duration::from_millis(10);
const STDOUT_LIMIT: usize = 1024 * 1024;
const STDERR_LIMIT: usize = 16 * 1024;
const MAX_DIAGNOSTIC_CHARS: usize = 512;
const CURVE_POINT_COUNT: usize = MONITORING_FIRMWARE_CURVE_POINTS;
const PROGRESSIVE_DUTIES: [u8; CURVE_POINT_COUNT] = [
    30, 30, 31, 31, 32, 33, 33, 34, 35, 36, 38, 40, 41, 43, 45, 47, 49, 51, 53, 55, 58, 60, 63, 65,
    68, 70, 73, 75, 78, 81, 84, 87, 90, 91, 92, 94, 95, 97, 98, 100,
];

/// Production hardware implementation backed by liquidctl's stable JSON CLI.
///
/// Construction has no side effects. Device discovery starts on the first snapshot.
pub struct LiquidctlHardware {
    runner: Box<dyn CommandRunner>,
    io_lock: Arc<Mutex<()>>,
    usb_lock_timeout: Duration,
    registry: BTreeMap<DeviceId, RegisteredDevice>,
    active_paths: BTreeMap<Location, Vec<DeviceId>>,
    last_discovery: Option<Instant>,
    discovery_ttl: Duration,
    sequence: u64,
    cancellation: HardwareCancellation,
}

impl LiquidctlHardware {
    /// A service manager and its LCD worker share this lock. It covers the
    /// entire liquidctl subprocess, never the 2-second wait or image render.
    pub(crate) fn with_io_lock(io_lock: Arc<Mutex<()>>) -> Self {
        let cancellation = HardwareCancellation::default();
        Self {
            runner: Box::new(ProcessRunner::new(
                PathBuf::from(LIQUIDCTL_PATH),
                COMMAND_TIMEOUT,
                cancellation.clone(),
            )),
            io_lock,
            usb_lock_timeout: USB_LOCK_ACQUIRE_TIMEOUT,
            registry: BTreeMap::new(),
            active_paths: BTreeMap::new(),
            last_discovery: None,
            discovery_ttl: DISCOVERY_TTL,
            sequence: 0,
            cancellation,
        }
    }

    fn discovery_due(&self) -> bool {
        self.last_discovery
            .is_none_or(|last| last.elapsed() >= self.discovery_ttl)
    }

    fn discover(&mut self) -> Result<(), HardwareError> {
        let output = self.run_checked(&["--json", "list"], "device discovery")?;
        let listed = parse_list(&output)?;
        self.reconcile(listed);
        self.last_discovery = Some(Instant::now());
        Ok(())
    }

    fn refresh_status(&mut self) -> Result<(), HardwareError> {
        if self.active_paths.is_empty() {
            mark_offline(self.registry.values_mut());
            return Ok(());
        }

        let output = self.run_checked(&["--json", "status"], "status refresh")?;
        let statuses = parse_status(&output)?;

        for registered in self.registry.values_mut() {
            registered.online = false;
            registered.readings.clear();
        }

        for status in statuses {
            let Some(device_ids) = self.active_paths.get(&status.location) else {
                // Discovery is deliberately cached. A just-attached device will be
                // picked up when the short discovery cache expires.
                continue;
            };
            for device_id in device_ids {
                if let Some(registered) = self.registry.get_mut(device_id) {
                    registered.online = true;
                    registered.readings = status.readings.clone();
                    if status.has_fan_telemetry && registered.metadata.driver == "KrakenZ3" {
                        ensure_channel(&mut registered.channels, "fan", 0);
                    }
                }
            }
        }

        for registered in self.registry.values_mut().filter(|device| !device.online) {
            for channel in &mut registered.channels {
                channel.curve_state = CurveState::Unverified;
            }
        }

        Ok(())
    }

    fn reconcile(&mut self, listed: Vec<ListedDevice>) {
        let mut serial_counts = HashMap::<String, usize>::new();
        for device in &listed {
            if let Some(serial) = &device.serial_number {
                *serial_counts.entry(serial.clone()).or_default() += 1;
            }
        }
        let listed_serials = serial_counts
            .keys()
            .cloned()
            .collect::<std::collections::BTreeSet<_>>();

        self.active_paths.clear();

        // Sorting makes reconciliation deterministic even when liquidctl changes
        // enumeration order between invocations.
        let mut listed = listed;
        listed.sort_by(|left, right| {
            stable_device_id(left)
                .cmp(&stable_device_id(right))
                .then_with(|| left.location().cmp(&right.location()))
        });

        let mut previous = std::mem::take(&mut self.registry);
        let mut reconciled = BTreeMap::new();
        for metadata in listed {
            let serial_unique = metadata
                .serial_number
                .as_ref()
                .is_some_and(|serial| serial_counts.get(serial) == Some(&1));
            let device_id = registry_device_id(&metadata, serial_unique);
            let location = metadata.location();

            let mut registered = previous
                .remove(&device_id)
                .or_else(|| take_serial_alias(&mut previous, &metadata, serial_unique))
                .unwrap_or_else(|| RegisteredDevice::new(device_id.clone(), metadata.clone()));
            if registered.id != device_id {
                registered.id = device_id.clone();
                for channel in &mut registered.channels {
                    channel.curve_state = CurveState::Unverified;
                }
            }
            registered.metadata = metadata;
            registered.serial_unique = serial_unique;
            reconciled.insert(device_id.clone(), registered);
            self.active_paths
                .entry(location)
                .or_default()
                .push(device_id);
        }

        // Keep true tombstones, but discard obsolete aliases when a serial is
        // currently present under its unique/duplicate identity form.
        for (device_id, mut registered) in previous {
            if registered
                .metadata
                .serial_number
                .as_ref()
                .is_some_and(|serial| listed_serials.contains(serial))
            {
                continue;
            }
            mark_offline(std::iter::once(&mut registered));
            reconciled.insert(device_id, registered);
        }
        self.registry = reconciled;

        for device_ids in self.active_paths.values_mut() {
            device_ids.sort();
            device_ids.dedup();
        }
    }

    fn run_serialized(&mut self, args: &[String]) -> Result<ProcessOutput, RunnerError> {
        // Hold the shared lock for the entire command/response exchange. A
        // second liquidctl process must not interleave with an LCD upload.
        let _guard = acquire_usb_lock(&self.io_lock, &self.cancellation, self.usb_lock_timeout)?;
        // Cancellation may have arrived on the successful acquisition poll.
        // Even fake runners use this production pre-spawn check.
        if self.cancellation.is_cancelled() {
            return Err(RunnerError::Cancelled { started: false });
        }
        self.runner.run(args)
    }

    fn run_checked(&mut self, args: &[&str], operation: &str) -> Result<Vec<u8>, HardwareError> {
        let args = args
            .iter()
            .map(|argument| (*argument).to_owned())
            .collect::<Vec<_>>();
        match self.run_serialized(&args) {
            Ok(output) if output.success && output.stdout.truncated => {
                Err(HardwareError::with_kind(
                    HardwareErrorKind::InvalidData,
                    format!("liquidctl {operation} output exceeded the {STDOUT_LIMIT}-byte limit"),
                ))
            }
            Ok(output) if output.success => Ok(output.stdout.bytes),
            Ok(output) => Err(nonzero_exit_error(operation, &output)),
            Err(error) => Err(runner_error(operation, error)),
        }
    }

    /// Require the uniquely addressable 240x240 Standard model, never Elite/Z/2024.
    pub(crate) fn display_serial(&mut self, device_id: &DeviceId) -> Result<String, HardwareError> {
        // Refresh the list even if a client recently obtained a snapshot: a
        // replaced/disconnected device must not inherit a stale serial claim.
        self.discover()?;
        if !self
            .active_paths
            .values()
            .any(|ids| ids.contains(device_id))
        {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                "Kraken 2023 is not connected; refresh the device list",
            ));
        }
        let device = self.registry.get(device_id).ok_or_else(|| {
            HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                "Kraken 2023 is not connected; refresh the device list",
            )
        })?;
        let model = &device.metadata;
        if !is_standard_2023(model) {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::Unsupported,
                "LCD presets require the NZXT Kraken 2023 Standard (240x240)",
            ));
        }
        if !device.serial_unique {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::Unsupported,
                "Kraken 2023 requires a unique serial for LCD writes",
            ));
        }
        model.serial_number.clone().ok_or_else(|| {
            HardwareError::with_kind(
                HardwareErrorKind::Unsupported,
                "Kraken 2023 has no serial number",
            )
        })
    }

    /// Eligible online identity for the service snapshot, including before any
    /// LCD mode is selected. A preferred selected identity must match exactly:
    /// never substitute a different cooler or a stale tombstone.
    pub(crate) fn display_candidate(&self, preferred: Option<&DeviceId>) -> Option<DeviceId> {
        let eligible = |id: &DeviceId| {
            self.registry.get(id).is_some_and(|device| {
                device.online
                    && device.serial_unique
                    && device.metadata.serial_number.is_some()
                    && is_standard_2023(&device.metadata)
                    && self.active_paths.values().any(|ids| ids.contains(id))
            })
        };
        match preferred {
            Some(id) => eligible(id).then(|| id.clone()),
            None => self.registry.keys().find(|id| eligible(id)).cloned(),
        }
    }

    pub(crate) fn set_display(
        &mut self,
        serial: &str,
        image: Option<&std::path::Path>,
    ) -> Result<(), HardwareError> {
        let mut args = vec![
            "--serial".into(),
            serial.into(),
            "set".into(),
            "lcd".into(),
            "screen".into(),
        ];
        match image {
            Some(path) => {
                args.push("static".into());
                args.push(path.to_string_lossy().into_owned());
            }
            None => args.push("liquid".into()),
        }
        match self.run_serialized(&args) {
            Ok(output) if output.success => Ok(()),
            Ok(output) => Err(nonzero_exit_error("LCD write", &output)),
            Err(error) => Err(runner_error("LCD write", error)),
        }
    }

    /// Fresh, uniquely serial-addressable channel preflight for service resume.
    /// This is deliberately separate from the write: no intent is blocked for
    /// an offline, ambiguous or invalid target.
    pub(crate) fn preflight_resume_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<bool, HardwareError> {
        self.discover()?;
        self.refresh_status()?;
        let Some(device) = self.registry.get(device_id) else {
            return Ok(false);
        };
        if !device.online {
            return Ok(false);
        }
        if !device.serial_unique
            || device.metadata.serial_number.is_none()
            || !self
                .active_paths
                .values()
                .any(|ids| ids.contains(device_id))
            || !matches!(device.metadata.driver.as_str(), "KrakenZ3" | "KrakenX3")
        {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::Unsupported,
                "saved cooler is not uniquely serial-addressable and writable",
            ));
        }
        let channel = device
            .channels
            .iter()
            .find(|c| &c.id == channel_id)
            .ok_or_else(|| {
                HardwareError::with_kind(HardwareErrorKind::Unsupported, "saved channel is absent")
            })?;
        validate_curve(points, channel.min_duty)?;
        Ok(true)
    }

    fn cached_snapshot(&self) -> HardwareSnapshot {
        HardwareSnapshot {
            devices: self
                .registry
                .values()
                .map(RegisteredDevice::as_device)
                .collect(),
            sequence: self.sequence,
            host_control: Default::default(),
            kraken_display: Default::default(),
            monitoring: Default::default(),
        }
    }

    /// A failed USB refresh must not suppress independent host-control state.
    /// Keep device identity/edits, but never advertise stale readings or writes.
    pub(crate) fn offline_snapshot(&mut self) -> HardwareSnapshot {
        mark_offline(self.registry.values_mut());
        self.sequence = self.sequence.wrapping_add(1);
        self.cached_snapshot()
    }

    #[cfg(test)]
    pub(crate) fn unavailable_for_test() -> Self {
        struct Missing;
        impl CommandRunner for Missing {
            fn run(&mut self, _args: &[String]) -> Result<ProcessOutput, RunnerError> {
                Err(RunnerError::Io {
                    kind: io::ErrorKind::NotFound,
                    message: "simulated missing liquidctl".into(),
                    started: false,
                })
            }
        }
        Self::with_runner(Missing)
    }

    #[cfg(test)]
    fn with_runner(runner: impl CommandRunner + 'static) -> Self {
        Self::with_runner_and_lock(runner, Arc::new(Mutex::new(())))
    }

    #[cfg(test)]
    fn with_runner_and_lock(runner: impl CommandRunner + 'static, io_lock: Arc<Mutex<()>>) -> Self {
        Self {
            runner: Box::new(runner),
            io_lock,
            usb_lock_timeout: USB_LOCK_ACQUIRE_TIMEOUT,
            registry: BTreeMap::new(),
            active_paths: BTreeMap::new(),
            last_discovery: None,
            discovery_ttl: Duration::ZERO,
            sequence: 0,
            cancellation: HardwareCancellation::default(),
        }
    }

    #[cfg(test)]
    fn set_discovery_ttl(&mut self, ttl: Duration) {
        self.discovery_ttl = ttl;
    }

    #[cfg(test)]
    fn force_discovery(&mut self) {
        self.last_discovery = None;
    }
}

impl HardwareOperations for LiquidctlHardware {
    fn cancellation_handle(&self) -> HardwareCancellation {
        self.cancellation.clone()
    }

    fn snapshot(&mut self) -> Result<HardwareSnapshot, HardwareError> {
        if self.discovery_due() {
            self.discover()?;
        }
        self.refresh_status()?;
        self.sequence = self.sequence.wrapping_add(1);
        Ok(self.cached_snapshot())
    }

    fn apply_firmware_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError> {
        let (serial, channel_name, minimum_duty) = {
            let registered = self.registry.get(device_id).ok_or_else(|| {
                HardwareError::with_kind(
                    HardwareErrorKind::Unavailable,
                    format!("liquidctl device {device_id} is not in the device registry; refresh before applying a curve"),
                )
            })?;

            if !registered.online {
                return Err(HardwareError::with_kind(
                    HardwareErrorKind::Unavailable,
                    format!(
                        "{} is offline; reconnect it and refresh before applying a curve",
                        registered.display_name()
                    ),
                ));
            }
            let serial = registered.metadata.serial_number.as_ref().ok_or_else(|| {
                HardwareError::with_kind(
                    HardwareErrorKind::Unsupported,
                    format!(
                        "{} has no serial number, so it cannot be selected safely for a write",
                        registered.display_name()
                    ),
                )
            })?;
            if !registered.serial_unique {
                return Err(HardwareError::with_kind(
                    HardwareErrorKind::Unsupported,
                    format!(
                        "serial number {serial:?} is reported by more than one device; refusing an ambiguous write"
                    ),
                ));
            }
            if !matches!(registered.metadata.driver.as_str(), "KrakenZ3" | "KrakenX3") {
                return Err(HardwareError::with_kind(
                    HardwareErrorKind::Unsupported,
                    format!(
                        "liquidctl driver {} is telemetry-only in this application",
                        printable_driver(&registered.metadata.driver)
                    ),
                ));
            }

            let channel = registered
                .channels
                .iter()
                .find(|channel| &channel.id == channel_id)
                .ok_or_else(|| {
                    HardwareError::with_kind(
                        HardwareErrorKind::Unsupported,
                        format!(
                            "channel {channel_id} is not writable for liquidctl driver {}",
                            registered.metadata.driver
                        ),
                    )
                })?;
            (serial.clone(), channel.id.0.clone(), channel.min_duty)
        };

        validate_curve(points, minimum_duty)?;

        let mut args = Vec::with_capacity(5 + CURVE_POINT_COUNT * 2);
        args.extend([
            "--serial".to_owned(),
            serial,
            "set".to_owned(),
            channel_name,
            "speed".to_owned(),
        ]);
        for point in points {
            args.push(point.temperature.to_string());
            args.push(point.duty.to_string());
        }

        match self.run_serialized(&args) {
            Ok(output) if output.success => {
                let channel = self
                    .registry
                    .get_mut(device_id)
                    .and_then(|registered| {
                        registered
                            .channels
                            .iter_mut()
                            .find(|channel| &channel.id == channel_id)
                    })
                    .expect("validated registry channel must remain present");
                channel.points = points.to_vec();
                channel.curve_state = CurveState::Applied;
                Ok(())
            }
            Ok(output) => {
                self.mark_channel_unverified(device_id, channel_id);
                let error = nonzero_exit_error("curve write", &output);
                Err(HardwareError::with_kind(
                    HardwareErrorKind::UnknownOutcome,
                    format!("{error} The command started, so the device's curve state is unknown."),
                ))
            }
            Err(error) => {
                if error.write_outcome_unknown() {
                    self.mark_channel_unverified(device_id, channel_id);
                }
                Err(runner_error("curve write", error))
            }
        }
    }

    fn start_host_control(&mut self, _policy: &HostControlPolicy) -> Result<(), HardwareError> {
        Err(unsupported_host_control())
    }

    fn update_host_control(
        &mut self,
        _channel_policies: &[HostChannelPolicy],
    ) -> Result<(), HardwareError> {
        Err(unsupported_host_control())
    }

    fn stop_host_control(&mut self) -> Result<(), HardwareError> {
        Err(unsupported_host_control())
    }

    fn host_control_shutdown_handle(&self) -> HostControlShutdownHandle {
        HostControlShutdownHandle::disabled()
    }

    fn client_disconnected(&mut self) {
        self.cancellation.cancel();
        self.invalidate();
    }
}

fn unsupported_host_control() -> HardwareError {
    HardwareError::with_kind(
        HardwareErrorKind::Unsupported,
        "host control is unsupported by the liquidctl-only hardware adapter",
    )
}

impl LiquidctlHardware {
    pub(crate) fn invalidate(&mut self) {
        for registered in self.registry.values_mut() {
            for channel in &mut registered.channels {
                channel.curve_state = CurveState::Unverified;
            }
        }
    }

    fn mark_channel_unverified(&mut self, device_id: &DeviceId, channel_id: &ChannelId) {
        if let Some(channel) = self.registry.get_mut(device_id).and_then(|registered| {
            registered
                .channels
                .iter_mut()
                .find(|channel| &channel.id == channel_id)
        }) {
            channel.curve_state = CurveState::Unverified;
        }
    }
}

fn take_serial_alias(
    previous: &mut BTreeMap<DeviceId, RegisteredDevice>,
    metadata: &ListedDevice,
    serial_unique: bool,
) -> Option<RegisteredDevice> {
    let serial = metadata.serial_number.as_ref()?;
    let location = metadata.location();
    let alias = previous
        .iter()
        .find(|(_, device)| {
            device.metadata.serial_number.as_ref() == Some(serial)
                && same_device_family(&device.metadata, metadata)
                && device.metadata.location() == location
        })
        .map(|(device_id, _)| device_id.clone())
        .or_else(|| {
            if serial_unique {
                previous
                    .iter()
                    .find(|(_, device)| {
                        device.metadata.serial_number.as_ref() == Some(serial)
                            && same_device_family(&device.metadata, metadata)
                    })
                    .map(|(device_id, _)| device_id.clone())
            } else {
                None
            }
        });
    alias.and_then(|device_id| previous.remove(&device_id))
}

fn same_device_family(left: &ListedDevice, right: &ListedDevice) -> bool {
    left.driver == right.driver
        && left.vendor_id == right.vendor_id
        && left.product_id == right.product_id
}

fn mark_offline<'a>(devices: impl IntoIterator<Item = &'a mut RegisteredDevice>) {
    for device in devices {
        device.online = false;
        device.readings.clear();
        for channel in &mut device.channels {
            channel.curve_state = CurveState::Unverified;
        }
    }
}

fn validate_curve(points: &[CurvePoint], minimum_duty: u8) -> Result<(), HardwareError> {
    if points.len() != CURVE_POINT_COUNT {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::InvalidData,
            format!(
                "liquidctl firmware curves require exactly {CURVE_POINT_COUNT} points for 20°C through 59°C; received {}",
                points.len()
            ),
        ));
    }

    for (offset, point) in points.iter().enumerate() {
        let expected_temperature = 20 + offset as u8;
        if point.temperature != expected_temperature {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::InvalidData,
                format!(
                    "curve point {} must be {expected_temperature}°C, not {}°C; temperatures must be ordered 20 through 59",
                    offset + 1,
                    point.temperature
                ),
            ));
        }
        if point.duty < minimum_duty || point.duty > 100 {
            return Err(HardwareError::with_kind(
                HardwareErrorKind::InvalidData,
                format!(
                    "curve duty at {}°C is {}%; channel range is {minimum_duty}% through 100%",
                    point.temperature, point.duty
                ),
            ));
        }
    }

    if let Some(window) = points.windows(2).find(|pair| pair[1].duty < pair[0].duty) {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::InvalidData,
            format!(
                "curve duties must be nondecreasing, but {}°C is {}% after {}°C at {}%",
                window[1].temperature, window[1].duty, window[0].temperature, window[0].duty
            ),
        ));
    }
    if points.last().is_none_or(|point| point.duty != 100) {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::InvalidData,
            "the 59°C failsafe point must be 100% so liquidctl writes the displayed curve unchanged",
        ));
    }

    Ok(())
}

#[derive(Clone, Debug)]
struct RegisteredDevice {
    id: DeviceId,
    metadata: ListedDevice,
    online: bool,
    readings: Vec<Reading>,
    channels: Vec<CoolingChannel>,
    serial_unique: bool,
}

impl RegisteredDevice {
    fn new(id: DeviceId, metadata: ListedDevice) -> Self {
        let mut channels = Vec::new();
        match metadata.driver.as_str() {
            "KrakenZ3" | "KrakenX3" => ensure_channel(&mut channels, "pump", 20),
            _ => {}
        }
        Self {
            id,
            metadata,
            online: false,
            readings: Vec::new(),
            channels,
            serial_unique: false,
        }
    }

    fn display_name(&self) -> &str {
        if self.metadata.description.is_empty() {
            printable_driver(&self.metadata.driver)
        } else {
            &self.metadata.description
        }
    }

    fn as_device(&self) -> Device {
        Device {
            id: self.id.clone(),
            name: self.display_name().to_owned(),
            model: device_model(&self.metadata),
            kind: classify_device(&self.metadata),
            online: self.online,
            readings: self.readings.clone(),
            cooling_channels: if self.metadata.serial_number.is_some() && self.serial_unique {
                self.channels.clone()
            } else {
                Vec::new()
            },
        }
    }
}

fn ensure_channel(channels: &mut Vec<CoolingChannel>, id: &str, minimum_duty: u8) {
    if channels.iter().any(|channel| channel.id.0 == id) {
        return;
    }
    let name = if id == "pump" { "Pump" } else { "Radiator fan" };
    channels.push(CoolingChannel::unverified_firmware_curve(
        id,
        name,
        TemperatureSource::Liquid,
        minimum_duty,
        progressive_duties(minimum_duty),
    ));
}

fn progressive_duties(minimum_duty: u8) -> Vec<u8> {
    PROGRESSIVE_DUTIES
        .into_iter()
        .map(|duty| duty.max(minimum_duty))
        .collect()
}

fn is_standard_2023(device: &ListedDevice) -> bool {
    device.driver == "KrakenZ3"
        && device.description == "NZXT Kraken 2023"
        && device.vendor_id == Some(0x1e71)
        && device.product_id == Some(0x300e)
}

fn classify_device(device: &ListedDevice) -> DeviceKind {
    let searchable = format!("{} {}", device.description, device.driver).to_ascii_lowercase();
    if matches!(device.driver.as_str(), "KrakenZ3" | "KrakenX3")
        || ["kraken", "cooler", "hydro", "pump"]
            .iter()
            .any(|word| searchable.contains(word))
    {
        DeviceKind::LiquidCooler
    } else if ["fan", "smartdevice", "commander"]
        .iter()
        .any(|word| searchable.contains(word))
    {
        DeviceKind::FanController
    } else {
        DeviceKind::LightingController
    }
}

fn device_model(device: &ListedDevice) -> String {
    let mut details = Vec::new();
    match (device.vendor_id, device.product_id) {
        (Some(vendor), Some(product)) => details.push(format!("USB {vendor:04x}:{product:04x}")),
        (Some(vendor), None) => details.push(format!("USB {vendor:04x}:????")),
        (None, Some(product)) => details.push(format!("USB ????:{product:04x}")),
        (None, None) => {}
    }
    if let Some(release) = device.release_number {
        details.push(format!("rev {release:04x}"));
    }
    if !device.driver.is_empty() {
        details.push(device.driver.clone());
    }
    if !device.port.is_empty() {
        details.push(format!("port {}", device.port.join(".")));
    }
    if device.experimental {
        details.push("experimental".into());
    }
    if details.is_empty() {
        "liquidctl device".into()
    } else {
        details.join(" · ")
    }
}

fn printable_driver(driver: &str) -> &str {
    if driver.is_empty() {
        "unknown driver"
    } else {
        driver
    }
}

#[derive(Clone, Debug)]
struct ListedDevice {
    description: String,
    vendor_id: Option<u64>,
    product_id: Option<u64>,
    release_number: Option<u64>,
    serial_number: Option<String>,
    bus: String,
    address: String,
    port: Vec<String>,
    driver: String,
    experimental: bool,
}

impl ListedDevice {
    fn location(&self) -> Location {
        Location {
            bus: self.bus.clone(),
            address: self.address.clone(),
        }
    }

    fn path_identity(&self) -> String {
        format!("{}\0{}\0{}", self.bus, self.address, self.port.join("."))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Location {
    bus: String,
    address: String,
}

#[derive(Deserialize)]
struct RawListedDevice {
    #[serde(default)]
    description: Value,
    #[serde(default)]
    vendor_id: Value,
    #[serde(default)]
    product_id: Value,
    #[serde(default)]
    release_number: Value,
    #[serde(default)]
    serial_number: Value,
    #[serde(default)]
    bus: Value,
    #[serde(default)]
    address: Value,
    #[serde(default)]
    port: Value,
    #[serde(default)]
    driver: Value,
    #[serde(default)]
    experimental: Value,
}

fn parse_list(bytes: &[u8]) -> Result<Vec<ListedDevice>, HardwareError> {
    let raw: Vec<RawListedDevice> =
        serde_json::from_slice(bytes).map_err(|error| invalid_json_error("device list", error))?;
    raw.into_iter()
        .enumerate()
        .map(|(index, raw)| listed_device_from_raw(raw, index))
        .collect()
}

fn listed_device_from_raw(
    raw: RawListedDevice,
    index: usize,
) -> Result<ListedDevice, HardwareError> {
    let context = format!("liquidctl list record {}", index + 1);
    Ok(ListedDevice {
        description: nullable_string(&raw.description, "description", &context)?,
        vendor_id: nullable_unsigned(&raw.vendor_id, "vendor_id", &context)?,
        product_id: nullable_unsigned(&raw.product_id, "product_id", &context)?,
        release_number: nullable_unsigned(&raw.release_number, "release_number", &context)?,
        serial_number: nonempty_nullable_string(&raw.serial_number, "serial_number", &context)?,
        bus: scalar_path_part(&raw.bus, "bus", &context)?,
        address: scalar_path_part(&raw.address, "address", &context)?,
        port: parse_port(&raw.port, &context)?,
        driver: nullable_string(&raw.driver, "driver", &context)?,
        experimental: nullable_bool(&raw.experimental, "experimental", &context)?,
    })
}

fn nullable_string(value: &Value, field: &str, context: &str) -> Result<String, HardwareError> {
    match value {
        Value::Null => Ok(String::new()),
        Value::String(value) => Ok(value.trim().to_owned()),
        _ => Err(schema_error(format!(
            "{context} has a non-string {field} field"
        ))),
    }
}

fn nonempty_nullable_string(
    value: &Value,
    field: &str,
    context: &str,
) -> Result<Option<String>, HardwareError> {
    Ok(match nullable_string(value, field, context)? {
        value if value.is_empty() => None,
        value => Some(value),
    })
}

fn nullable_unsigned(
    value: &Value,
    field: &str,
    context: &str,
) -> Result<Option<u64>, HardwareError> {
    match value {
        Value::Null => Ok(None),
        Value::Number(number) => number.as_u64().map(Some).ok_or_else(|| {
            schema_error(format!("{context} has an invalid unsigned {field} field"))
        }),
        Value::String(text) => {
            let text = text.trim();
            if text.is_empty() {
                return Ok(None);
            }
            let parsed = text
                .strip_prefix("0x")
                .or_else(|| text.strip_prefix("0X"))
                .map_or_else(|| text.parse::<u64>(), |hex| u64::from_str_radix(hex, 16))
                .map_err(|_| schema_error(format!("{context} has an invalid {field} field")))?;
            Ok(Some(parsed))
        }
        _ => Err(schema_error(format!(
            "{context} has a non-numeric {field} field"
        ))),
    }
}

fn scalar_path_part(value: &Value, field: &str, context: &str) -> Result<String, HardwareError> {
    match value {
        Value::Null => Ok(String::new()),
        Value::String(value) => Ok(value.trim().to_owned()),
        Value::Number(value) => Ok(value.to_string()),
        _ => Err(schema_error(format!(
            "{context} has a non-scalar {field} field"
        ))),
    }
}

fn parse_port(value: &Value, context: &str) -> Result<Vec<String>, HardwareError> {
    match value {
        Value::Null => Ok(Vec::new()),
        Value::Array(parts) => parts
            .iter()
            .map(|part| scalar_path_part(part, "port", context))
            .collect(),
        Value::String(_) | Value::Number(_) => Ok(vec![scalar_path_part(value, "port", context)?]),
        _ => Err(schema_error(format!(
            "{context} has a non-scalar port field"
        ))),
    }
}

fn nullable_bool(value: &Value, field: &str, context: &str) -> Result<bool, HardwareError> {
    match value {
        Value::Null => Ok(false),
        Value::Bool(value) => Ok(*value),
        _ => Err(schema_error(format!(
            "{context} has a non-boolean {field} field"
        ))),
    }
}

fn stable_device_id(device: &ListedDevice) -> DeviceId {
    let driver: String = if device.driver.is_empty() {
        "unknown".into()
    } else {
        device
            .driver
            .chars()
            .flat_map(char::to_lowercase)
            .map(|character| {
                if character.is_ascii_alphanumeric() || character == '_' {
                    character
                } else {
                    '-'
                }
            })
            .collect()
    };
    let vendor = device
        .vendor_id
        .map_or_else(|| "none".into(), |value| format!("{value:04x}"));
    let product = device
        .product_id
        .map_or_else(|| "none".into(), |value| format!("{value:04x}"));
    let identity = device.serial_number.as_ref().map_or_else(
        || format!("path-{}", hex_bytes(device.path_identity().as_bytes())),
        |serial| format!("serial-{}", hex_bytes(serial.as_bytes())),
    );
    DeviceId::new(format!("liquidctl-{driver}-{vendor}-{product}-{identity}"))
}

fn registry_device_id(device: &ListedDevice, serial_unique: bool) -> DeviceId {
    let base = stable_device_id(device);
    if device.serial_number.is_some() && !serial_unique {
        DeviceId::new(format!(
            "{}-duplicate-path-{}",
            base.0,
            hex_bytes(device.path_identity().as_bytes())
        ))
    } else {
        base
    }
}

fn hex_bytes(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut encoded = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        encoded.push(HEX[usize::from(byte >> 4)] as char);
        encoded.push(HEX[usize::from(byte & 0x0f)] as char);
    }
    encoded
}

#[derive(Deserialize)]
struct RawStatusDevice {
    #[serde(default)]
    bus: Value,
    #[serde(default)]
    address: Value,
    #[serde(default)]
    description: Value,
    #[serde(default)]
    status: Value,
}

struct ParsedStatus {
    location: Location,
    readings: Vec<Reading>,
    has_fan_telemetry: bool,
}

fn parse_status(bytes: &[u8]) -> Result<Vec<ParsedStatus>, HardwareError> {
    let raw: Vec<RawStatusDevice> =
        serde_json::from_slice(bytes).map_err(|error| invalid_json_error("status", error))?;
    raw.into_iter()
        .enumerate()
        .map(|(index, raw)| status_from_raw(raw, index))
        .collect()
}

fn status_from_raw(raw: RawStatusDevice, index: usize) -> Result<ParsedStatus, HardwareError> {
    let context = format!("liquidctl status record {}", index + 1);
    // Validate this field even though the list description remains authoritative.
    let _description = nullable_string(&raw.description, "description", &context)?;
    let location = Location {
        bus: scalar_path_part(&raw.bus, "bus", &context)?,
        address: scalar_path_part(&raw.address, "address", &context)?,
    };
    let entries = match raw.status {
        Value::Null => Vec::new(),
        Value::Array(entries) => entries,
        _ => {
            return Err(schema_error(format!(
                "{context} has a non-array status field"
            )));
        }
    };

    let mut readings = Vec::new();
    let mut has_fan_telemetry = false;
    for (entry_index, entry) in entries.into_iter().enumerate() {
        let Value::Object(mut entry) = entry else {
            return Err(schema_error(format!(
                "{context} entry {} is not an object",
                entry_index + 1
            )));
        };
        let key = entry.remove("key").unwrap_or(Value::Null);
        let value = entry.remove("value").unwrap_or(Value::Null);
        let unit = entry.remove("unit").unwrap_or(Value::Null);
        let key = nullable_string(&key, "key", &context)?;
        let unit = nullable_string(&unit, "unit", &context)?;
        if key.is_empty() {
            continue;
        }

        let lower_key = key.to_ascii_lowercase();
        if lower_key.starts_with("fan ") || lower_key == "fan" {
            has_fan_telemetry = true;
        }

        if let Some(channel_count) = argb_channel_count(&key, &value, &unit) {
            readings.push(Reading::new(
                "ARGB Channels",
                f64::from(channel_count),
                "",
                ReadingKind::ChannelCount,
            ));
            continue;
        }

        let Some(number) = value.as_f64() else {
            // Firmware versions and mode names are intentionally not represented
            // by the numeric Reading model.
            continue;
        };
        let Some(kind) = reading_kind(&key, &unit) else {
            continue;
        };
        readings.push(Reading::new(key, number, unit, kind));
    }

    Ok(ParsedStatus {
        location,
        readings,
        has_fan_telemetry,
    })
}

fn argb_channel_count(key: &str, value: &Value, unit: &str) -> Option<u16> {
    let value_is_empty = value.is_null() || value.as_str().is_some_and(str::is_empty);
    if !value_is_empty || !unit.is_empty() {
        return None;
    }
    let (label, count) = key.rsplit_once(':')?;
    if !label.trim().eq_ignore_ascii_case("ARGB Channels") {
        return None;
    }
    count.trim().parse().ok()
}

fn reading_kind(key: &str, unit: &str) -> Option<ReadingKind> {
    let key = key.to_ascii_lowercase();
    let unit = unit.to_ascii_lowercase();
    if unit == "°c" || unit == "c" || key.contains("temperature") {
        Some(ReadingKind::Temperature)
    } else if unit == "rpm" || key.contains("speed") {
        Some(ReadingKind::Speed)
    } else if unit == "%" || key.contains("duty") {
        Some(ReadingKind::Duty)
    } else if key.contains("channels") || key.contains("channel count") {
        Some(ReadingKind::ChannelCount)
    } else {
        None
    }
}

fn schema_error(message: impl Into<String>) -> HardwareError {
    HardwareError::with_kind(HardwareErrorKind::InvalidData, message)
}

fn invalid_json_error(section: &str, error: serde_json::Error) -> HardwareError {
    HardwareError::with_kind(
        HardwareErrorKind::InvalidData,
        format!(
            "liquidctl returned malformed {section} JSON at line {}, column {}: {error}; verify liquidctl 1.16 or newer is installed",
            error.line(),
            error.column()
        ),
    )
}

/// Wait only for this command's lock budget, polling so cancellation is not
/// trapped behind a busy LCD upload. Poison recovery matches the old mutex
/// handling: retain serialization by taking the poisoned guard, not bypassing
/// the lock. A failure here is always before the runner/subprocess starts.
fn acquire_usb_lock<'a>(
    lock: &'a Mutex<()>,
    cancellation: &HardwareCancellation,
    budget: Duration,
) -> Result<MutexGuard<'a, ()>, RunnerError> {
    let started = Instant::now();
    loop {
        if cancellation.is_cancelled() {
            return Err(RunnerError::Cancelled { started: false });
        }
        if started.elapsed() >= budget {
            return Err(RunnerError::LockTimeout { budget });
        }
        match lock.try_lock() {
            Ok(guard) => return Ok(guard),
            Err(TryLockError::Poisoned(poisoned)) => return Ok(poisoned.into_inner()),
            Err(TryLockError::WouldBlock) => {
                thread::sleep(USB_LOCK_POLL_INTERVAL.min(budget.saturating_sub(started.elapsed())));
            }
        }
    }
}

trait CommandRunner: Send {
    fn run(&mut self, args: &[String]) -> Result<ProcessOutput, RunnerError>;
}

struct ProcessRunner {
    executable: PathBuf,
    timeout: Duration,
    cancellation: HardwareCancellation,
}

impl ProcessRunner {
    fn new(executable: PathBuf, timeout: Duration, cancellation: HardwareCancellation) -> Self {
        Self {
            executable,
            timeout,
            cancellation,
        }
    }
}

impl CommandRunner for ProcessRunner {
    fn run(&mut self, args: &[String]) -> Result<ProcessOutput, RunnerError> {
        if self.cancellation.is_cancelled() {
            return Err(RunnerError::Cancelled { started: false });
        }

        let mut command = Command::new(&self.executable);
        command
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(unix)]
        command.process_group(0);
        // The subprocess budget starts at spawn, not after output setup, and
        // never includes the independently bounded USB mutex acquisition.
        let started = Instant::now();
        let mut child = command.spawn().map_err(|error| RunnerError::Io {
            kind: error.kind(),
            message: format!("failed to start {}: {error}", self.executable.display()),
            started: false,
        })?;

        let stdout = match child.stdout.take() {
            Some(stdout) => stdout,
            None => {
                terminate_and_reap(&mut child);
                return Err(RunnerError::Io {
                    kind: io::ErrorKind::Other,
                    message: "failed to capture liquidctl standard output".into(),
                    started: true,
                });
            }
        };
        let stderr = match child.stderr.take() {
            Some(stderr) => stderr,
            None => {
                terminate_and_reap(&mut child);
                return Err(RunnerError::Io {
                    kind: io::ErrorKind::Other,
                    message: "failed to capture liquidctl standard error".into(),
                    started: true,
                });
            }
        };

        let stdout_reader = match spawn_reader(stdout, "liquidctl-stdout", STDOUT_LIMIT) {
            Ok(reader) => reader,
            Err(error) => {
                terminate_and_reap(&mut child);
                return Err(RunnerError::Io {
                    kind: error.kind(),
                    message: format!("failed to start liquidctl output reader: {error}"),
                    started: true,
                });
            }
        };
        let stderr_reader = match spawn_reader(stderr, "liquidctl-stderr", STDERR_LIMIT) {
            Ok(reader) => reader,
            Err(error) => {
                terminate_and_reap(&mut child);
                let _ = join_reader(stdout_reader);
                return Err(RunnerError::Io {
                    kind: error.kind(),
                    message: format!("failed to start liquidctl error reader: {error}"),
                    started: true,
                });
            }
        };

        enum Completion {
            Finished(std::process::ExitStatus),
            TimedOut,
            Cancelled,
        }

        let completion = loop {
            if self.cancellation.is_cancelled() {
                terminate_and_reap(&mut child);
                break Completion::Cancelled;
            }
            match child.try_wait() {
                Ok(Some(status)) => {
                    kill_process_group(child.id());
                    break Completion::Finished(status);
                }
                Ok(None) if started.elapsed() >= self.timeout => {
                    terminate_and_reap(&mut child);
                    break Completion::TimedOut;
                }
                Ok(None) => {
                    let remaining = self.timeout.saturating_sub(started.elapsed());
                    thread::sleep(PROCESS_POLL_INTERVAL.min(remaining));
                }
                Err(error) => {
                    terminate_and_reap(&mut child);
                    let _ = join_reader(stdout_reader);
                    let _ = join_reader(stderr_reader);
                    return Err(RunnerError::Io {
                        kind: error.kind(),
                        message: format!("failed while waiting for liquidctl: {error}"),
                        started: true,
                    });
                }
            }
        };

        let stdout = join_reader(stdout_reader);
        let stderr = join_reader(stderr_reader);
        match completion {
            Completion::TimedOut => {
                let _ = stdout;
                let _ = stderr;
                Err(RunnerError::Timeout)
            }
            Completion::Cancelled => {
                let _ = stdout;
                let _ = stderr;
                Err(RunnerError::Cancelled { started: true })
            }
            Completion::Finished(status) => Ok(ProcessOutput {
                success: status.success(),
                code: status.code(),
                stdout: stdout?,
                stderr: stderr?,
            }),
        }
    }
}

fn spawn_reader<R>(
    mut input: R,
    name: &str,
    retained_limit: usize,
) -> io::Result<thread::JoinHandle<io::Result<CapturedOutput>>>
where
    R: Read + Send + 'static,
{
    thread::Builder::new().name(name.into()).spawn(move || {
        let mut bytes = Vec::with_capacity(retained_limit.min(8192));
        let mut buffer = [0_u8; 8192];
        let mut truncated = false;
        loop {
            let read = input.read(&mut buffer)?;
            if read == 0 {
                break;
            }
            let remaining = retained_limit.saturating_sub(bytes.len());
            let retained = read.min(remaining);
            bytes.extend_from_slice(&buffer[..retained]);
            truncated |= retained < read;
        }
        Ok(CapturedOutput { bytes, truncated })
    })
}

fn join_reader(
    reader: thread::JoinHandle<io::Result<CapturedOutput>>,
) -> Result<CapturedOutput, RunnerError> {
    match reader.join() {
        Ok(Ok(output)) => Ok(output),
        Ok(Err(error)) => Err(RunnerError::Io {
            kind: error.kind(),
            message: format!("failed to read liquidctl output: {error}"),
            started: true,
        }),
        Err(_) => Err(RunnerError::Io {
            kind: io::ErrorKind::Other,
            message: "liquidctl output reader stopped unexpectedly".into(),
            started: true,
        }),
    }
}

fn terminate_and_reap(child: &mut std::process::Child) {
    kill_process_group(child.id());
    let _ = child.kill();
    let _ = child.wait();
}

#[cfg(unix)]
fn kill_process_group(process_id: u32) {
    if let Ok(process_group) = i32::try_from(process_id) {
        // SAFETY: the child starts in a dedicated process group whose ID equals
        // its PID. A negative PID asks kill(2) to signal that entire group.
        unsafe {
            libc::kill(-process_group, libc::SIGKILL);
        }
    }
}

#[cfg(not(unix))]
fn kill_process_group(_process_id: u32) {}

#[derive(Debug)]
struct CapturedOutput {
    bytes: Vec<u8>,
    truncated: bool,
}

#[derive(Debug)]
struct ProcessOutput {
    success: bool,
    code: Option<i32>,
    stdout: CapturedOutput,
    stderr: CapturedOutput,
}

#[derive(Debug)]
enum RunnerError {
    Io {
        kind: io::ErrorKind,
        message: String,
        started: bool,
    },
    /// The child started and its process group was terminated and reaped.
    Timeout,
    /// No runner call or subprocess spawn occurred.
    LockTimeout {
        budget: Duration,
    },
    Cancelled {
        started: bool,
    },
}

impl RunnerError {
    fn write_outcome_unknown(&self) -> bool {
        match self {
            Self::Io { started, .. } | Self::Cancelled { started } => *started,
            Self::Timeout => true,
            Self::LockTimeout { .. } => false,
        }
    }
}

fn runner_error(operation: &str, error: RunnerError) -> HardwareError {
    match error {
        RunnerError::Timeout => timeout_error(operation),
        // Timeout at the protocol boundary means a write may have started.
        // Report lock exhaustion as definite unavailability instead: the USB
        // command did not start, so there is no unknown hardware write outcome.
        RunnerError::LockTimeout { budget } => HardwareError::with_kind(
            HardwareErrorKind::Unavailable,
            format!(
                "liquidctl {operation} timed out waiting for the shared USB lock after {} ms; command did not start (no subprocess spawned)",
                budget.as_millis()
            ),
        ),
        RunnerError::Cancelled { started } => HardwareError::with_kind(
            if started {
                HardwareErrorKind::UnknownOutcome
            } else {
                HardwareErrorKind::Unavailable
            },
            if started {
                format!(
                    "liquidctl {operation} was cancelled and its process group was terminated; device state is unknown"
                )
            } else {
                format!("liquidctl {operation} was cancelled before it started")
            },
        ),
        RunnerError::Io {
            kind: _,
            message,
            started,
        } if started && operation == "curve write" => HardwareError::with_kind(
            HardwareErrorKind::UnknownOutcome,
            format!(
                "liquidctl curve write failed after the command started: {message}; device state is unknown"
            ),
        ),
        RunnerError::Io {
            kind,
            message,
            started: _,
        } => match kind {
            io::ErrorKind::NotFound => HardwareError::with_kind(
                HardwareErrorKind::Unavailable,
                format!(
                    "liquidctl is unavailable during {operation}: {message}. Install liquidctl 1.16 or newer at {LIQUIDCTL_PATH}"
                ),
            ),
            io::ErrorKind::PermissionDenied => HardwareError::with_kind(
                HardwareErrorKind::PermissionDenied,
                format!(
                    "permission denied while starting liquidctl for {operation}: {message}. Check executable permissions"
                ),
            ),
            _ => HardwareError::with_kind(
                HardwareErrorKind::Internal,
                format!("could not run liquidctl for {operation}: {message}"),
            ),
        },
    }
}

fn timeout_error(operation: &str) -> HardwareError {
    let outcome = if operation == "curve write" {
        "; device state is unknown"
    } else {
        ""
    };
    HardwareError::with_kind(
        HardwareErrorKind::Timeout,
        format!(
            "liquidctl {operation} exceeded the {} second timeout; its process group was terminated and reaped{outcome}",
            COMMAND_TIMEOUT.as_secs()
        ),
    )
}

fn nonzero_exit_error(operation: &str, output: &ProcessOutput) -> HardwareError {
    let diagnostic = truncated_diagnostic(&output.stderr.bytes, output.stderr.truncated);
    let lower = diagnostic.to_ascii_lowercase();
    let (kind, guidance) = if [
        "permission denied",
        "insufficient permissions",
        "access denied",
        "operation not permitted",
        "open failed",
    ]
    .iter()
    .any(|needle| lower.contains(needle))
    {
        (
            HardwareErrorKind::PermissionDenied,
            "Install the liquidctl udev rules and grant this user access to the device.",
        )
    } else if ["not supported", "unsupported", "not implemented"]
        .iter()
        .any(|needle| lower.contains(needle))
    {
        (
            HardwareErrorKind::Unsupported,
            "Update liquidctl or verify that this device and channel support the requested operation.",
        )
    } else if [
        "no device matches",
        "no devices match",
        "device not found",
        "device disconnected",
        "could not open",
        "not available",
    ]
    .iter()
    .any(|needle| lower.contains(needle))
    {
        (
            HardwareErrorKind::Unavailable,
            "Reconnect the device and verify it appears in `liquidctl list`.",
        )
    } else {
        (
            HardwareErrorKind::Internal,
            "Run liquidctl manually to inspect the complete diagnostic.",
        )
    };
    let exit = output.code.map_or_else(
        || "terminated by signal".into(),
        |code| format!("exit code {code}"),
    );
    let diagnostic = if diagnostic.is_empty() {
        "no error output".into()
    } else {
        diagnostic
    };
    HardwareError::with_kind(
        kind,
        format!("liquidctl {operation} failed ({exit}): {diagnostic}. {guidance}"),
    )
}

fn truncated_diagnostic(stderr: &[u8], capture_truncated: bool) -> String {
    let text = String::from_utf8_lossy(stderr);
    let text = text.trim();
    let mut chars = text.chars();
    let mut diagnostic = chars
        .by_ref()
        .take(MAX_DIAGNOSTIC_CHARS)
        .collect::<String>();
    if chars.next().is_some() || capture_truncated {
        diagnostic.push('…');
    }
    diagnostic
}

#[cfg(test)]
mod tests {
    use std::{
        collections::VecDeque,
        sync::{Arc, Mutex},
    };

    use super::*;

    const Z3_LIST: &str = r#"[
        {
            "description": "NZXT Kraken Z (Z53, Z63 or Z73) (experimental)",
            "vendor_id": 7793,
            "product_id": 12296,
            "release_number": 513,
            "serial_number": "Z3-SERIAL-Å",
            "bus": "hid",
            "address": "/dev/hidraw4",
            "port": null,
            "driver": "KrakenZ3",
            "experimental": true
        }
    ]"#;

    const Z3_STATUS: &str = r#"[
        {
            "bus": "hid",
            "address": "/dev/hidraw4",
            "description": "NZXT Kraken Z (Z53, Z63 or Z73) (experimental)",
            "status": [
                {"key": "Liquid temperature", "value": 31.7, "unit": "°C"},
                {"key": "Pump speed", "value": 2251, "unit": "rpm"},
                {"key": "Pump duty", "value": 55, "unit": "%"},
                {"key": "Fan speed", "value": 1040, "unit": "rpm"},
                {"key": "Fan duty", "value": 42, "unit": "%"},
                {"key": "Firmware version", "value": "1.8.0", "unit": ""}
            ]
        }
    ]"#;

    const X3_LIST: &str = r#"[
        {
            "description": "NZXT Kraken X (X53, X63 or X73)",
            "vendor_id": 7793,
            "product_id": 8199,
            "release_number": 512,
            "serial_number": "X3SERIAL",
            "bus": 1,
            "address": 5,
            "port": [2, 3],
            "driver": "KrakenX3",
            "experimental": false
        }
    ]"#;

    const X3_STATUS: &str = r#"[
        {
            "bus": "1",
            "address": 5,
            "description": "NZXT Kraken X (X53, X63 or X73)",
            "status": [
                {"key": "Liquid temperature", "value": 29.4, "unit": "°C"},
                {"key": "Pump speed", "value": 1948, "unit": "rpm"},
                {"key": "Pump duty", "value": 55, "unit": "%"}
            ]
        }
    ]"#;

    #[derive(Clone)]
    struct MockControl {
        shared: Arc<Mutex<MockState>>,
    }

    struct MockRunner {
        shared: Arc<Mutex<MockState>>,
    }

    struct MockState {
        responses: VecDeque<MockResponse>,
        calls: Vec<Vec<String>>,
    }

    enum MockResponse {
        Success(Vec<u8>),
        TruncatedSuccess(Vec<u8>),
        Exit { code: Option<i32>, stderr: Vec<u8> },
        Error(RunnerError),
    }

    impl MockControl {
        fn new(responses: impl IntoIterator<Item = MockResponse>) -> (Self, MockRunner) {
            let shared = Arc::new(Mutex::new(MockState {
                responses: responses.into_iter().collect(),
                calls: Vec::new(),
            }));
            (
                Self {
                    shared: shared.clone(),
                },
                MockRunner { shared },
            )
        }

        fn calls(&self) -> Vec<Vec<String>> {
            self.shared.lock().unwrap().calls.clone()
        }

        fn remaining(&self) -> usize {
            self.shared.lock().unwrap().responses.len()
        }
    }

    impl CommandRunner for MockRunner {
        fn run(&mut self, args: &[String]) -> Result<ProcessOutput, RunnerError> {
            let mut state = self.shared.lock().unwrap();
            state.calls.push(args.to_vec());
            match state.responses.pop_front().expect("unexpected runner call") {
                MockResponse::Success(stdout) => Ok(ProcessOutput {
                    success: true,
                    code: Some(0),
                    stdout: CapturedOutput {
                        bytes: stdout,
                        truncated: false,
                    },
                    stderr: CapturedOutput {
                        bytes: Vec::new(),
                        truncated: false,
                    },
                }),
                MockResponse::TruncatedSuccess(stdout) => Ok(ProcessOutput {
                    success: true,
                    code: Some(0),
                    stdout: CapturedOutput {
                        bytes: stdout,
                        truncated: true,
                    },
                    stderr: CapturedOutput {
                        bytes: Vec::new(),
                        truncated: false,
                    },
                }),
                MockResponse::Exit { code, stderr } => Ok(ProcessOutput {
                    success: false,
                    code,
                    stdout: CapturedOutput {
                        bytes: Vec::new(),
                        truncated: false,
                    },
                    stderr: CapturedOutput {
                        bytes: stderr,
                        truncated: false,
                    },
                }),
                MockResponse::Error(error) => Err(error),
            }
        }
    }

    fn success(text: &str) -> MockResponse {
        MockResponse::Success(text.as_bytes().to_vec())
    }

    fn backend_with(
        responses: impl IntoIterator<Item = MockResponse>,
    ) -> (LiquidctlHardware, MockControl) {
        let (control, runner) = MockControl::new(responses);
        (LiquidctlHardware::with_runner(runner), control)
    }

    fn points(minimum: u8) -> Vec<CurvePoint> {
        (20..60)
            .map(|temperature| CurvePoint {
                temperature,
                duty: if temperature == 59 {
                    100
                } else {
                    minimum.max(temperature + 20)
                },
            })
            .collect()
    }

    fn device_by_driver<'a>(snapshot: &'a HardwareSnapshot, driver: &str) -> &'a Device {
        snapshot
            .devices
            .iter()
            .find(|device| device.model.contains(driver))
            .unwrap()
    }

    const DISPLAY_LIST: &str = r#"[{
        "description":"NZXT Kraken 2023", "vendor_id":7793, "product_id":12302,
        "serial_number":"LCD-SERIAL", "driver":"KrakenZ3", "bus":"hid",
        "address":"/dev/hidraw5", "port":null
    }]"#;

    #[test]
    fn display_worker_uploads_builtin_once_then_renders_to_private_cleaned_path() {
        let root = tempfile::tempdir().unwrap();
        let status = r#"[{"bus":"hid","address":"/dev/hidraw5","description":"NZXT Kraken 2023","status":[{"key":"Liquid temperature","value":31.2,"unit":"°C"}]}]"#;
        let (backend, control) = backend_with([
            success(DISPLAY_LIST),
            success(""), // built-in
            success(DISPLAY_LIST),
            success(DISPLAY_LIST),
            success(status),
            success(""), // static
        ]);
        let worker = crate::kraken_display::KrakenDisplayWorker::with_sources(
            backend,
            crate::telemetry::HostTelemetry::at_empty_test_root(root.path()),
        );
        let id = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        worker.select(id.clone(), nzxt_cam_core::KrakenDisplayMode::BuiltinLiquid);
        let started = Instant::now();
        while control.calls().len() < 2 && started.elapsed() < Duration::from_secs(1) {
            thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(control.calls().len(), 2);
        thread::sleep(Duration::from_millis(2_050)); // No periodic built-in re-upload.
        assert_eq!(control.calls().len(), 2);
        assert_eq!(control.remaining(), 4);
        worker.select(id, nzxt_cam_core::KrakenDisplayMode::CpuLiquid);
        let started = Instant::now();
        while control.calls().len() < 6 && started.elapsed() < Duration::from_secs(1) {
            thread::sleep(Duration::from_millis(2));
        }
        let calls = control.calls();
        assert_eq!(calls.len(), 6);
        assert_eq!(
            &calls[1],
            &["--serial", "LCD-SERIAL", "set", "lcd", "screen", "liquid"]
        );
        assert_eq!(
            &calls[5][..6],
            &["--serial", "LCD-SERIAL", "set", "lcd", "screen", "static"]
        );
        let image = std::path::PathBuf::from(&calls[5][6]);
        assert!(image.starts_with(std::env::temp_dir()));
        assert_eq!(&std::fs::read(&image).unwrap()[..8], b"\x89PNG\r\n\x1a\n");
        assert_eq!(control.remaining(), 0);
        drop(worker);
        assert!(!image.exists());
    }

    #[test]
    fn selected_lcd_image_refreshes_same_exact_identity_after_two_seconds() {
        let root = tempfile::tempdir().unwrap();
        let status = r#"[{"bus":"hid","address":"/dev/hidraw5","description":"NZXT Kraken 2023","status":[{"key":"Liquid temperature","value":31.2,"unit":"°C"}]}]"#;
        let (backend, control) = backend_with([
            success(DISPLAY_LIST),
            success(DISPLAY_LIST),
            success(status),
            success(""),
            success(DISPLAY_LIST),
            success(DISPLAY_LIST),
            success(status),
            success(""),
        ]);
        let worker = crate::kraken_display::KrakenDisplayWorker::with_sources(
            backend,
            crate::telemetry::HostTelemetry::at_empty_test_root(root.path()),
        );
        let id = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        worker.select(id.clone(), nzxt_cam_core::KrakenDisplayMode::CpuLiquid);
        let started = Instant::now();
        while control.calls().len() < 8 && started.elapsed() < Duration::from_secs(3) {
            thread::sleep(Duration::from_millis(5));
        }
        let calls = control.calls();
        assert_eq!(calls.len(), 8);
        for call in [&calls[3], &calls[7]] {
            assert_eq!(
                &call[..6],
                &["--serial", "LCD-SERIAL", "set", "lcd", "screen", "static"]
            );
        }
        assert_eq!(worker.snapshot().device_id, Some(id));
        drop(worker);
    }

    #[test]
    fn lcd_candidate_is_advertised_before_selection_and_cleared_when_offline() {
        let status = r#"[{"bus":"hid","address":"/dev/hidraw5","description":"NZXT Kraken 2023","status":[{"key":"Liquid temperature","value":31.2,"unit":"°C"}]}]"#;
        let (mut backend, _) =
            backend_with([success(DISPLAY_LIST), success(status), success("[]")]);
        let first = backend.snapshot().unwrap();
        let id = first.devices[0].id.clone();
        assert_eq!(backend.display_candidate(None), Some(id.clone()));
        assert_eq!(backend.display_candidate(Some(&id)), Some(id.clone()));
        let offline = backend.snapshot().unwrap();
        assert!(!offline.devices[0].online);
        assert_eq!(backend.display_candidate(Some(&id)), None);
    }

    #[test]
    fn online_replacement_is_discoverable_but_never_the_selected_identity() {
        let replacement = DISPLAY_LIST
            .replace("LCD-SERIAL", "LCD-NEW")
            .replace("/dev/hidraw5", "/dev/hidraw7");
        let status = r#"[{"bus":"hid","address":"/dev/hidraw7","description":"NZXT Kraken 2023","status":[{"key":"Liquid temperature","value":35.0,"unit":"°C"}]}]"#;
        let (mut backend, control) = backend_with([success(&replacement), success(status)]);
        let previous = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        let current = stable_device_id(&parse_list(replacement.as_bytes()).unwrap()[0]);
        backend.snapshot().unwrap();
        assert_eq!(backend.display_candidate(None), Some(current));
        assert_eq!(backend.display_candidate(Some(&previous)), None);
        assert!(
            control
                .calls()
                .iter()
                .all(|call| call == &["--json", "list"] || call == &["--json", "status"])
        );
    }

    #[test]
    fn lcd_identity_requires_exact_standard_model_unique_serial_and_current_presence() {
        let (mut backend, control) = backend_with([success(DISPLAY_LIST), success("[]")]);
        let id = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        assert_eq!(backend.display_serial(&id).unwrap(), "LCD-SERIAL");
        assert_eq!(
            backend.display_serial(&id).unwrap_err().kind(),
            HardwareErrorKind::Unavailable
        );
        assert_eq!(control.calls().len(), 2);
        for list in [
            DISPLAY_LIST.replace("KrakenZ3", "KrakenX3"),
            DISPLAY_LIST.replace("NZXT Kraken 2023", "NZXT Kraken 2023 Elite"),
            DISPLAY_LIST.replace("12302", "12300"),
            DISPLAY_LIST.replace("LCD-SERIAL", ""),
            format!(
                "[{},{}]",
                &DISPLAY_LIST[1..DISPLAY_LIST.len() - 1],
                &DISPLAY_LIST[1..DISPLAY_LIST.len() - 1]
            ),
        ] {
            let (mut backend, control) = backend_with([success(&list)]);
            let id = registry_device_id(&parse_list(list.as_bytes()).unwrap()[0], false);
            assert!(
                backend.display_serial(&id).is_err(),
                "unexpected accepted list: {list}"
            );
            assert_eq!(
                control.calls(),
                vec![vec!["--json".to_owned(), "list".to_owned()]]
            );
        }
    }

    #[test]
    fn resume_preflight_requires_fresh_online_unique_channel_and_firmware_curve() {
        let status = r#"[{"bus":"hid","address":"/dev/hidraw5","description":"NZXT Kraken 2023","status":[{"key":"Liquid temperature","value":33.1,"unit":"°C"}]}]"#;
        let (mut backend, control) = backend_with([
            success(DISPLAY_LIST),
            success(status),
            success(DISPLAY_LIST),
            success(status),
            success(DISPLAY_LIST),
            success("[]"),
        ]);
        let id = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        let points: Vec<_> = (20..60)
            .map(|temperature| CurvePoint {
                temperature,
                duty: if temperature == 59 { 100 } else { 50 },
            })
            .collect();
        assert!(
            !backend
                .preflight_resume_curve(
                    &DeviceId::new("replacement"),
                    &ChannelId::new("pump"),
                    &points
                )
                .unwrap()
        );
        assert!(
            backend
                .preflight_resume_curve(&id, &ChannelId::new("pump"), &points)
                .unwrap()
        );
        assert!(
            !backend
                .preflight_resume_curve(&id, &ChannelId::new("pump"), &points)
                .unwrap()
        );
        assert_eq!(control.calls().len(), 6);

        let (mut backend, control) = backend_with([success(DISPLAY_LIST), success(status)]);
        let bad: Vec<_> = (20..60)
            .map(|temperature| CurvePoint {
                temperature,
                duty: if temperature == 59 { 100 } else { 0 },
            })
            .collect();
        assert_eq!(
            backend
                .preflight_resume_curve(&id, &ChannelId::new("pump"), &bad)
                .unwrap_err()
                .kind(),
            HardwareErrorKind::InvalidData
        );
        assert_eq!(control.calls().len(), 2); // No firmware command.

        let duplicate = format!(
            "[{},{}]",
            &DISPLAY_LIST[1..DISPLAY_LIST.len() - 1],
            &DISPLAY_LIST[1..DISPLAY_LIST.len() - 1]
        );
        let (mut backend, control) = backend_with([success(&duplicate), success(status)]);
        assert!(
            !backend
                .preflight_resume_curve(&id, &ChannelId::new("pump"), &points)
                .unwrap()
        );
        assert_eq!(control.calls().len(), 2); // Duplicate serial cannot authorize a write.
    }

    #[test]
    fn unselected_worker_never_probes_or_uploads_without_opt_in() {
        let (control, runner) = MockControl::new([]);
        let root = tempfile::tempdir().unwrap();
        let worker = crate::kraken_display::KrakenDisplayWorker::with_sources(
            LiquidctlHardware::with_runner(runner),
            crate::telemetry::HostTelemetry::at_empty_test_root(root.path()),
        );
        thread::sleep(Duration::from_millis(30));
        assert_eq!(
            worker.snapshot(),
            nzxt_cam_core::KrakenDisplaySnapshot::default()
        );
        assert!(control.calls().is_empty());
        drop(worker);
    }

    #[test]
    fn interactive_selection_retries_only_exact_identity_when_replaced() {
        let replacement = DISPLAY_LIST
            .replace("LCD-SERIAL", "LCD-NEW")
            .replace("/dev/hidraw5", "/dev/hidraw7");
        let (control, runner) = MockControl::new([
            success(&replacement), // first attempt: old selection has disappeared
            success(&replacement), // retry after ~2s: only the old identity is checked
        ]);
        let root = tempfile::tempdir().unwrap();
        let worker = crate::kraken_display::KrakenDisplayWorker::with_sources(
            LiquidctlHardware::with_runner(runner),
            crate::telemetry::HostTelemetry::at_empty_test_root(root.path()),
        );
        let previous = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        worker.select(
            previous.clone(),
            nzxt_cam_core::KrakenDisplayMode::CpuGpuLiquid,
        );
        let started = Instant::now();
        while control.calls().len() < 2 && started.elapsed() < Duration::from_secs(3) {
            thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(control.calls().len(), 2);
        assert!(
            control
                .calls()
                .iter()
                .all(|call| call == &["--json", "list"])
        );
        assert_eq!(worker.snapshot().device_id, Some(previous));
        assert_eq!(
            worker.snapshot().mode,
            nzxt_cam_core::KrakenDisplayMode::CpuGpuLiquid
        );
        assert!(worker.snapshot().last_error.is_some());
        drop(worker);
    }

    #[test]
    fn selected_lcd_recovers_on_retry_only_when_exact_identity_returns() {
        let replacement = DISPLAY_LIST
            .replace("LCD-SERIAL", "LCD-NEW")
            .replace("/dev/hidraw5", "/dev/hidraw7");
        let status = r#"[{"bus":"hid","address":"/dev/hidraw5","description":"NZXT Kraken 2023","status":[{"key":"Liquid temperature","value":33.1,"unit":"°C"}]}]"#;
        let (control, runner) = MockControl::new([
            success(&replacement), // old selection gone, no LCD command
            success(DISPLAY_LIST), // same identity reconnects after ~2s
            success(DISPLAY_LIST),
            success(status),
            success(""),
        ]);
        let root = tempfile::tempdir().unwrap();
        let worker = crate::kraken_display::KrakenDisplayWorker::with_sources(
            LiquidctlHardware::with_runner(runner),
            crate::telemetry::HostTelemetry::at_empty_test_root(root.path()),
        );
        let selected = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        worker.select(selected.clone(), nzxt_cam_core::KrakenDisplayMode::Cpu);
        let started = Instant::now();
        while control.calls().len() < 5 && started.elapsed() < Duration::from_secs(3) {
            thread::sleep(Duration::from_millis(5));
        }
        let calls = control.calls();
        assert_eq!(calls.len(), 5);
        assert_eq!(
            &calls[4][..6],
            &["--serial", "LCD-SERIAL", "set", "lcd", "screen", "static"]
        );
        assert_eq!(worker.snapshot().device_id, Some(selected));
        drop(worker);
    }

    #[test]
    fn restored_display_never_selects_or_uploads_to_replacement() {
        let replacement = DISPLAY_LIST
            .replace("LCD-SERIAL", "LCD-NEW")
            .replace("/dev/hidraw5", "/dev/hidraw7");
        let (control, runner) = MockControl::new([success(&replacement)]);
        let root = tempfile::tempdir().unwrap();
        let worker = crate::kraken_display::KrakenDisplayWorker::with_sources(
            LiquidctlHardware::with_runner(runner),
            crate::telemetry::HostTelemetry::at_empty_test_root(root.path()),
        );
        let previous = stable_device_id(&parse_list(DISPLAY_LIST.as_bytes()).unwrap()[0]);
        worker.select_restored(previous.clone(), nzxt_cam_core::KrakenDisplayMode::Cpu);
        assert!(worker.snapshot().last_error.is_some()); // Selected, not applied.
        let started = Instant::now();
        while control.calls().is_empty() && started.elapsed() < Duration::from_secs(1) {
            thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(
            control.calls(),
            vec![vec![String::from("--json"), String::from("list")]]
        );
        assert_eq!(worker.snapshot().device_id, Some(previous));
        assert_eq!(
            worker.snapshot().mode,
            nzxt_cam_core::KrakenDisplayMode::Cpu
        );
        assert!(worker.snapshot().last_error.is_some());
        drop(worker);
    }

    #[test]
    fn display_upload_and_manager_snapshot_cannot_run_liquidctl_concurrently() {
        struct BlockedUpload {
            entered: std::sync::mpsc::Sender<()>,
            release: std::sync::mpsc::Receiver<()>,
        }
        impl CommandRunner for BlockedUpload {
            fn run(&mut self, _args: &[String]) -> Result<ProcessOutput, RunnerError> {
                self.entered.send(()).unwrap();
                self.release.recv().unwrap();
                Ok(ProcessOutput {
                    success: true,
                    code: Some(0),
                    stdout: CapturedOutput {
                        bytes: Vec::new(),
                        truncated: false,
                    },
                    stderr: CapturedOutput {
                        bytes: Vec::new(),
                        truncated: false,
                    },
                })
            }
        }
        let io_lock = Arc::new(Mutex::new(()));
        let (entered_tx, entered_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let mut display = LiquidctlHardware::with_runner_and_lock(
            BlockedUpload {
                entered: entered_tx,
                release: release_rx,
            },
            io_lock.clone(),
        );
        let (control, runner) = MockControl::new([success("[]")]);
        let mut manager = LiquidctlHardware::with_runner_and_lock(runner, io_lock);
        let upload = thread::spawn(move || {
            display.set_display(
                "unique-serial",
                Some(std::path::Path::new("/tmp/private.png")),
            )
        });
        entered_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        let (attempted_tx, attempted_rx) = std::sync::mpsc::channel();
        let snapshot = thread::spawn(move || {
            attempted_tx.send(()).unwrap();
            manager.snapshot()
        });
        attempted_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        thread::sleep(Duration::from_millis(30));
        assert!(
            control.calls().is_empty(),
            "snapshot interleaved with an LCD upload"
        );
        release_tx.send(()).unwrap();
        upload.join().unwrap().unwrap();
        snapshot.join().unwrap().unwrap();
        assert_eq!(
            control.calls(),
            vec![vec!["--json".to_owned(), "list".to_owned()]]
        );
    }

    #[test]
    fn usb_lock_timeout_is_before_spawn_and_preserves_confirmed_curve_state() {
        let (mut backend, control) = backend_with([
            success(Z3_LIST),
            success(Z3_STATUS),
            success(""),
            success(""), // must never be consumed by the blocked write
        ]);
        assert_eq!(backend.usb_lock_timeout, USB_LOCK_ACQUIRE_TIMEOUT);
        let id = backend.snapshot().unwrap().devices[0].id.clone();
        let confirmed = points(20);
        backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &confirmed)
            .unwrap();
        // Scale only the injected acquisition budget, using the same polling
        // and runner dispatch path as production, without a six-second sleep.
        backend.usb_lock_timeout = Duration::from_millis(30);
        let io_lock = backend.io_lock.clone();
        let _held = io_lock.lock().unwrap();
        let mut attempted = confirmed.clone();
        attempted[0].duty += 1;
        let started = Instant::now();
        let error = backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &attempted)
            .unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::Unavailable);
        assert!(error.message().contains("shared USB lock"));
        assert!(error.message().contains("no subprocess spawned"));
        assert!(!error.message().contains("terminated"));
        assert!(!error.message().contains("state is unknown"));
        assert!(started.elapsed() < Duration::from_secs(1));
        assert_eq!(control.calls().len(), 3);
        assert_eq!(control.remaining(), 1);
        let pump = &backend.registry[&id].channels[0];
        assert_eq!(pump.points, confirmed);
        assert_eq!(pump.curve_state, CurveState::Applied);
        assert!(
            !RunnerError::LockTimeout {
                budget: USB_LOCK_ACQUIRE_TIMEOUT
            }
            .write_outcome_unknown()
        );
        assert!(RunnerError::Timeout.write_outcome_unknown());
    }

    #[test]
    fn cancellation_while_waiting_for_usb_lock_never_calls_runner() {
        let (mut backend, control) = backend_with([success("[]")]);
        let io_lock = backend.io_lock.clone();
        let _held = io_lock.lock().unwrap();
        let cancellation = backend.cancellation_handle();
        let (attempted_tx, attempted_rx) = std::sync::mpsc::channel();
        let operation = thread::spawn(move || {
            attempted_tx.send(()).unwrap();
            backend.snapshot()
        });
        attempted_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        // Short real synchronization delay: the production wait has its full
        // six-second budget, so cancellation must wake it rather than expiry.
        thread::sleep(Duration::from_millis(30));
        let started = Instant::now();
        cancellation.cancel();
        let error = operation.join().unwrap().unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::Unavailable);
        assert!(error.message().contains("cancelled before it started"));
        assert!(started.elapsed() < Duration::from_secs(1));
        assert!(control.calls().is_empty());
        assert_eq!(control.remaining(), 1);
    }

    #[test]
    fn pre_spawn_cancellation_and_poison_recovery_keep_usb_serialization() {
        let (mut backend, control) = backend_with([success("[]")]);
        let io_lock = backend.io_lock.clone();
        let poisoned_lock = io_lock.clone();
        assert!(
            thread::spawn(move || {
                let _held = poisoned_lock.lock().unwrap();
                panic!("simulated LCD panic while holding USB lock");
            })
            .join()
            .is_err()
        );
        assert!(io_lock.is_poisoned());
        backend.cancellation_handle().cancel();
        assert_eq!(
            backend.snapshot().unwrap_err().kind(),
            HardwareErrorKind::Unavailable
        );
        assert!(control.calls().is_empty());
        backend.cancellation_handle().reset();
        // Even a poisoned mutex still excludes a competing process.
        let held = io_lock.lock().unwrap_err().into_inner();
        backend.usb_lock_timeout = Duration::from_millis(20);
        assert!(
            backend
                .snapshot()
                .unwrap_err()
                .message()
                .contains("shared USB lock")
        );
        assert!(control.calls().is_empty());
        drop(held);
        assert!(backend.snapshot().unwrap().devices.is_empty());
        assert_eq!(control.calls().len(), 1);
    }

    #[test]
    fn lcd_write_uses_exact_serial_and_fixed_builtin_or_private_static_args() {
        let (mut backend, control) = backend_with([success(""), success("")]);
        backend.set_display("LCD-SERIAL", None).unwrap();
        backend
            .set_display(
                "LCD-SERIAL",
                Some(std::path::Path::new("/tmp/service-only/display.png")),
            )
            .unwrap();
        assert_eq!(
            control.calls(),
            vec![
                vec!["--serial", "LCD-SERIAL", "set", "lcd", "screen", "liquid"]
                    .into_iter()
                    .map(str::to_owned)
                    .collect::<Vec<_>>(),
                vec![
                    "--serial",
                    "LCD-SERIAL",
                    "set",
                    "lcd",
                    "screen",
                    "static",
                    "/tmp/service-only/display.png"
                ]
                .into_iter()
                .map(str::to_owned)
                .collect::<Vec<_>>(),
            ]
        );
    }

    #[test]
    fn offline_snapshot_preserves_identity_but_invalidates_usb_claims() {
        let (mut hardware, _) = backend_with([success(Z3_LIST), success(Z3_STATUS)]);
        let fresh = hardware.snapshot().unwrap();
        assert!(fresh.devices.iter().any(|device| device.online));
        for device in hardware.registry.values_mut() {
            for channel in &mut device.channels {
                channel.curve_state = CurveState::Applied;
            }
        }
        let offline = hardware.offline_snapshot();
        assert!(offline.sequence > fresh.sequence);
        assert_eq!(
            offline
                .devices
                .iter()
                .map(|device| &device.id)
                .collect::<Vec<_>>(),
            fresh
                .devices
                .iter()
                .map(|device| &device.id)
                .collect::<Vec<_>>()
        );
        for device in &offline.devices {
            assert!(!device.online);
            assert!(device.readings.is_empty());
            assert!(
                device
                    .cooling_channels
                    .iter()
                    .all(|channel| channel.curve_state == CurveState::Unverified)
            );
        }
    }

    #[test]
    fn liquidctl_adapter_explicitly_rejects_host_lifecycle_operations() {
        let (mut hardware, control) = backend_with([]);
        let policy = HostControlPolicy {
            channels: Vec::new(),
        };

        for error in [
            hardware.start_host_control(&policy).unwrap_err(),
            hardware.stop_host_control().unwrap_err(),
        ] {
            assert_eq!(error.kind(), HardwareErrorKind::Unsupported);
        }
        let update = HostChannelPolicy {
            channel_id: ChannelId::new("case-fan"),
            curve: nzxt_cam_core::HostCurve {
                source: nzxt_cam_core::HostTemperatureSource::Cpu,
                points: Vec::new(),
            },
        };
        assert_eq!(
            hardware.update_host_control(&[update]).unwrap_err().kind(),
            HardwareErrorKind::Unsupported
        );
        hardware.host_control_shutdown_handle().request_shutdown();
        assert!(control.calls().is_empty());
    }

    #[cfg(unix)]
    #[test]
    fn production_runner_times_out_and_reaps_a_child_process_tree() {
        let cancellation = HardwareCancellation::default();
        let mut runner = ProcessRunner::new(
            PathBuf::from("/bin/sh"),
            Duration::from_millis(40),
            cancellation,
        );
        let started = Instant::now();

        let result = runner.run(&["-c".into(), "sleep 5 & wait".into()]);

        assert!(matches!(result, Err(RunnerError::Timeout)));
        assert!(started.elapsed() < Duration::from_secs(2));
    }

    #[cfg(unix)]
    #[test]
    fn production_runner_cancellation_terminates_a_child_process_tree() {
        let cancellation = HardwareCancellation::default();
        let runner_cancellation = cancellation.clone();
        let started = Instant::now();
        let handle = thread::spawn(move || {
            let mut runner = ProcessRunner::new(
                PathBuf::from("/bin/sh"),
                Duration::from_secs(5),
                runner_cancellation,
            );
            runner.run(&["-c".into(), "sleep 5 & wait".into()])
        });
        thread::sleep(Duration::from_millis(40));
        cancellation.cancel();

        let result = handle.join().unwrap();

        assert!(matches!(
            result,
            Err(RunnerError::Cancelled { started: true })
        ));
        assert!(started.elapsed() < Duration::from_secs(2));
    }

    #[cfg(unix)]
    #[test]
    fn production_runner_bounds_and_records_stdout_and_stderr_truncation() {
        let cancellation = HardwareCancellation::default();
        let mut runner = ProcessRunner::new(
            PathBuf::from("/bin/sh"),
            Duration::from_secs(5),
            cancellation,
        );
        let script = format!(
            "head -c {} /dev/zero; head -c {} /dev/zero >&2; exit 9",
            STDOUT_LIMIT + 257,
            STDERR_LIMIT + 257
        );

        let output = runner.run(&["-c".into(), script]).unwrap();

        assert!(!output.success);
        assert_eq!(output.stdout.bytes.len(), STDOUT_LIMIT);
        assert!(output.stdout.truncated);
        assert_eq!(output.stderr.bytes.len(), STDERR_LIMIT);
        assert!(output.stderr.truncated);
        let diagnostic = truncated_diagnostic(&output.stderr.bytes, output.stderr.truncated);
        assert_eq!(diagnostic.chars().count(), MAX_DIAGNOSTIC_CHARS + 1);
        assert!(diagnostic.ends_with('…'));
        let error = nonzero_exit_error("test operation", &output);
        assert_eq!(error.kind(), HardwareErrorKind::Internal);
        assert!(error.message().len() <= MAX_DIAGNOSTIC_CHARS);
    }

    #[test]
    fn truncated_list_and_status_output_are_rejected_before_json_parsing() {
        let (mut list_hardware, _) = backend_with([MockResponse::TruncatedSuccess(b"[]".to_vec())]);
        let list_error = list_hardware.snapshot().unwrap_err();
        assert_eq!(list_error.kind(), HardwareErrorKind::InvalidData);
        assert!(list_error.message().contains("1048576-byte limit"));

        let (mut status_hardware, _) = backend_with([
            success(Z3_LIST),
            MockResponse::TruncatedSuccess(Z3_STATUS.as_bytes().to_vec()),
        ]);
        let status_error = status_hardware.snapshot().unwrap_err();
        assert_eq!(status_error.kind(), HardwareErrorKind::InvalidData);
        assert!(status_error.message().contains("status refresh output"));
    }

    #[test]
    fn construction_is_side_effect_free() {
        let (mut hardware, control) = backend_with([]);
        assert!(control.calls().is_empty());
        hardware.force_discovery();
        assert!(control.calls().is_empty());
    }

    #[test]
    fn parses_liquidctl_116_null_and_scalar_shapes_and_argb_special_case() {
        let list = r#"[
            {
                "description": null, "vendor_id": "0x1e71", "product_id": 8195,
                "release_number": null, "serial_number": null,
                "bus": 3, "address": "7", "port": [1, "4"],
                "driver": "AuraLed", "experimental": null
            }
        ]"#;
        let status = r#"[
            {
                "bus": "3", "address": 7, "description": null,
                "status": [
                    {"key": "ARGB Channels: 3", "value": "", "unit": ""},
                    {"key": "Ignored text", "value": "active", "unit": ""}
                ]
            }
        ]"#;
        let (mut backend, control) = backend_with([success(list), success(status)]);

        let snapshot = backend.snapshot().unwrap();

        assert_eq!(snapshot.devices.len(), 1);
        let device = &snapshot.devices[0];
        assert_eq!(device.name, "AuraLed");
        assert!(device.model.contains("USB 1e71:2003"));
        assert!(device.model.contains("port 1.4"));
        assert!(device.online);
        assert!(device.cooling_channels.is_empty());
        assert_eq!(
            device.readings,
            vec![Reading::new(
                "ARGB Channels",
                3.0,
                "",
                ReadingKind::ChannelCount
            )]
        );
        assert_eq!(
            control.calls(),
            vec![
                vec!["--json".to_owned(), "list".to_owned()],
                vec!["--json".to_owned(), "status".to_owned()]
            ]
        );
    }

    #[test]
    fn actual_nzxt_fixtures_expose_driver_specific_unverified_curves() {
        let combined_list = format!(
            "[{},{}]",
            &Z3_LIST[1..Z3_LIST.len() - 1],
            &X3_LIST[1..X3_LIST.len() - 1]
        );
        let combined_status = format!(
            "[{},{}]",
            &Z3_STATUS[1..Z3_STATUS.len() - 1],
            &X3_STATUS[1..X3_STATUS.len() - 1]
        );
        let (mut backend, _) = backend_with([success(&combined_list), success(&combined_status)]);

        let snapshot = backend.snapshot().unwrap();
        let z3 = device_by_driver(&snapshot, "KrakenZ3");
        let x3 = device_by_driver(&snapshot, "KrakenX3");

        assert!(z3.id.0.contains("krakenz3-1e71-3008-serial-"));
        assert!(z3.id.0.ends_with("c385"));
        assert_eq!(z3.kind, DeviceKind::LiquidCooler);
        assert_eq!(z3.readings.len(), 5);
        assert_eq!(z3.cooling_channels.len(), 2);
        assert_eq!(z3.cooling_channels[0].id.0, "pump");
        assert_eq!(z3.cooling_channels[0].min_duty, 20);
        assert_eq!(z3.cooling_channels[1].id.0, "fan");
        assert_eq!(z3.cooling_channels[1].min_duty, 0);

        assert_eq!(x3.cooling_channels.len(), 1);
        assert_eq!(x3.cooling_channels[0].id.0, "pump");
        assert_eq!(x3.cooling_channels[0].min_duty, 20);
        for channel in z3.cooling_channels.iter().chain(&x3.cooling_channels) {
            assert_eq!(channel.points.len(), 40);
            assert_eq!(channel.points[0].temperature, 20);
            assert_eq!(channel.points[39].temperature, 59);
            assert_eq!(channel.max_duty, 100);
            assert_eq!(channel.source, TemperatureSource::Liquid);
            assert_eq!(channel.curve_state, CurveState::Unverified);
            let expected = PROGRESSIVE_DUTIES
                .iter()
                .enumerate()
                .map(|(offset, duty)| CurvePoint {
                    temperature: 20 + offset as u8,
                    duty: (*duty).max(channel.min_duty),
                })
                .collect::<Vec<_>>();
            assert_eq!(channel.points, expected);
        }
    }

    #[test]
    fn z3_fan_is_hidden_until_fan_telemetry_is_observed() {
        let no_fan_status = r#"[
            {
                "bus": "hid", "address": "/dev/hidraw4", "description": "NZXT Kraken Z",
                "status": [
                    {"key": "Liquid temperature", "value": 31.7, "unit": "°C"},
                    {"key": "Pump speed", "value": 2251, "unit": "rpm"},
                    {"key": "Pump duty", "value": 55, "unit": "%"}
                ]
            }
        ]"#;
        let (mut backend, _) = backend_with([success(Z3_LIST), success(no_fan_status)]);

        let snapshot = backend.snapshot().unwrap();

        assert_eq!(snapshot.devices[0].cooling_channels.len(), 1);
        assert_eq!(snapshot.devices[0].cooling_channels[0].id.0, "pump");
    }

    #[test]
    fn serial_id_survives_reordering_and_address_changes_with_deterministic_order() {
        let first = r#"[
            {"description":"Unknown","vendor_id":1,"product_id":2,"release_number":null,"serial_number":null,"bus":"usb","address":9,"port":null,"driver":"Other","experimental":false},
            {"description":"Kraken","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"stable","bus":"hid","address":"old","port":null,"driver":"KrakenZ3","experimental":false}
        ]"#;
        let first_status = r#"[
            {"bus":"usb","address":"9","description":"Unknown","status":[]},
            {"bus":"hid","address":"old","description":"Kraken","status":[]}
        ]"#;
        let second = r#"[
            {"description":"Kraken","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"stable","bus":"hid","address":"new","port":null,"driver":"KrakenZ3","experimental":false},
            {"description":"Unknown","vendor_id":1,"product_id":2,"release_number":null,"serial_number":null,"bus":"usb","address":9,"port":null,"driver":"Other","experimental":false}
        ]"#;
        let second_status = r#"[
            {"bus":"hid","address":"new","description":"Kraken","status":[]},
            {"bus":"usb","address":9,"description":"Unknown","status":[]}
        ]"#;
        let (mut backend, _) = backend_with([
            success(first),
            success(first_status),
            success(second),
            success(second_status),
        ]);

        let first_snapshot = backend.snapshot().unwrap();
        let serial_id = first_snapshot
            .devices
            .iter()
            .find(|device| device.name == "Kraken")
            .unwrap()
            .id
            .clone();
        let first_ids = first_snapshot
            .devices
            .iter()
            .map(|device| device.id.clone())
            .collect::<Vec<_>>();
        let second_snapshot = backend.snapshot().unwrap();
        let second_serial_id = second_snapshot
            .devices
            .iter()
            .find(|device| device.name == "Kraken")
            .unwrap()
            .id
            .clone();
        let second_ids = second_snapshot
            .devices
            .iter()
            .map(|device| device.id.clone())
            .collect::<Vec<_>>();

        assert_eq!(serial_id, second_serial_id);
        assert_eq!(first_ids, second_ids);
        assert!(serial_id.0.ends_with("737461626c65"));
    }

    #[test]
    fn no_serial_and_duplicate_serial_devices_are_telemetry_only() {
        let list = r#"[
            {"description":"No serial","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":null,"bus":"hid","address":"a","port":null,"driver":"KrakenZ3","experimental":false},
            {"description":"Duplicate A","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"duplicate","bus":"hid","address":"b","port":null,"driver":"KrakenZ3","experimental":false},
            {"description":"Duplicate B","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"duplicate","bus":"hid","address":"c","port":null,"driver":"KrakenZ3","experimental":false}
        ]"#;
        let status = r#"[
            {"bus":"hid","address":"a","description":"No serial","status":[]},
            {"bus":"hid","address":"b","description":"Duplicate A","status":[]},
            {"bus":"hid","address":"c","description":"Duplicate B","status":[]}
        ]"#;
        let (mut backend, _) = backend_with([success(list), success(status)]);

        let snapshot = backend.snapshot().unwrap();

        assert_eq!(snapshot.devices.len(), 3);
        assert_eq!(
            snapshot
                .devices
                .iter()
                .map(|device| &device.id)
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            3
        );
        assert!(snapshot.devices.iter().all(|device| device.online));
        assert!(
            snapshot
                .devices
                .iter()
                .all(|device| device.cooling_channels.is_empty())
        );
        assert!(
            snapshot
                .devices
                .iter()
                .find(|device| device.name == "No serial")
                .unwrap()
                .id
                .0
                .contains("-path-")
        );
    }

    #[test]
    fn serial_uniqueness_transitions_migrate_aliases_without_phantoms() {
        let unique_b = r#"[
            {"description":"Kraken B","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"same","bus":"hid","address":"b","port":null,"driver":"KrakenZ3","experimental":false}
        ]"#;
        let duplicate = r#"[
            {"description":"Kraken B","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"same","bus":"hid","address":"b","port":null,"driver":"KrakenZ3","experimental":false},
            {"description":"Kraken C","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"same","bus":"hid","address":"c","port":null,"driver":"KrakenZ3","experimental":false}
        ]"#;
        let unique_c = r#"[
            {"description":"Kraken C","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"same","bus":"hid","address":"c","port":null,"driver":"KrakenZ3","experimental":false}
        ]"#;
        let status_b = r#"[
            {"bus":"hid","address":"b","description":"Kraken B","status":[]}
        ]"#;
        let status_duplicate = r#"[
            {"bus":"hid","address":"b","description":"Kraken B","status":[]},
            {"bus":"hid","address":"c","description":"Kraken C","status":[]}
        ]"#;
        let status_c = r#"[
            {"bus":"hid","address":"c","description":"Kraken C","status":[]}
        ]"#;
        let (mut backend, _) = backend_with([
            success(unique_b),
            success(status_b),
            success(duplicate),
            success(status_duplicate),
            success(unique_c),
            success(status_c),
        ]);

        let first = backend.snapshot().unwrap();
        let stable_id = first.devices[0].id.clone();
        assert_eq!(first.devices.len(), 1);
        assert_eq!(first.devices[0].cooling_channels.len(), 1);

        let duplicated = backend.snapshot().unwrap();
        assert_eq!(duplicated.devices.len(), 2);
        assert!(duplicated.devices.iter().all(|device| device.online));
        assert!(
            duplicated
                .devices
                .iter()
                .all(|device| device.cooling_channels.is_empty())
        );
        assert!(
            duplicated
                .devices
                .iter()
                .all(|device| device.id.0.contains("-duplicate-path-"))
        );

        let unique_again = backend.snapshot().unwrap();
        assert_eq!(unique_again.devices.len(), 1);
        assert_eq!(unique_again.devices[0].id, stable_id);
        assert_eq!(unique_again.devices[0].name, "Kraken C");
        assert!(unique_again.devices[0].online);
        assert_eq!(unique_again.devices[0].cooling_channels.len(), 1);
        assert_eq!(backend.registry.len(), 1);
    }

    #[test]
    fn cross_model_serial_aliases_never_migrate_incompatible_channels() {
        let duplicate = r#"[
            {"description":"Z3","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"same","bus":"hid","address":"z","port":null,"driver":"KrakenZ3","experimental":false},
            {"description":"X3","vendor_id":7793,"product_id":8199,"release_number":1,"serial_number":"same","bus":"hid","address":"x","port":null,"driver":"KrakenX3","experimental":false}
        ]"#;
        let duplicate_status = r#"[
            {"bus":"hid","address":"z","description":"Z3","status":[
                {"key":"Pump speed","value":2200,"unit":"rpm"},
                {"key":"Fan speed","value":1000,"unit":"rpm"}
            ]},
            {"bus":"hid","address":"x","description":"X3","status":[
                {"key":"Pump speed","value":2100,"unit":"rpm"}
            ]}
        ]"#;
        let moved_unique_x3 = r#"[
            {"description":"X3","vendor_id":7793,"product_id":8199,"release_number":1,"serial_number":"same","bus":"hid","address":"moved","port":null,"driver":"KrakenX3","experimental":false}
        ]"#;
        let moved_status = r#"[
            {"bus":"hid","address":"moved","description":"X3","status":[
                {"key":"Pump speed","value":2100,"unit":"rpm"}
            ]}
        ]"#;
        let (mut backend, _) = backend_with([
            success(duplicate),
            success(duplicate_status),
            success("[]"),
            success(moved_unique_x3),
            success(moved_status),
        ]);

        let duplicated = backend.snapshot().unwrap();
        assert_eq!(duplicated.devices.len(), 2);
        assert!(
            duplicated
                .devices
                .iter()
                .all(|device| device.cooling_channels.is_empty())
        );

        let absent = backend.snapshot().unwrap();
        assert_eq!(absent.devices.len(), 2);
        assert!(absent.devices.iter().all(|device| !device.online));

        let unique = backend.snapshot().unwrap();
        assert_eq!(unique.devices.len(), 1);
        assert_eq!(unique.devices[0].name, "X3");
        assert_eq!(unique.devices[0].cooling_channels.len(), 1);
        assert_eq!(unique.devices[0].cooling_channels[0].id.0, "pump");
        assert_eq!(unique.devices[0].cooling_channels[0].min_duty, 20);
        assert_eq!(backend.registry.len(), 1);
    }

    #[test]
    fn absent_devices_become_tombstones_and_reconnect_with_points_intact() {
        let moved_list = Z3_LIST.replace("/dev/hidraw4", "/dev/hidraw9");
        let moved_status = Z3_STATUS.replace("/dev/hidraw4", "/dev/hidraw9");
        let (mut backend, _) = backend_with([
            success(Z3_LIST),
            success(Z3_STATUS),
            MockResponse::Success(Vec::new()),
            success("[]"),
            success(&moved_list),
            success(&moved_status),
        ]);

        let first = backend.snapshot().unwrap();
        let id = first.devices[0].id.clone();
        let applied = points(20);
        backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &applied)
            .unwrap();

        let offline = backend.snapshot().unwrap();
        assert_eq!(offline.devices.len(), 1);
        assert!(!offline.devices[0].online);
        assert!(offline.devices[0].readings.is_empty());
        assert_eq!(offline.devices[0].cooling_channels[0].points, applied);
        assert_eq!(
            offline.devices[0].cooling_channels[0].curve_state,
            CurveState::Unverified
        );

        let reconnected = backend.snapshot().unwrap();
        assert_eq!(reconnected.devices.len(), 1);
        assert_eq!(reconnected.devices[0].id, id);
        assert!(reconnected.devices[0].online);
        assert_eq!(reconnected.devices[0].cooling_channels[0].points, applied);
        assert_eq!(
            reconnected.devices[0].cooling_channels[0].curve_state,
            CurveState::Unverified
        );
    }

    #[test]
    fn malformed_and_process_failures_are_classified_with_bounded_diagnostics() {
        let cases = [
            (
                MockResponse::Error(RunnerError::Io {
                    kind: io::ErrorKind::NotFound,
                    message: "missing binary".into(),
                    started: false,
                }),
                HardwareErrorKind::Unavailable,
            ),
            (
                MockResponse::Error(RunnerError::Io {
                    kind: io::ErrorKind::PermissionDenied,
                    message: "cannot execute".into(),
                    started: false,
                }),
                HardwareErrorKind::PermissionDenied,
            ),
            (
                MockResponse::Error(RunnerError::Timeout),
                HardwareErrorKind::Timeout,
            ),
            (
                MockResponse::Exit {
                    code: Some(1),
                    stderr: b"insufficient permissions".to_vec(),
                },
                HardwareErrorKind::PermissionDenied,
            ),
            (
                MockResponse::Exit {
                    code: Some(2),
                    stderr: b"operation not supported by device".to_vec(),
                },
                HardwareErrorKind::Unsupported,
            ),
            (
                MockResponse::Exit {
                    code: Some(3),
                    stderr: b"device disconnected".to_vec(),
                },
                HardwareErrorKind::Unavailable,
            ),
            (
                MockResponse::Exit {
                    code: Some(4),
                    stderr: vec![b'x'; MAX_DIAGNOSTIC_CHARS + 100],
                },
                HardwareErrorKind::Internal,
            ),
        ];
        for (response, expected_kind) in cases {
            let (mut backend, _) = backend_with([response]);
            let error = backend.snapshot().unwrap_err();
            assert_eq!(error.kind(), expected_kind);
            assert!(error.to_string().chars().count() < MAX_DIAGNOSTIC_CHARS + 250);
        }

        let (mut backend, _) = backend_with([success("{ definitely not json")]);
        let error = backend.snapshot().unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::InvalidData);
        assert!(error.to_string().contains("malformed device list JSON"));

        let (mut backend, _) = backend_with([success(Z3_LIST), success("not valid status JSON")]);
        let error = backend.snapshot().unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::InvalidData);
        assert!(error.to_string().contains("malformed status JSON"));
    }

    #[test]
    fn apply_uses_exact_serial_curve_arguments_and_caches_success() {
        let (mut backend, control) = backend_with([
            success(Z3_LIST),
            success(Z3_STATUS),
            success(""),
            success(Z3_STATUS),
        ]);
        backend.set_discovery_ttl(Duration::from_secs(300));
        let snapshot = backend.snapshot().unwrap();
        let id = snapshot.devices[0].id.clone();
        let curve = points(20);

        backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &curve)
            .unwrap();

        let mut expected = vec![
            "--serial".to_owned(),
            "Z3-SERIAL-Å".to_owned(),
            "set".to_owned(),
            "pump".to_owned(),
            "speed".to_owned(),
        ];
        for point in &curve {
            expected.push(point.temperature.to_string());
            expected.push(point.duty.to_string());
        }
        assert_eq!(control.calls()[2], expected);

        let refreshed = backend.snapshot().unwrap();
        let pump = refreshed.devices[0]
            .cooling_channels
            .iter()
            .find(|channel| channel.id.0 == "pump")
            .unwrap();
        assert_eq!(pump.points, curve);
        assert_eq!(pump.curve_state, CurveState::Applied);
        assert_eq!(
            control.calls()[3],
            vec!["--json".to_owned(), "status".to_owned()]
        );
        assert_eq!(control.remaining(), 0);
    }

    #[test]
    fn successful_curve_write_ignores_truncated_stdout() {
        let (mut hardware, _) = backend_with([
            success(Z3_LIST),
            success(Z3_STATUS),
            MockResponse::TruncatedSuccess(vec![b'x'; 8]),
        ]);
        let snapshot = hardware.snapshot().unwrap();
        let id = snapshot.devices[0].id.clone();
        let curve = points(20);

        hardware
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &curve)
            .unwrap();

        let pump = hardware.registry[&id]
            .channels
            .iter()
            .find(|channel| channel.id.0 == "pump")
            .unwrap();
        assert_eq!(pump.curve_state, CurveState::Applied);
        assert_eq!(pump.points, curve);
    }

    #[test]
    fn client_disconnect_cancels_and_invalidates_all_curves_without_losing_cache() {
        let (mut hardware, _) = backend_with([
            success(Z3_LIST),
            success(Z3_STATUS),
            success(""),
            success(""),
        ]);
        let snapshot = hardware.snapshot().unwrap();
        let id = snapshot.devices[0].id.clone();
        let pump_points = points(20);
        let fan_points = points(0);
        hardware
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &pump_points)
            .unwrap();
        hardware
            .apply_firmware_curve(&id, &ChannelId::new("fan"), &fan_points)
            .unwrap();

        hardware.client_disconnected();

        assert!(hardware.cancellation_handle().is_cancelled());
        assert_eq!(hardware.registry.len(), 1);
        let channels = &hardware.registry[&id].channels;
        assert_eq!(channels[0].points, pump_points);
        assert_eq!(channels[1].points, fan_points);
        assert!(
            channels
                .iter()
                .all(|channel| channel.curve_state == CurveState::Unverified)
        );
    }

    #[test]
    fn post_dispatch_failures_make_the_written_channel_unverified() {
        let failures = [
            MockResponse::Exit {
                code: Some(1),
                stderr: b"device disconnected during write".to_vec(),
            },
            MockResponse::Error(RunnerError::Io {
                kind: io::ErrorKind::BrokenPipe,
                message: "output pipe failed".into(),
                started: true,
            }),
        ];

        for failure in failures {
            let (mut backend, _) =
                backend_with([success(Z3_LIST), success(Z3_STATUS), success(""), failure]);
            let snapshot = backend.snapshot().unwrap();
            let id = snapshot.devices[0].id.clone();
            let applied = points(20);
            backend
                .apply_firmware_curve(&id, &ChannelId::new("pump"), &applied)
                .unwrap();
            let mut attempted = applied.clone();
            attempted[0].duty += 1;

            let error = backend
                .apply_firmware_curve(&id, &ChannelId::new("pump"), &attempted)
                .unwrap_err();

            assert!(error.write_outcome_unknown());
            let pump = backend.registry[&id]
                .channels
                .iter()
                .find(|channel| channel.id.0 == "pump")
                .unwrap();
            assert_eq!(pump.points, applied);
            assert_eq!(pump.curve_state, CurveState::Unverified);
        }
    }

    #[test]
    fn pre_spawn_failure_preserves_previous_applied_state() {
        let (mut backend, _) = backend_with([
            success(Z3_LIST),
            success(Z3_STATUS),
            success(""),
            MockResponse::Error(RunnerError::Io {
                kind: io::ErrorKind::NotFound,
                message: "binary disappeared".into(),
                started: false,
            }),
        ]);
        let snapshot = backend.snapshot().unwrap();
        let id = snapshot.devices[0].id.clone();
        let applied = points(20);
        backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &applied)
            .unwrap();

        let error = backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &applied)
            .unwrap_err();

        assert!(!error.write_outcome_unknown());
        assert_eq!(error.kind(), HardwareErrorKind::Unavailable);
        let pump = backend.registry[&id]
            .channels
            .iter()
            .find(|channel| channel.id.0 == "pump")
            .unwrap();
        assert_eq!(pump.curve_state, CurveState::Applied);
    }

    #[test]
    fn every_apply_validation_failure_happens_before_runner_invocation() {
        let list = r#"[
            {"description":"Z3","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"unique","bus":"hid","address":"z","port":null,"driver":"KrakenZ3","experimental":false},
            {"description":"No serial","vendor_id":7793,"product_id":8199,"release_number":1,"serial_number":null,"bus":"hid","address":"n","port":null,"driver":"KrakenX3","experimental":false},
            {"description":"Unknown","vendor_id":1,"product_id":2,"release_number":1,"serial_number":"other","bus":"hid","address":"u","port":null,"driver":"OtherDriver","experimental":false},
            {"description":"Dup 1","vendor_id":7793,"product_id":8199,"release_number":1,"serial_number":"dup","bus":"hid","address":"d1","port":null,"driver":"KrakenX3","experimental":false},
            {"description":"Dup 2","vendor_id":7793,"product_id":12296,"release_number":1,"serial_number":"dup","bus":"hid","address":"d2","port":null,"driver":"KrakenZ3","experimental":false}
        ]"#;
        let status = r#"[
            {"bus":"hid","address":"z","description":"Z3","status":[]},
            {"bus":"hid","address":"n","description":"No serial","status":[]},
            {"bus":"hid","address":"u","description":"Unknown","status":[]},
            {"bus":"hid","address":"d1","description":"Dup 1","status":[]},
            {"bus":"hid","address":"d2","description":"Dup 2","status":[]}
        ]"#;
        let (mut backend, control) = backend_with([success(list), success(status)]);
        let snapshot = backend.snapshot().unwrap();
        let id = |name: &str| {
            snapshot
                .devices
                .iter()
                .find(|device| device.name == name)
                .unwrap()
                .id
                .clone()
        };
        let valid = points(20);
        let baseline_calls = control.calls().len();

        let rejections = [
            backend.apply_firmware_curve(
                &DeviceId::new("missing"),
                &ChannelId::new("pump"),
                &valid,
            ),
            backend.apply_firmware_curve(&id("No serial"), &ChannelId::new("pump"), &points(50)),
            backend.apply_firmware_curve(&id("Dup 1"), &ChannelId::new("pump"), &points(50)),
            backend.apply_firmware_curve(&id("Unknown"), &ChannelId::new("pump"), &valid),
            backend.apply_firmware_curve(&id("Z3"), &ChannelId::new("fan"), &valid),
            backend.apply_firmware_curve(&id("Z3"), &ChannelId::new("missing"), &valid),
            backend.apply_firmware_curve(&id("Z3"), &ChannelId::new("pump"), &valid[..39]),
        ];
        assert!(rejections.iter().all(Result::is_err));

        let mut unordered = valid.clone();
        unordered.swap(0, 1);
        assert!(
            backend
                .apply_firmware_curve(&id("Z3"), &ChannelId::new("pump"), &unordered)
                .is_err()
        );
        let mut below_minimum = valid.clone();
        below_minimum[0].duty = 19;
        assert!(
            backend
                .apply_firmware_curve(&id("Z3"), &ChannelId::new("pump"), &below_minimum)
                .is_err()
        );
        let mut above_maximum = valid.clone();
        above_maximum[0].duty = 101;
        assert!(
            backend
                .apply_firmware_curve(&id("Z3"), &ChannelId::new("pump"), &above_maximum)
                .is_err()
        );
        let mut decreasing = valid.clone();
        decreasing[1].duty = decreasing[0].duty - 1;
        assert!(
            backend
                .apply_firmware_curve(&id("Z3"), &ChannelId::new("pump"), &decreasing)
                .is_err()
        );
        let mut missing_failsafe = valid.clone();
        missing_failsafe.last_mut().unwrap().duty = 99;
        assert!(
            backend
                .apply_firmware_curve(&id("Z3"), &ChannelId::new("pump"), &missing_failsafe)
                .is_err()
        );

        backend.registry.get_mut(&id("Z3")).unwrap().online = false;
        assert!(
            backend
                .apply_firmware_curve(&id("Z3"), &ChannelId::new("pump"), &valid)
                .is_err()
        );
        assert_eq!(control.calls().len(), baseline_calls);
    }

    #[test]
    fn timeout_marks_only_target_channel_unverified_and_does_not_retry() {
        let (mut backend, control) = backend_with([
            success(Z3_LIST),
            success(Z3_STATUS),
            success(""),
            success(""),
            MockResponse::Error(RunnerError::Timeout),
        ]);
        let snapshot = backend.snapshot().unwrap();
        let id = snapshot.devices[0].id.clone();
        let pump_points = points(20);
        let fan_points = points(0);
        backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &pump_points)
            .unwrap();
        backend
            .apply_firmware_curve(&id, &ChannelId::new("fan"), &fan_points)
            .unwrap();

        let mut attempted = pump_points.clone();
        attempted[0].duty += 1;
        let error = backend
            .apply_firmware_curve(&id, &ChannelId::new("pump"), &attempted)
            .unwrap_err();

        assert_eq!(error.kind(), HardwareErrorKind::Timeout);
        assert_eq!(control.calls().len(), 5);
        let registered = backend.registry.get(&id).unwrap();
        let pump = registered
            .channels
            .iter()
            .find(|channel| channel.id.0 == "pump")
            .unwrap();
        let fan = registered
            .channels
            .iter()
            .find(|channel| channel.id.0 == "fan")
            .unwrap();
        assert_eq!(pump.points, pump_points);
        assert_eq!(pump.curve_state, CurveState::Unverified);
        assert_eq!(fan.points, fan_points);
        assert_eq!(fan.curve_state, CurveState::Applied);
        assert_eq!(control.remaining(), 0);
    }
}
