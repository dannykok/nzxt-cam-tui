use std::{
    ffi::OsStr,
    fs,
    path::{Path, PathBuf},
};

use nzxt_cam_core::{Device, DeviceId, DeviceKind, Reading, ReadingKind};

use crate::{
    config::HardwareConfig,
    it8689::discover as discover_it8689,
    nvidia::{NvidiaSource, NvmlSource},
};

const MIN_CPU_MILLIDEGREES: i64 = -20_000;
const MAX_CPU_MILLIDEGREES: i64 = 150_000;
const MAX_HWMON_INTEGER: i64 = 1_000_000;

#[derive(Clone, Debug)]
struct TelemetryRoots {
    hwmon: PathBuf,
    dmi: PathBuf,
}

impl TelemetryRoots {
    fn system() -> Self {
        Self {
            hwmon: PathBuf::from("/sys/class/hwmon"),
            dmi: PathBuf::from("/sys/class/dmi/id"),
        }
    }
}

/// Concrete, read-only host telemetry. It has no registration/plugin surface;
/// only the NVML call is injected in tests so no real driver or library is used.
pub struct HostTelemetry {
    config: HardwareConfig,
    roots: TelemetryRoots,
    nvidia: Box<dyn NvidiaSource>,
    appeared: bool,
}

impl HostTelemetry {
    pub fn new(config: HardwareConfig) -> Self {
        Self {
            config,
            roots: TelemetryRoots::system(),
            nvidia: Box::new(NvmlSource::new()),
            appeared: false,
        }
    }

    /// Samples CPU, NVIDIA, and IT8689 independently. An unavailable or invalid
    /// source contributes no readings and cannot suppress another source.
    pub fn sample_device(&mut self) -> Option<Device> {
        let mut readings = Vec::new();

        if let Some(millidegrees) = sample_k10temp(&self.roots.hwmon) {
            readings.push(Reading::new(
                "CPU Tctl",
                millidegrees as f64 / 1_000.0,
                "°C",
                ReadingKind::Temperature,
            ));
        }

        if let Some(uuid) = self.config.nvidia_uuid.as_deref()
            && let Some(temperature) = self
                .nvidia
                .temperature_celsius(uuid, self.config.nvidia_pci_bus_id.as_deref())
            && temperature.is_finite()
        {
            readings.push(Reading::new(
                "NVIDIA GPU",
                temperature,
                "°C",
                ReadingKind::Temperature,
            ));
        }

        readings.extend(sample_it8689(&self.roots.hwmon, &self.roots.dmi));

        if readings.is_empty() {
            return self.appeared.then(host_device_tombstone);
        }
        self.appeared = true;
        Some(host_device(readings, true))
    }

    #[cfg(test)]
    pub(crate) fn at_empty_test_root(root: &Path) -> Self {
        // Default config has no NVIDIA identity, so the lazy NVML adapter is
        // never initialized. All filesystem observations remain in this root.
        let mut telemetry = Self::new(HardwareConfig::default());
        telemetry.roots = TelemetryRoots {
            hwmon: root.join("hwmon"),
            dmi: root.join("dmi"),
        };
        telemetry
    }

    #[cfg(test)]
    fn with_source_and_roots(
        config: HardwareConfig,
        nvidia: impl NvidiaSource + 'static,
        hwmon: PathBuf,
        dmi: PathBuf,
    ) -> Self {
        Self {
            config,
            roots: TelemetryRoots { hwmon, dmi },
            nvidia: Box::new(nvidia),
            appeared: false,
        }
    }
}

fn host_device_tombstone() -> Device {
    host_device(Vec::new(), false)
}

fn host_device(readings: Vec<Reading>, online: bool) -> Device {
    Device {
        id: DeviceId::new("host-telemetry"),
        name: "Host telemetry".into(),
        model: "Read-only host sensors".into(),
        kind: DeviceKind::FanController,
        online,
        readings,
        cooling_channels: Vec::new(),
    }
}

pub(crate) fn sample_k10temp(hwmon_root: &Path) -> Option<i64> {
    let mut matches = Vec::new();
    for hwmon in hwmon_directories(hwmon_root) {
        if read_trimmed(hwmon.join("name")).as_deref() != Some("k10temp") {
            continue;
        }
        let Ok(entries) = fs::read_dir(&hwmon) else {
            continue;
        };
        for entry in entries.flatten() {
            let file_name = entry.file_name();
            let Some(index) = temperature_label_index(&file_name) else {
                continue;
            };
            if read_trimmed(entry.path()).as_deref() == Some("Tctl") {
                matches.push(hwmon.join(format!("temp{index}_input")));
            }
        }
    }
    if matches.len() != 1 {
        return None;
    }
    read_bounded_integer(&matches[0], MIN_CPU_MILLIDEGREES, MAX_CPU_MILLIDEGREES)
}

fn temperature_label_index(file_name: &OsStr) -> Option<&str> {
    let file_name = file_name.to_str()?;
    let index = file_name.strip_prefix("temp")?.strip_suffix("_label")?;
    (!index.is_empty() && index.bytes().all(|byte| byte.is_ascii_digit())).then_some(index)
}

fn sample_it8689(hwmon_root: &Path, dmi_root: &Path) -> Vec<Reading> {
    let Some(device) = discover_it8689(hwmon_root, dmi_root) else {
        return Vec::new();
    };

    let mut readings = Vec::new();
    for channel in [3, 4] {
        if let Some(speed) = read_bounded_integer(
            &device.path().join(format!("fan{channel}_input")),
            0,
            MAX_HWMON_INTEGER,
        ) {
            readings.push(Reading::new(
                format!("IT8689 fan {channel}"),
                speed as f64,
                "rpm",
                ReadingKind::Speed,
            ));
        }
        if let Some(mode) =
            read_bounded_integer(&device.path().join(format!("pwm{channel}_enable")), 0, 2)
        {
            readings.push(Reading::new(
                format!("IT8689 fan {channel} mode"),
                mode as f64,
                "",
                ReadingKind::Mode,
            ));
        }
    }
    readings
}

fn hwmon_directories(root: &Path) -> Vec<PathBuf> {
    let Ok(entries) = fs::read_dir(root) else {
        return Vec::new();
    };
    entries
        .flatten()
        .filter_map(|entry| {
            let name = entry.file_name();
            let name = name.to_str()?;
            let suffix = name.strip_prefix("hwmon")?;
            (!suffix.is_empty() && suffix.bytes().all(|byte| byte.is_ascii_digit()))
                .then(|| entry.path())
        })
        .collect()
}

fn read_trimmed(path: impl AsRef<Path>) -> Option<String> {
    fs::read_to_string(path)
        .ok()
        .map(|value| value.trim().to_owned())
}

fn read_bounded_integer(path: &Path, minimum: i64, maximum: i64) -> Option<i64> {
    let value = read_trimmed(path)?.parse::<i64>().ok()?;
    (minimum..=maximum).contains(&value).then_some(value)
}

#[cfg(test)]
mod tests {
    use std::{
        collections::VecDeque,
        fs,
        sync::{Arc, Mutex},
    };

    use tempfile::TempDir;

    use super::*;
    use crate::it8689::{BOARD_NAME, BOARD_VENDOR, PLATFORM_COMPONENT};

    #[derive(Clone, Default)]
    struct FakeNvidia {
        state: Arc<Mutex<FakeNvidiaState>>,
    }

    #[derive(Clone, Copy)]
    enum FakeNvidiaOutcome {
        Temperature(f64),
        InitFailure,
        DeviceReadFailure,
        TemperatureReadFailure,
        PciMismatch,
    }

    #[derive(Default)]
    struct FakeNvidiaState {
        outcomes: VecDeque<FakeNvidiaOutcome>,
        calls: Vec<(String, Option<String>)>,
    }

    impl FakeNvidia {
        fn with_outcomes(outcomes: impl IntoIterator<Item = FakeNvidiaOutcome>) -> Self {
            let source = Self::default();
            source.state.lock().unwrap().outcomes.extend(outcomes);
            source
        }
    }

    impl NvidiaSource for FakeNvidia {
        fn temperature_celsius(
            &mut self,
            uuid: &str,
            expected_pci_bus_id: Option<&str>,
        ) -> Option<f64> {
            let mut state = self.state.lock().unwrap();
            state
                .calls
                .push((uuid.to_owned(), expected_pci_bus_id.map(str::to_owned)));
            match state.outcomes.pop_front() {
                Some(FakeNvidiaOutcome::Temperature(value)) => Some(value),
                Some(
                    FakeNvidiaOutcome::InitFailure
                    | FakeNvidiaOutcome::DeviceReadFailure
                    | FakeNvidiaOutcome::TemperatureReadFailure
                    | FakeNvidiaOutcome::PciMismatch,
                )
                | None => None,
            }
        }
    }

    struct Fixture {
        temp: TempDir,
        hwmon: PathBuf,
        dmi: PathBuf,
    }

    impl Fixture {
        fn new() -> Self {
            let temp = tempfile::tempdir().unwrap();
            let hwmon = temp.path().join("hwmon");
            let dmi = temp.path().join("dmi");
            fs::create_dir_all(&hwmon).unwrap();
            fs::create_dir_all(&dmi).unwrap();
            Self { temp, hwmon, dmi }
        }

        fn hwmon(&self, number: usize, name: &str) -> PathBuf {
            let path = self.hwmon.join(format!("hwmon{number}"));
            fs::create_dir_all(&path).unwrap();
            fs::write(path.join("name"), format!("{name}\n")).unwrap();
            path
        }

        fn supported_board(&self) {
            fs::write(self.dmi.join("board_vendor"), format!("{BOARD_VENDOR}\n")).unwrap();
            fs::write(self.dmi.join("board_name"), format!("{BOARD_NAME}\n")).unwrap();
        }

        fn telemetry(&self, config: HardwareConfig, nvidia: FakeNvidia) -> HostTelemetry {
            HostTelemetry::with_source_and_roots(
                config,
                nvidia,
                self.hwmon.clone(),
                self.dmi.clone(),
            )
        }
    }

    #[test]
    fn k10temp_rescans_renumbered_hwmon_and_requires_one_exact_tctl() {
        let fixture = Fixture::new();
        let first = fixture.hwmon(2, "k10temp");
        fs::write(first.join("temp1_label"), "Tctl\n").unwrap();
        fs::write(first.join("temp1_input"), "42500\n").unwrap();
        assert_eq!(sample_k10temp(&fixture.hwmon), Some(42_500));

        fs::rename(&first, fixture.hwmon.join("hwmon91")).unwrap();
        fs::write(fixture.hwmon.join("hwmon91/temp1_input"), "43000\n").unwrap();
        assert_eq!(sample_k10temp(&fixture.hwmon), Some(43_000));

        let second = fixture.hwmon(7, "k10temp");
        fs::write(second.join("temp2_label"), "Tctl\n").unwrap();
        fs::write(second.join("temp2_input"), "44000\n").unwrap();
        assert_eq!(sample_k10temp(&fixture.hwmon), None);

        fs::write(second.join("temp2_label"), "Tccd1\n").unwrap();
        assert_eq!(sample_k10temp(&fixture.hwmon), Some(43_000));
        fs::write(fixture.hwmon.join("hwmon91/temp1_label"), "tctl\n").unwrap();
        assert_eq!(sample_k10temp(&fixture.hwmon), None);
    }

    #[test]
    fn k10temp_omits_missing_malformed_and_out_of_bounds_inputs() {
        let fixture = Fixture::new();
        let hwmon = fixture.hwmon(0, "k10temp");
        fs::write(hwmon.join("temp3_label"), "Tctl\n").unwrap();
        assert_eq!(sample_k10temp(&fixture.hwmon), None);

        for value in ["not-an-integer", "-20001", "150001"] {
            fs::write(hwmon.join("temp3_input"), value).unwrap();
            assert_eq!(sample_k10temp(&fixture.hwmon), None);
        }
        for (value, expected) in [("-20000", -20_000), ("150000", 150_000)] {
            fs::write(hwmon.join("temp3_input"), value).unwrap();
            assert_eq!(sample_k10temp(&fixture.hwmon), Some(expected));
        }
    }

    fn make_it8689(fixture: &Fixture, number: usize, name: &str) -> PathBuf {
        let platform = fixture.temp.path().join(PLATFORM_COMPONENT);
        let sensors = platform.join(format!("sensor-{number}"));
        fs::create_dir_all(&sensors).unwrap();
        let link = fixture.hwmon.join(format!("hwmon{number}"));
        std::os::unix::fs::symlink(&sensors, &link).unwrap();
        fs::write(sensors.join("name"), format!("{name}\n")).unwrap();
        sensors
    }

    #[test]
    fn it8689_reads_raw_channels_partially_and_never_changes_files() {
        let fixture = Fixture::new();
        fixture.supported_board();
        let sensor = make_it8689(&fixture, 12, "it8689_variant");
        fs::write(sensor.join("fan3_input"), "975\n").unwrap();
        fs::write(sensor.join("pwm3_enable"), "2\n").unwrap();
        fs::write(sensor.join("fan4_input"), "bad\n").unwrap();
        fs::write(sensor.join("pwm4_enable"), "0\n").unwrap();
        let before = ["975\n", "2\n", "bad\n", "0\n"];

        let readings = sample_it8689(&fixture.hwmon, &fixture.dmi);
        assert_eq!(
            readings
                .iter()
                .map(|reading| (&*reading.label, reading.value, reading.kind))
                .collect::<Vec<_>>(),
            vec![
                ("IT8689 fan 3", 975.0, ReadingKind::Speed),
                ("IT8689 fan 3 mode", 2.0, ReadingKind::Mode),
                ("IT8689 fan 4 mode", 0.0, ReadingKind::Mode),
            ]
        );
        for (file, expected) in [
            ("fan3_input", before[0]),
            ("pwm3_enable", before[1]),
            ("fan4_input", before[2]),
            ("pwm4_enable", before[3]),
        ] {
            assert_eq!(fs::read_to_string(sensor.join(file)).unwrap(), expected);
        }

        fs::remove_file(sensor.join("pwm3_enable")).unwrap();
        let partial = sample_it8689(&fixture.hwmon, &fixture.dmi);
        assert_eq!(
            partial
                .iter()
                .map(|reading| reading.label.as_str())
                .collect::<Vec<_>>(),
            vec!["IT8689 fan 3", "IT8689 fan 4 mode"]
        );
    }

    #[test]
    fn it8689_omits_negative_and_excessive_integers_independently() {
        let fixture = Fixture::new();
        fixture.supported_board();
        let sensor = make_it8689(&fixture, 4, "it8689");
        fs::write(sensor.join("fan3_input"), "-1\n").unwrap();
        fs::write(sensor.join("pwm3_enable"), "1000001\n").unwrap();
        fs::write(sensor.join("fan4_input"), "1000000\n").unwrap();
        fs::write(sensor.join("pwm4_enable"), "1\n").unwrap();

        let readings = sample_it8689(&fixture.hwmon, &fixture.dmi);
        assert_eq!(readings.len(), 2);
        assert_eq!(readings[0].label, "IT8689 fan 4");
        assert_eq!(readings[1].label, "IT8689 fan 4 mode");
    }

    #[test]
    fn it8689_modes_are_limited_to_zero_through_two() {
        let fixture = Fixture::new();
        fixture.supported_board();
        let sensor = make_it8689(&fixture, 8, "it8689");
        fs::write(sensor.join("pwm3_enable"), "-1\n").unwrap();
        fs::write(sensor.join("pwm4_enable"), "3\n").unwrap();
        assert!(sample_it8689(&fixture.hwmon, &fixture.dmi).is_empty());

        fs::write(sensor.join("pwm3_enable"), "0\n").unwrap();
        fs::write(sensor.join("pwm4_enable"), "2\n").unwrap();
        assert_eq!(sample_it8689(&fixture.hwmon, &fixture.dmi).len(), 2);
    }

    #[test]
    fn nvidia_is_not_loaded_or_called_without_configuration() {
        let fixture = Fixture::new();
        let fake = FakeNvidia::with_outcomes([FakeNvidiaOutcome::Temperature(60.0)]);
        let state = fake.state.clone();
        let mut telemetry = fixture.telemetry(HardwareConfig::default(), fake);

        assert!(state.lock().unwrap().calls.is_empty());
        assert!(telemetry.sample_device().is_none());
        assert!(state.lock().unwrap().calls.is_empty());
    }

    #[test]
    fn nvidia_uses_uuid_and_pci_and_retries_all_operational_failures() {
        let fixture = Fixture::new();
        let fake = FakeNvidia::with_outcomes([
            FakeNvidiaOutcome::InitFailure,
            FakeNvidiaOutcome::DeviceReadFailure,
            FakeNvidiaOutcome::TemperatureReadFailure,
            FakeNvidiaOutcome::Temperature(61.0),
            FakeNvidiaOutcome::PciMismatch,
            FakeNvidiaOutcome::Temperature(62.0),
        ]);
        let state = fake.state.clone();
        let config = HardwareConfig {
            nvidia_uuid: Some("GPU-selected".into()),
            nvidia_pci_bus_id: Some("00000000:01:00.0".into()),
            host_control: None,
        };
        let mut telemetry = fixture.telemetry(config, fake);

        assert!(state.lock().unwrap().calls.is_empty());
        assert!(telemetry.sample_device().is_none()); // lazy initialization failure
        assert!(telemetry.sample_device().is_none()); // UUID/device read failure
        assert!(telemetry.sample_device().is_none()); // temperature read failure
        assert_eq!(telemetry.sample_device().unwrap().readings[0].value, 61.0); // PCI match
        assert!(!telemetry.sample_device().unwrap().online); // PCI mismatch
        assert_eq!(telemetry.sample_device().unwrap().readings[0].value, 62.0); // later retry
        assert_eq!(state.lock().unwrap().calls.len(), 6);
        assert!(
            state
                .lock()
                .unwrap()
                .calls
                .iter()
                .all(|call| { call == &("GPU-selected".into(), Some("00000000:01:00.0".into())) })
        );
    }

    #[test]
    fn collectors_are_independent_and_host_device_has_stable_shape_and_tombstone() {
        let fixture = Fixture::new();
        let cpu = fixture.hwmon(5, "k10temp");
        fs::write(cpu.join("temp1_label"), "Tctl\n").unwrap();
        fs::write(cpu.join("temp1_input"), "51250\n").unwrap();
        let fake = FakeNvidia::with_outcomes([
            FakeNvidiaOutcome::DeviceReadFailure,
            FakeNvidiaOutcome::Temperature(65.0),
            FakeNvidiaOutcome::TemperatureReadFailure,
        ]);
        let config = HardwareConfig {
            nvidia_uuid: Some("GPU-selected".into()),
            nvidia_pci_bus_id: None,
            host_control: None,
        };
        let mut telemetry = fixture.telemetry(config, fake);

        let first = telemetry.sample_device().unwrap();
        assert_eq!(first.id, DeviceId::new("host-telemetry"));
        assert_eq!(first.name, "Host telemetry");
        assert_eq!(first.kind, DeviceKind::FanController);
        assert!(first.cooling_channels.is_empty());
        assert_eq!(first.readings.len(), 1); // GPU failure did not suppress CPU

        fs::write(cpu.join("temp1_input"), "bad\n").unwrap();
        let second = telemetry.sample_device().unwrap();
        assert_eq!(second.readings[0].label, "NVIDIA GPU"); // CPU failure did not suppress GPU

        let third = telemetry.sample_device().unwrap();
        assert!(!third.online);
        assert!(third.readings.is_empty());
        assert!(third.cooling_channels.is_empty());
    }
}
