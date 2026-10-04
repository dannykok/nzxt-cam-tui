use std::{
    fs,
    path::{Path, PathBuf},
};

pub const BOARD_VENDOR: &str = "Gigabyte Technology Co., Ltd.";
pub const BOARD_NAME: &str = "B650 AORUS ELITE AX ICE";
pub const CHIP_NAME: &str = "it8689";
pub const CHIP_ADDRESS: u16 = 0x0a40;
pub const PLATFORM_COMPONENT: &str = "it87.2624";

/// A uniquely discovered, identity-checked IT8689 hwmon device.
///
/// This type intentionally exposes only its canonical read path. Discovery
/// does not open writable files or offer any mutation operation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DiscoveredIt8689 {
    hwmon_path: PathBuf,
}

impl DiscoveredIt8689 {
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.hwmon_path
    }
}

/// Finds exactly one supported IT8689 device under the supplied read-only
/// sysfs roots. Missing, mismatched, and ambiguous devices are all unavailable.
#[must_use]
pub fn discover(hwmon_root: &Path, dmi_root: &Path) -> Option<DiscoveredIt8689> {
    if read_trimmed(dmi_root.join("board_vendor")).as_deref() != Some(BOARD_VENDOR)
        || read_trimmed(dmi_root.join("board_name")).as_deref() != Some(BOARD_NAME)
    {
        return None;
    }

    let candidates = hwmon_directories(hwmon_root)
        .into_iter()
        .filter_map(|hwmon| {
            let name = read_trimmed(hwmon.join("name"))?;
            if name != CHIP_NAME
                && !name
                    .strip_prefix(CHIP_NAME)
                    .is_some_and(|suffix| suffix.starts_with('_'))
            {
                return None;
            }
            let canonical = fs::canonicalize(hwmon).ok()?;
            canonical
                .components()
                .any(|component| component.as_os_str() == PLATFORM_COMPONENT)
                .then_some(DiscoveredIt8689 {
                    hwmon_path: canonical,
                })
        })
        .collect::<Vec<_>>();

    (candidates.len() == 1).then(|| candidates.into_iter().next().unwrap())
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

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, fs};

    use tempfile::TempDir;

    use super::*;

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

        fn supported_board(&self) {
            fs::write(self.dmi.join("board_vendor"), format!("{BOARD_VENDOR}\n")).unwrap();
            fs::write(self.dmi.join("board_name"), format!("{BOARD_NAME}\n")).unwrap();
        }

        fn device(&self, number: usize, name: &str) -> PathBuf {
            let platform = self.temp.path().join(PLATFORM_COMPONENT);
            let sensors = platform.join(format!("sensor-{number}"));
            fs::create_dir_all(&sensors).unwrap();
            let link = self.hwmon.join(format!("hwmon{number}"));
            std::os::unix::fs::symlink(&sensors, link).unwrap();
            fs::write(sensors.join("name"), format!("{name}\n")).unwrap();
            sensors
        }
    }

    fn file_contents(root: &Path) -> BTreeMap<PathBuf, Vec<u8>> {
        fn collect(root: &Path, path: &Path, output: &mut BTreeMap<PathBuf, Vec<u8>>) {
            for entry in fs::read_dir(path).unwrap().flatten() {
                let file_type = entry.file_type().unwrap();
                if file_type.is_symlink() {
                    continue;
                }
                if file_type.is_dir() {
                    collect(root, &entry.path(), output);
                } else if file_type.is_file() {
                    output.insert(
                        entry.path().strip_prefix(root).unwrap().to_owned(),
                        fs::read(entry.path()).unwrap(),
                    );
                }
            }
        }

        let mut output = BTreeMap::new();
        collect(root, root, &mut output);
        output
    }

    #[test]
    fn discovery_requires_exact_vendor_board_chip_and_platform_independently() {
        let fixture = Fixture::new();
        let sensor = fixture.device(1, CHIP_NAME);

        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());
        fs::write(fixture.dmi.join("board_vendor"), "wrong\n").unwrap();
        fs::write(fixture.dmi.join("board_name"), format!("{BOARD_NAME}\n")).unwrap();
        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());
        fs::write(
            fixture.dmi.join("board_vendor"),
            format!("{BOARD_VENDOR}\n"),
        )
        .unwrap();
        fs::write(fixture.dmi.join("board_name"), "wrong\n").unwrap();
        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());

        fixture.supported_board();
        fs::write(sensor.join("name"), "wrong\n").unwrap();
        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());
        fs::write(sensor.join("name"), format!("{CHIP_NAME}-suffix\n")).unwrap();
        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());
        fs::write(sensor.join("name"), format!("{CHIP_NAME}\n")).unwrap();
        assert_eq!(
            discover(&fixture.hwmon, &fixture.dmi).unwrap().path(),
            sensor
        );

        fs::remove_file(fixture.hwmon.join("hwmon1")).unwrap();
        let wrong_platform = fixture.hwmon.join("hwmon1");
        fs::create_dir_all(&wrong_platform).unwrap();
        fs::write(wrong_platform.join("name"), format!("{CHIP_NAME}\n")).unwrap();
        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());
    }

    #[test]
    fn discovery_accepts_exact_name_prefix_policy_and_rejects_ambiguity() {
        let fixture = Fixture::new();
        fixture.supported_board();
        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());

        let first = fixture.device(1, CHIP_NAME);
        assert_eq!(
            discover(&fixture.hwmon, &fixture.dmi).unwrap().path(),
            first
        );
        fixture.device(2, "it8689_aux");
        assert!(discover(&fixture.hwmon, &fixture.dmi).is_none());
    }

    #[test]
    fn discovery_only_reads_the_fake_tree() {
        let fixture = Fixture::new();
        fixture.supported_board();
        let sensor = fixture.device(9, "it8689_variant");
        fs::write(sensor.join("fan3_input"), "900\n").unwrap();
        let before = file_contents(fixture.temp.path());

        let found = discover(&fixture.hwmon, &fixture.dmi).unwrap();

        assert_eq!(found.path(), sensor);
        assert_eq!(file_contents(fixture.temp.path()), before);
    }
}
