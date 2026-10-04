use std::{
    collections::HashSet,
    error::Error,
    fmt, fs,
    io::{self, Read},
    os::unix::fs::{MetadataExt, OpenOptionsExt},
    path::Path,
};

use nzxt_cam_core::{ChannelId, HostChannelCapability};
use serde::{Deserialize, Serialize};

use crate::it8689::{BOARD_NAME, BOARD_VENDOR, CHIP_ADDRESS, CHIP_NAME, PLATFORM_COMPONENT};

pub const HARDWARE_CONFIG_PATH: &str = "/etc/nzxt-cam/hardware.toml";
const MAX_CONFIG_BYTES: usize = 4 * 1024;
const MAX_IDENTITY_BYTES: usize = 128;
const MAX_CHANNEL_ID_BYTES: usize = 128;
const MAX_CHANNEL_NAME_BYTES: usize = 128;
const MAX_CONFIG_ERROR_BYTES: usize = 512;
const GROUP_OR_WORLD_WRITABLE: u32 = 0o022;

#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareConfig {
    pub nvidia_uuid: Option<String>,
    pub nvidia_pci_bus_id: Option<String>,
    pub host_control: Option<HostControlConfig>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostControlConfig {
    pub board_vendor: String,
    pub board_name: String,
    pub chip_name: String,
    pub chip_address: u16,
    pub platform_component: String,
    pub channels: Vec<HostChannelConfig>,
}

impl HostControlConfig {
    #[must_use]
    pub fn capabilities(&self) -> Vec<HostChannelCapability> {
        self.channels.iter().map(Into::into).collect()
    }
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostChannelConfig {
    pub id: ChannelId,
    pub name: String,
    pub pwm_channel: u8,
    pub fan_channel: u8,
    pub minimum_duty_percent: u8,
}

impl From<&HostChannelConfig> for HostChannelCapability {
    fn from(channel: &HostChannelConfig) -> Self {
        Self {
            channel_id: channel.id.clone(),
            name: channel.name.clone(),
            minimum_duty_percent: channel.minimum_duty_percent,
        }
    }
}

impl From<HostChannelConfig> for HostChannelCapability {
    fn from(channel: HostChannelConfig) -> Self {
        Self {
            channel_id: channel.id,
            name: channel.name,
            minimum_duty_percent: channel.minimum_duty_percent,
        }
    }
}

impl HardwareConfig {
    /// Loads a configuration from an arbitrary path without production
    /// ownership checks. This is intended for tests and offline validation.
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let file = match fs::File::open(path) {
            Ok(file) => file,
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Self::default()),
            Err(error) => return Err(read_error(path, error)),
        };
        Self::read(file, path)
    }

    /// Loads the fixed production configuration path and requires an existing
    /// file to be a root-owned regular file not writable by group or others.
    pub fn load_default() -> Result<Self, ConfigError> {
        let path = Path::new(HARDWARE_CONFIG_PATH);
        let path_metadata = match fs::symlink_metadata(path) {
            Ok(metadata) => metadata,
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Self::default()),
            Err(error) => return Err(metadata_error(path, error)),
        };
        validate_production_metadata(ConfigFileMetadata::from(&path_metadata))?;

        let file = match fs::OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
            .open(path)
        {
            Ok(file) => file,
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Self::default()),
            Err(error) => return Err(read_error(path, error)),
        };
        let opened_metadata = file
            .metadata()
            .map_err(|error| metadata_error(path, error))?;
        validate_production_metadata(ConfigFileMetadata::from(&opened_metadata))?;
        Self::read(file, path)
    }

    fn read(file: fs::File, path: &Path) -> Result<Self, ConfigError> {
        let mut bytes = Vec::with_capacity(MAX_CONFIG_BYTES + 1);
        file.take((MAX_CONFIG_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(|error| read_error(path, error))?;
        if bytes.len() > MAX_CONFIG_BYTES {
            return Err(ConfigError::new(format!(
                "hardware config exceeds the {MAX_CONFIG_BYTES}-byte limit"
            )));
        }
        let text = std::str::from_utf8(&bytes)
            .map_err(|_| ConfigError::new("hardware config is not valid UTF-8"))?;
        let config: Self = toml::from_str(text)
            .map_err(|error| ConfigError::new(format!("invalid hardware config: {error}")))?;
        config.validate()?;
        Ok(config)
    }

    fn validate(&self) -> Result<(), ConfigError> {
        if let Some(uuid) = &self.nvidia_uuid {
            validate_identity("nvidia_uuid", uuid)?;
        }
        if let Some(pci) = &self.nvidia_pci_bus_id {
            if self.nvidia_uuid.is_none() {
                return Err(ConfigError::new("nvidia_pci_bus_id requires nvidia_uuid"));
            }
            validate_identity("nvidia_pci_bus_id", pci)?;
        }
        if let Some(host_control) = &self.host_control {
            host_control.validate()?;
        }
        Ok(())
    }
}

impl HostControlConfig {
    pub(crate) fn validate(&self) -> Result<(), ConfigError> {
        for (field, actual, expected) in [
            ("board_vendor", self.board_vendor.as_str(), BOARD_VENDOR),
            ("board_name", self.board_name.as_str(), BOARD_NAME),
            ("chip_name", self.chip_name.as_str(), CHIP_NAME),
            (
                "platform_component",
                self.platform_component.as_str(),
                PLATFORM_COMPONENT,
            ),
        ] {
            if actual != expected {
                return Err(ConfigError::new(format!(
                    "host_control.{field} must exactly match {expected:?}"
                )));
            }
        }
        if self.chip_address != CHIP_ADDRESS {
            return Err(ConfigError::new(format!(
                "host_control.chip_address must exactly match {CHIP_ADDRESS} (0x{CHIP_ADDRESS:04x})"
            )));
        }
        if !(1..=2).contains(&self.channels.len()) {
            return Err(ConfigError::new(
                "host_control.channels must contain one or two configured candidates",
            ));
        }

        let mut ids = HashSet::new();
        let mut names = HashSet::new();
        let mut pwm_channels = HashSet::new();
        let mut fan_channels = HashSet::new();
        for channel in &self.channels {
            validate_label(
                "host_control channel id",
                &channel.id.0,
                MAX_CHANNEL_ID_BYTES,
            )?;
            validate_label(
                "host_control channel name",
                &channel.name,
                MAX_CHANNEL_NAME_BYTES,
            )?;
            if !ids.insert(channel.id.clone()) {
                return Err(ConfigError::new(format!(
                    "host_control channel id {:?} is duplicated",
                    channel.id.0
                )));
            }
            if !names.insert(channel.name.as_str()) {
                return Err(ConfigError::new(format!(
                    "host_control channel name {:?} is duplicated",
                    channel.name
                )));
            }
            if !pwm_channels.insert(channel.pwm_channel) {
                return Err(ConfigError::new(format!(
                    "host_control pwm_channel {} is duplicated",
                    channel.pwm_channel
                )));
            }
            if !fan_channels.insert(channel.fan_channel) {
                return Err(ConfigError::new(format!(
                    "host_control fan_channel {} is duplicated",
                    channel.fan_channel
                )));
            }
            if !matches!((channel.pwm_channel, channel.fan_channel), (3, 3) | (4, 4)) {
                return Err(ConfigError::new(format!(
                    "host_control channel {:?} must use candidate mapping (3, 3) or (4, 4)",
                    channel.id.0
                )));
            }
            if !(30..=100).contains(&channel.minimum_duty_percent) {
                return Err(ConfigError::new(format!(
                    "host_control channel {:?} minimum_duty_percent must be in 30..=100",
                    channel.id.0
                )));
            }
        }
        Ok(())
    }
}

fn validate_identity(field: &str, value: &str) -> Result<(), ConfigError> {
    if value.is_empty() || value.trim() != value || value.chars().any(char::is_control) {
        return Err(ConfigError::new(format!(
            "{field} must be a non-empty identity without surrounding whitespace or control characters"
        )));
    }
    if value.len() > MAX_IDENTITY_BYTES {
        return Err(ConfigError::new(format!(
            "{field} exceeds the {MAX_IDENTITY_BYTES}-byte limit"
        )));
    }
    Ok(())
}

fn validate_label(field: &str, value: &str, maximum_bytes: usize) -> Result<(), ConfigError> {
    if value.is_empty() || value.trim() != value || value.chars().any(char::is_control) {
        return Err(ConfigError::new(format!(
            "{field} must be a non-empty value without surrounding whitespace or control characters"
        )));
    }
    if value.len() > maximum_bytes {
        return Err(ConfigError::new(format!(
            "{field} exceeds the {maximum_bytes}-byte limit"
        )));
    }
    Ok(())
}

fn read_error(path: &Path, error: io::Error) -> ConfigError {
    ConfigError::new(format!(
        "could not read hardware config {}: {error}",
        path.display()
    ))
}

fn metadata_error(path: &Path, error: io::Error) -> ConfigError {
    ConfigError::new(format!(
        "could not inspect hardware config {}: {error}",
        path.display()
    ))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ConfigFileMetadata {
    is_regular_file: bool,
    owner_uid: u32,
    mode: u32,
}

impl From<&fs::Metadata> for ConfigFileMetadata {
    fn from(metadata: &fs::Metadata) -> Self {
        Self {
            is_regular_file: metadata.is_file(),
            owner_uid: metadata.uid(),
            mode: metadata.mode(),
        }
    }
}

fn validate_production_metadata(metadata: ConfigFileMetadata) -> Result<(), ConfigError> {
    if !metadata.is_regular_file {
        return Err(ConfigError::new("hardware config must be a regular file"));
    }
    if metadata.owner_uid != 0 {
        return Err(ConfigError::new(
            "hardware config must be owned by root (uid 0)",
        ));
    }
    if metadata.mode & GROUP_OR_WORLD_WRITABLE != 0 {
        return Err(ConfigError::new(
            "hardware config must not be writable by group or others",
        ));
    }
    Ok(())
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConfigError {
    message: String,
}

impl ConfigError {
    fn new(message: impl Into<String>) -> Self {
        let mut message = message.into();
        if message.len() > MAX_CONFIG_ERROR_BYTES {
            let mut boundary = MAX_CONFIG_ERROR_BYTES;
            while !message.is_char_boundary(boundary) {
                boundary -= 1;
            }
            message.truncate(boundary);
        }
        Self { message }
    }
}

impl fmt::Display for ConfigError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl Error for ConfigError {}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    fn valid_host_control(channels: &str) -> String {
        format!(
            "[host_control]\n\
             board_vendor = {BOARD_VENDOR:?}\n\
             board_name = {BOARD_NAME:?}\n\
             chip_name = {CHIP_NAME:?}\n\
             chip_address = {CHIP_ADDRESS}\n\
             platform_component = {PLATFORM_COMPONENT:?}\n\
             {channels}"
        )
    }

    fn channel(id: &str, name: &str, pwm: u8, fan: u8, minimum: u8) -> String {
        format!(
            "[[host_control.channels]]\n\
             id = {id:?}\n\
             name = {name:?}\n\
             pwm_channel = {pwm}\n\
             fan_channel = {fan}\n\
             minimum_duty_percent = {minimum}\n"
        )
    }

    fn load_text(
        temp: &TempDir,
        name: &str,
        contents: &str,
    ) -> Result<HardwareConfig, ConfigError> {
        let path = temp.path().join(name);
        fs::write(&path, contents).unwrap();
        HardwareConfig::load(&path)
    }

    #[test]
    fn config_missing_file_is_disabled_and_valid_config_is_loaded() {
        let temp = tempfile::tempdir().unwrap();
        let missing = temp.path().join("missing.toml");
        assert_eq!(
            HardwareConfig::load(&missing).unwrap(),
            HardwareConfig::default()
        );

        let loaded = load_text(
            &temp,
            "hardware.toml",
            "nvidia_uuid = \"GPU-00000000-0000-0000-0000-000000000000\"\n\
             nvidia_pci_bus_id = \"00000000:01:00.0\"\n",
        )
        .unwrap();
        assert_eq!(
            loaded.nvidia_uuid.as_deref(),
            Some("GPU-00000000-0000-0000-0000-000000000000")
        );
        assert_eq!(
            loaded.nvidia_pci_bus_id.as_deref(),
            Some("00000000:01:00.0")
        );
        assert!(loaded.host_control.is_none());
    }

    #[test]
    fn config_rejects_unknown_empty_overlong_and_pci_without_uuid_with_bounded_errors() {
        let temp = tempfile::tempdir().unwrap();
        for (index, contents) in [
            "unknown = true".to_owned(),
            "nvidia_uuid = \"\"".to_owned(),
            format!("nvidia_uuid = {:?}", "x".repeat(MAX_IDENTITY_BYTES + 1)),
            "nvidia_pci_bus_id = \"00000000:01:00.0\"".to_owned(),
            "#".repeat(MAX_CONFIG_BYTES + 1),
        ]
        .into_iter()
        .enumerate()
        {
            let error = load_text(&temp, &format!("invalid-{index}.toml"), &contents).unwrap_err();
            assert!(error.to_string().len() <= MAX_CONFIG_ERROR_BYTES);
        }
    }

    #[test]
    fn valid_host_control_loads_candidates_and_converts_capabilities() {
        let temp = tempfile::tempdir().unwrap();
        let channels = format!(
            "{}{}",
            channel("front-intake", "Front intake candidate", 3, 3, 30),
            channel("rear-exhaust", "Rear exhaust candidate", 4, 4, 45)
        );
        let config = load_text(&temp, "host.toml", &valid_host_control(&channels)).unwrap();
        let host = config.host_control.unwrap();

        assert_eq!(host.channels.len(), 2);
        assert_eq!(
            host.capabilities(),
            vec![
                HostChannelCapability {
                    channel_id: ChannelId::new("front-intake"),
                    name: "Front intake candidate".into(),
                    minimum_duty_percent: 30,
                },
                HostChannelCapability {
                    channel_id: ChannelId::new("rear-exhaust"),
                    name: "Rear exhaust candidate".into(),
                    minimum_duty_percent: 45,
                },
            ]
        );
    }

    #[test]
    fn host_control_requires_every_exact_compiled_identity() {
        let temp = tempfile::tempdir().unwrap();
        let base = valid_host_control(&channel("candidate", "Candidate", 3, 3, 30));
        for (index, (from, to)) in [
            (BOARD_VENDOR.to_owned(), "Other vendor".to_owned()),
            (BOARD_NAME.to_owned(), "Other board".to_owned()),
            (CHIP_NAME.to_owned(), "it8688".to_owned()),
            (CHIP_ADDRESS.to_string(), (CHIP_ADDRESS + 1).to_string()),
            (PLATFORM_COMPONENT.to_owned(), "it87.2625".to_owned()),
        ]
        .into_iter()
        .enumerate()
        {
            let invalid = base.replacen(&from, &to, 1);
            assert!(load_text(&temp, &format!("identity-{index}.toml"), &invalid).is_err());
        }
    }

    #[test]
    fn host_control_rejects_missing_excess_and_unknown_channel_tables() {
        let temp = tempfile::tempdir().unwrap();
        assert!(load_text(&temp, "missing.toml", &valid_host_control("")).is_err());
        assert!(load_text(&temp, "empty.toml", &valid_host_control("channels = []\n"),).is_err());

        let three = format!(
            "{}{}{}",
            channel("one", "One", 3, 3, 30),
            channel("two", "Two", 4, 4, 30),
            channel("three", "Three", 3, 3, 30)
        );
        assert!(load_text(&temp, "three.toml", &valid_host_control(&three)).is_err());

        let unknown = format!("{}future = true\n", channel("one", "One", 3, 3, 30));
        assert!(load_text(&temp, "unknown.toml", &valid_host_control(&unknown)).is_err());

        let platform_line = format!("platform_component = {PLATFORM_COMPONENT:?}\n");
        let table_unknown = valid_host_control(&channel("one", "One", 3, 3, 30)).replacen(
            &platform_line,
            &format!("{platform_line}future = true\n"),
            1,
        );
        assert!(load_text(&temp, "table-unknown.toml", &table_unknown).is_err());
    }

    #[test]
    fn host_control_rejects_invalid_labels_mappings_and_duty_bounds() {
        let temp = tempfile::tempdir().unwrap();
        let invalid_channels = [
            channel("", "Candidate", 3, 3, 30),
            channel(" candidate", "Candidate", 3, 3, 30),
            channel(&"x".repeat(MAX_CHANNEL_ID_BYTES + 1), "Candidate", 3, 3, 30),
            channel("candidate", "", 3, 3, 30),
            channel(
                "candidate",
                &"x".repeat(MAX_CHANNEL_NAME_BYTES + 1),
                3,
                3,
                30,
            ),
            channel("candidate", "Candidate", 3, 4, 30),
            channel("candidate", "Candidate", 2, 2, 30),
            channel("candidate", "Candidate", 3, 3, 29),
            channel("candidate", "Candidate", 3, 3, 101),
        ];
        for (index, invalid) in invalid_channels.into_iter().enumerate() {
            assert!(
                load_text(
                    &temp,
                    &format!("channel-{index}.toml"),
                    &valid_host_control(&invalid),
                )
                .is_err()
            );
        }
    }

    #[test]
    fn host_control_rejects_duplicate_ids_names_and_mappings() {
        let temp = tempfile::tempdir().unwrap();
        let duplicates = [
            format!(
                "{}{}",
                channel("same", "One", 3, 3, 30),
                channel("same", "Two", 4, 4, 30)
            ),
            format!(
                "{}{}",
                channel("one", "Same", 3, 3, 30),
                channel("two", "Same", 4, 4, 30)
            ),
            format!(
                "{}{}",
                channel("one", "One", 3, 3, 30),
                channel("two", "Two", 3, 3, 30)
            ),
            format!(
                "{}{}",
                channel("one", "One", 3, 3, 30),
                channel("two", "Two", 3, 4, 30)
            ),
            format!(
                "{}{}",
                channel("one", "One", 3, 3, 30),
                channel("two", "Two", 4, 3, 30)
            ),
        ];
        for (index, duplicate) in duplicates.into_iter().enumerate() {
            assert!(
                load_text(
                    &temp,
                    &format!("duplicate-{index}.toml"),
                    &valid_host_control(&duplicate),
                )
                .is_err()
            );
        }
    }

    #[test]
    fn production_metadata_requires_regular_root_owned_non_writable_file() {
        let secure = ConfigFileMetadata {
            is_regular_file: true,
            owner_uid: 0,
            mode: 0o100_640,
        };
        assert_eq!(validate_production_metadata(secure), Ok(()));

        for invalid in [
            ConfigFileMetadata {
                is_regular_file: false,
                ..secure
            },
            ConfigFileMetadata {
                owner_uid: 1000,
                ..secure
            },
            ConfigFileMetadata {
                mode: 0o100_660,
                ..secure
            },
            ConfigFileMetadata {
                mode: 0o100_646,
                ..secure
            },
        ] {
            let error = validate_production_metadata(invalid).unwrap_err();
            assert!(error.to_string().len() <= MAX_CONFIG_ERROR_BYTES);
        }
    }
}
