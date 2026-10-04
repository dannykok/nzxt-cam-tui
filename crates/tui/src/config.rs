use std::{env, error::Error, fmt, fs, io, io::Write, path::PathBuf};

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct AppConfig {
    pub confirm_apply: bool,
    pub default_profiles: Vec<DefaultProfile>,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            confirm_apply: true,
            default_profiles: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DefaultProfile {
    pub device_id: String,
    pub channel_id: String,
    pub profile: String,
}

#[derive(Clone, Debug)]
pub struct ConfigStore {
    path: PathBuf,
}

impl ConfigStore {
    pub fn from_environment() -> Result<Self, ConfigError> {
        let config_home = match env::var_os("XDG_CONFIG_HOME") {
            Some(path) if !path.is_empty() => PathBuf::from(path),
            _ => {
                let home = env::var_os("HOME").ok_or_else(|| {
                    ConfigError::Invalid(
                        "neither XDG_CONFIG_HOME nor HOME is set; cannot locate config.conf".into(),
                    )
                })?;
                PathBuf::from(home).join(".config")
            }
        };
        Ok(Self::at(
            config_home.join("nzxt-cam-tui").join("config.conf"),
        ))
    }

    pub fn at(path: impl Into<PathBuf>) -> Self {
        Self { path: path.into() }
    }

    #[cfg(test)]
    pub fn path(&self) -> &std::path::Path {
        &self.path
    }

    pub fn load(&self) -> Result<AppConfig, ConfigError> {
        let contents = match fs::read_to_string(&self.path) {
            Ok(contents) => contents,
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                return Ok(AppConfig::default());
            }
            Err(error) => return Err(error.into()),
        };
        let config = toml::from_str::<AppConfig>(&contents)?;
        validate_config(&config)?;
        Ok(config)
    }

    /// Update only the app's firmware-apply preference. Preserve legacy
    /// profile bindings and unknown user keys rather than rewriting a subset.
    pub fn save_confirm_apply(&self, enabled: bool) -> Result<(), ConfigError> {
        let parent = self.path.parent().ok_or_else(|| {
            ConfigError::Invalid("app config path has no parent directory".into())
        })?;
        fs::create_dir_all(parent)?;
        let mut settings = match fs::symlink_metadata(&self.path) {
            Ok(metadata) if !metadata.is_file() => {
                return Err(ConfigError::Invalid(
                    "app config is not a regular file".into(),
                ));
            }
            Ok(_) => toml::from_str::<toml::Table>(&fs::read_to_string(&self.path)?)?,
            Err(error) if error.kind() == io::ErrorKind::NotFound => toml::Table::new(),
            Err(error) => return Err(error.into()),
        };
        settings.insert("confirm_apply".into(), toml::Value::Boolean(enabled));
        let serialized = toml::to_string_pretty(&settings).map_err(ConfigError::Serialize)?;
        // A unique same-directory file keeps rename atomic and cannot leave a
        // fixed stale .tmp path that blocks every future Settings save.
        let mut temp = tempfile::NamedTempFile::new_in(parent)?;
        temp.write_all(serialized.as_bytes())?;
        temp.as_file().sync_all()?;
        temp.persist(&self.path)
            .map_err(|error| ConfigError::Io(error.error))?;
        #[cfg(unix)]
        fs::File::open(parent)?.sync_all()?;
        Ok(())
    }
}

fn validate_config(config: &AppConfig) -> Result<(), ConfigError> {
    for saved in &config.default_profiles {
        if saved.device_id.trim().is_empty()
            || saved.channel_id.trim().is_empty()
            || saved.profile.trim().is_empty()
        {
            return Err(ConfigError::Invalid(
                "default profile entries require device_id, channel_id, and profile".into(),
            ));
        }
    }
    Ok(())
}

#[derive(Debug)]
pub enum ConfigError {
    Io(io::Error),
    Parse(toml::de::Error),
    Serialize(toml::ser::Error),
    Invalid(String),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "{error}"),
            Self::Parse(error) => write!(formatter, "invalid TOML: {error}"),
            Self::Serialize(error) => write!(formatter, "cannot write TOML: {error}"),
            Self::Invalid(message) => formatter.write_str(message),
        }
    }
}

impl Error for ConfigError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Parse(error) => Some(error),
            Self::Serialize(error) => Some(error),
            Self::Invalid(_) => None,
        }
    }
}

impl From<io::Error> for ConfigError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

impl From<toml::de::Error> for ConfigError {
    fn from(error: toml::de::Error) -> Self {
        Self::Parse(error)
    }
}

#[cfg(test)]
mod tests {
    use std::time::SystemTime;

    use super::*;

    fn temporary_store() -> (PathBuf, ConfigStore) {
        let nonce = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let directory = env::temp_dir().join(format!(
            "nzxt-cam-tui-config-{}-{nonce}",
            std::process::id()
        ));
        let path = directory.join("config.conf");
        (directory, ConfigStore::at(path))
    }

    #[test]
    fn missing_file_uses_safe_confirmation_default() {
        let (directory, store) = temporary_store();

        let config = store.load().unwrap();

        assert!(config.confirm_apply);
        assert!(config.default_profiles.is_empty());
        assert!(!directory.exists());
    }

    #[test]
    fn parses_confirmation_opt_out() {
        let (directory, store) = temporary_store();
        fs::create_dir_all(store.path().parent().unwrap()).unwrap();
        fs::write(store.path(), "confirm_apply = false\n").unwrap();

        let config = store.load().unwrap();

        assert!(!config.confirm_apply);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn settings_write_keeps_legacy_bindings_and_unknown_keys() {
        let (directory, store) = temporary_store();
        fs::create_dir_all(store.path().parent().unwrap()).unwrap();
        fs::write(
            store.path(),
            "confirm_apply = true\ncustom_setting = 'keep'\n[[default_profiles]]\ndevice_id = 'cooler'\nchannel_id = 'pump'\nprofile = 'Silent'\n",
        )
        .unwrap();

        let stale_temp = store.path().with_extension("conf.tmp");
        fs::write(&stale_temp, "interrupted old save").unwrap();
        store.save_confirm_apply(false).unwrap();
        assert!(!store.load().unwrap().confirm_apply);
        assert_eq!(
            fs::read_to_string(stale_temp).unwrap(),
            "interrupted old save"
        );
        let raw = fs::read_to_string(store.path()).unwrap();
        assert!(raw.contains("custom_setting = \"keep\""));
        assert!(raw.contains("profile = \"Silent\""));
        assert_eq!(store.load().unwrap().default_profiles[0].profile, "Silent");
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn settings_write_creates_config_and_refuses_symlink() {
        let (directory, store) = temporary_store();
        store.save_confirm_apply(false).unwrap();
        assert!(!store.load().unwrap().confirm_apply);
        let target = store.path().with_file_name("other.conf");
        fs::rename(store.path(), &target).unwrap();
        #[cfg(unix)]
        {
            std::os::unix::fs::symlink(&target, store.path()).unwrap();
            assert!(store.save_confirm_apply(true).is_err());
            assert!(
                !fs::read_to_string(&target)
                    .unwrap()
                    .contains("confirm_apply = true")
            );
        }
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn legacy_defaults_are_read_only_and_preserved() {
        let (directory, store) = temporary_store();
        fs::create_dir_all(store.path().parent().unwrap()).unwrap();
        let raw = "confirm_apply = false\n[[default_profiles]]\ndevice_id = 'cooler'\nchannel_id = 'pump'\nprofile = 'Silent'\n";
        fs::write(store.path(), raw).unwrap();
        let config = store.load().unwrap();
        assert!(!config.confirm_apply);
        assert_eq!(
            config.default_profiles,
            vec![DefaultProfile {
                device_id: "cooler".into(),
                channel_id: "pump".into(),
                profile: "Silent".into(),
            }]
        );
        assert_eq!(fs::read_to_string(store.path()).unwrap(), raw);
        fs::remove_dir_all(directory).unwrap();
    }
}
