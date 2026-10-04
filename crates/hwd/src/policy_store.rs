//! Fixed, durable service intent. An enabled record is authorization to resume only
//! with the exact configuration that committed it; Disabled is a durable tombstone.
use crate::{
    config::HostControlConfig,
    hardware::{HardwareError, HardwareErrorKind},
};
use nzxt_cam_core::HostControlPolicy;
use serde::{Deserialize, Serialize};
use std::{
    fs::{self, OpenOptions},
    io::{self, Read, Write},
    os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt},
    path::{Path, PathBuf},
};

pub const ACTIVE_POLICY_PATH: &str = "/var/lib/nzxt-cam/active-policy.json";
const TEMP: &str = "active-policy.json.tmp";
const LIMIT: u64 = 32 * 1024;

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ConfigIdentity {
    pub host_control: HostControlConfig,
    pub nvidia_uuid: Option<String>,
    pub nvidia_pci_bus_id: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(tag = "state", rename_all = "snake_case", deny_unknown_fields)]
pub enum SavedPolicy {
    Enabled {
        version: u8,
        complete_policy: HostControlPolicy,
        exact_config_identity: ConfigIdentity,
    },
    Disabled {
        version: u8,
    },
}
impl SavedPolicy {
    pub fn validate_version(&self) -> Result<(), HardwareError> {
        let version = match self {
            Self::Enabled { version, .. } | Self::Disabled { version } => *version,
        };
        if version == 1 {
            Ok(())
        } else {
            Err(fail("unsupported saved policy version"))
        }
    }
}

pub trait PolicyStore: Send {
    fn load(&mut self) -> Result<Option<SavedPolicy>, HardwareError>;
    fn persist(&mut self, policy: &SavedPolicy) -> Result<(), HardwareError>;
}
fn fail(message: impl Into<String>) -> HardwareError {
    HardwareError::with_kind(HardwareErrorKind::RestoreRequired, message)
}
fn io_failure(action: &str, error: io::Error) -> HardwareError {
    fail(format!("{action}: {error}"))
}

pub struct FilePolicyStore {
    path: PathBuf,
    enforce_root: bool,
    #[cfg(test)]
    fail_after: Option<&'static str>,
}
impl FilePolicyStore {
    pub fn new() -> Self {
        Self {
            path: ACTIVE_POLICY_PATH.into(),
            enforce_root: true,
            #[cfg(test)]
            fail_after: None,
        }
    }
    #[cfg(test)]
    pub(crate) fn at(path: PathBuf) -> Self {
        Self {
            path,
            enforce_root: false,
            fail_after: None,
        }
    }
    #[cfg(test)]
    pub(crate) fn failing_after(mut self, point: &'static str) -> Self {
        self.fail_after = Some(point);
        self
    }
    fn fault(&self, _point: &'static str) -> Result<(), HardwareError> {
        #[cfg(test)]
        if self.fail_after == Some(_point) {
            return Err(fail(format!("injected policy {_point} fault")));
        }
        Ok(())
    }
    fn parent(&self) -> Result<&Path, HardwareError> {
        self.path
            .parent()
            .ok_or_else(|| fail("policy directory missing"))
    }
    fn validate_parent(&self, allow_absent: bool) -> Result<bool, HardwareError> {
        let parent = self.parent()?;
        let meta = match fs::symlink_metadata(parent) {
            Ok(meta) => meta,
            Err(error) if allow_absent && error.kind() == io::ErrorKind::NotFound => {
                return Ok(false);
            }
            Err(error) => return Err(io_failure("inspect policy directory", error)),
        };
        if !meta.is_dir()
            || (self.enforce_root && (meta.uid() != 0 || meta.mode() & 0o7777 != 0o700))
        {
            return Err(fail(
                "policy directory must be a private root-owned directory",
            ));
        }
        Ok(true)
    }
    fn temp(&self) -> Result<PathBuf, HardwareError> {
        Ok(self.parent()?.join(TEMP))
    }
    fn no_temp(&self) -> Result<(), HardwareError> {
        match fs::symlink_metadata(self.temp()?) {
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(io_failure("inspect policy temporary state", error)),
            Ok(_) => Err(fail("unresolved policy temporary state")),
        }
    }
    fn validate_file(&self, file: &fs::File) -> Result<(), HardwareError> {
        let m = file
            .metadata()
            .map_err(|e| io_failure("inspect policy file", e))?;
        if !m.is_file()
            || m.nlink() != 1
            || m.mode() & 0o7777 != 0o600
            || (self.enforce_root && m.uid() != 0)
        {
            Err(fail(
                "policy file must be root-owned regular single-link mode 0600",
            ))
        } else {
            Ok(())
        }
    }
}
impl Default for FilePolicyStore {
    fn default() -> Self {
        Self::new()
    }
}
impl PolicyStore for FilePolicyStore {
    fn load(&mut self) -> Result<Option<SavedPolicy>, HardwareError> {
        if !self.validate_parent(true)? {
            return Ok(None);
        }
        self.no_temp()?;
        let file = match OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(&self.path)
        {
            Ok(f) => f,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(None),
            Err(e) => return Err(io_failure("open policy", e)),
        };
        self.validate_file(&file)?;
        let mut bytes = Vec::new();
        file.take(LIMIT + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| io_failure("read policy", e))?;
        if bytes.len() as u64 > LIMIT {
            return Err(fail("policy exceeds 32KiB"));
        }
        let saved: SavedPolicy = serde_json::from_slice(&bytes)
            .map_err(|e| fail(format!("invalid saved policy: {e}")))?;
        saved.validate_version()?;
        Ok(Some(saved))
    }
    fn persist(&mut self, policy: &SavedPolicy) -> Result<(), HardwareError> {
        policy.validate_version()?;
        self.validate_parent(false)?;
        self.no_temp()?;
        // Refuse replacing malformed/unsafe state; callers must resolve it explicitly.
        self.load()?;
        let bytes =
            serde_json::to_vec(policy).map_err(|e| fail(format!("serialize policy: {e}")))?;
        if bytes.len() as u64 > LIMIT {
            return Err(fail("policy exceeds 32KiB"));
        }
        let temp = self.temp()?;
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(&temp)
            .map_err(|e| io_failure("create policy temporary file", e))?;
        file.set_permissions(fs::Permissions::from_mode(0o600))
            .map_err(|e| io_failure("chmod policy", e))?;
        self.validate_file(&file)?;
        self.fault("before_write")?;
        file.write_all(&bytes)
            .map_err(|e| io_failure("write policy", e))?;
        self.fault("after_write")?;
        file.sync_all().map_err(|e| io_failure("sync policy", e))?;
        self.fault("after_file_sync")?;
        drop(file);
        self.fault("before_rename")?;
        fs::rename(&temp, &self.path).map_err(|e| io_failure("install policy", e))?;
        self.fault("after_rename")?;
        OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(self.parent()?)
            .and_then(|f| f.sync_all())
            .map_err(|e| io_failure("sync policy directory", e))?;
        self.fault("after_parent_sync")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn tombstone_round_trip_and_rejects_uncertain_temp() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let mut store = FilePolicyStore::at(path);
        store
            .persist(&SavedPolicy::Disabled { version: 1 })
            .unwrap();
        assert_eq!(
            store.load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
        fs::write(dir.path().join(TEMP), b"uncertain").unwrap();
        assert!(store.load().is_err());
        assert!(
            store
                .persist(&SavedPolicy::Disabled { version: 1 })
                .is_err()
        );
    }
    #[test]
    fn failures_at_each_install_boundary_never_discard_uncertainty() {
        for point in [
            "before_write",
            "after_write",
            "after_file_sync",
            "before_rename",
            "after_rename",
            "after_parent_sync",
        ] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("active-policy.json");
            let mut store = FilePolicyStore::at(path.clone()).failing_after(point);
            assert!(
                store
                    .persist(&SavedPolicy::Disabled { version: 1 })
                    .is_err(),
                "{point}"
            );
            if point == "after_rename" || point == "after_parent_sync" {
                assert_eq!(
                    FilePolicyStore::at(path).load().unwrap(),
                    Some(SavedPolicy::Disabled { version: 1 })
                );
            } else {
                assert!(dir.path().join(TEMP).exists());
                assert!(FilePolicyStore::at(path).load().is_err());
            }
        }
    }
    #[test]
    fn bounded_and_nofollow_even_for_fifo() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let mut store = FilePolicyStore::at(path.clone());
        fs::write(&path, vec![b'X'; LIMIT as usize + 1]).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        assert!(store.load().is_err());
        fs::remove_file(&path).unwrap();
        std::os::unix::fs::symlink(dir.path().join("absent"), &path).unwrap();
        assert!(store.load().is_err());
        fs::remove_file(&path).unwrap();
        use std::os::unix::ffi::OsStrExt;
        let path_c = std::ffi::CString::new(path.as_os_str().as_bytes()).unwrap();
        assert_eq!(unsafe { libc::mkfifo(path_c.as_ptr(), 0o600) }, 0);
        assert!(store.load().is_err());
    }
    #[test]
    fn rejects_hardlinks_unknown_versions_and_extra_fields() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let mut store = FilePolicyStore::at(path.clone());
        store
            .persist(&SavedPolicy::Disabled { version: 1 })
            .unwrap();
        let second = dir.path().join("second-link");
        fs::hard_link(&path, &second).unwrap();
        assert!(store.load().is_err());
        fs::remove_file(second).unwrap();
        for invalid in [
            r#"{"state":"disabled","version":2}"#,
            r#"{"state":"disabled","version":1,"surprise":true}"#,
        ] {
            fs::write(&path, invalid).unwrap();
            assert!(store.load().is_err());
        }
    }
    #[test]
    fn rejects_unsafe_file_and_parent() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let mut store = FilePolicyStore::at(path.clone());
        fs::write(&path, b"{}").unwrap();
        assert!(store.load().is_err());
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        assert!(store.load().is_err());
        fs::remove_file(&path).unwrap();
        std::os::unix::fs::symlink(dir.path(), dir.path().join("link")).unwrap();
        assert!(
            FilePolicyStore::at(dir.path().join("link/active-policy.json"))
                .load()
                .is_err()
        );
    }
}
