//! Durable, service-owned monitoring intent, independent of the host policy store.
//!
//! This is authorization, not evidence of hardware state. A single service writer must
//! durably mark the *exact* target InFlightUnknown before attempting ANY firmware write.
//! A failed/uncertain write or crash leaves it blocked, including across restarts.
//! Only definite confirmed success permits `mark_ready`. On startup only opted-in,
//! auto-resume, Ready targets may be considered for replay; missing/offline targets
//! require no state transition.
//! `auto_resume` never triggers a write here. No hardware calls are made by this module.
//!
//! Provision /var/lib/nzxt-cam as root-owned mode 0700 before using the production
//! store. A single writer is required: this module does not coordinate processes.
use crate::hardware::{HardwareError, HardwareErrorKind};
use nzxt_cam_core::{ChannelId, CurvePoint, DeviceId, KrakenDisplayMode};
use serde::{Deserialize, Serialize};
use std::{
    collections::HashSet,
    fs::{self, OpenOptions},
    io::{self, Read, Write},
    os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt},
    path::{Path, PathBuf},
};

pub const MONITOR_INTENT_PATH: &str = "/var/lib/nzxt-cam/monitor-intent.json";
const TEMP: &str = "monitor-intent.json.tmp";
const LIMIT: u64 = 64 * 1024;
const MAX_TARGETS: usize = nzxt_cam_protocol::MAX_MONITORING_FIRMWARE_CURVES + 1; // plus one display
const MAX_POINTS: usize = 64;
const MAX_ID_BYTES: usize = 256;

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum WriteState {
    Ready,
    InFlightUnknown,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MonitorTarget {
    AioCurve {
        device_id: DeviceId,
        channel_id: ChannelId,
        points: Vec<CurvePoint>,
    },
    Display {
        device_id: DeviceId,
        mode: KrakenDisplayMode,
    },
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SavedTarget {
    pub target: MonitorTarget,
    pub state: WriteState,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SavedMonitorIntent {
    pub version: u8,
    pub opted_in: bool,
    /// Authorizes consideration of Ready targets on *future service startups* only.
    pub auto_resume: bool,
    pub targets: Vec<SavedTarget>,
}

impl SavedMonitorIntent {
    pub fn validate(&self) -> Result<(), HardwareError> {
        if self.version != 1 {
            return Err(fail("unsupported monitor intent version"));
        }
        if self.auto_resume && !self.opted_in {
            return Err(fail("auto_resume requires opt-in"));
        }
        if self.targets.len() > MAX_TARGETS {
            return Err(fail("too many monitor targets"));
        }
        let mut keys = HashSet::new();
        for saved in &self.targets {
            let key = target_key(&saved.target)?;
            if !keys.insert(key) {
                return Err(fail("duplicate monitor target"));
            }
        }
        Ok(())
    }
}

fn identity(value: &str) -> Result<(), HardwareError> {
    if value.len() > MAX_ID_BYTES || value.trim().is_empty() || value.chars().any(char::is_control)
    {
        return Err(fail("invalid monitor target identity"));
    }
    Ok(())
}

// Key excludes the payload: a different curve or display mode for the same
// physical target must not bypass a blocked write.
fn target_key(target: &MonitorTarget) -> Result<(u8, &str, Option<&str>), HardwareError> {
    match target {
        MonitorTarget::AioCurve {
            device_id,
            channel_id,
            points,
        } => {
            identity(&device_id.0)?;
            identity(&channel_id.0)?;
            if points.is_empty() || points.len() > MAX_POINTS {
                return Err(fail("invalid monitor curve point count"));
            }
            Ok((0, &device_id.0, Some(&channel_id.0)))
        }
        MonitorTarget::Display { device_id, .. } => {
            identity(&device_id.0)?;
            Ok((1, &device_id.0, None))
        }
    }
}

fn fail(message: impl Into<String>) -> HardwareError {
    HardwareError::with_kind(HardwareErrorKind::RestoreRequired, message)
}
fn io_failure(action: &str, error: io::Error) -> HardwareError {
    fail(format!("{action}: {error}"))
}

pub struct FileMonitorIntentStore {
    path: PathBuf,
    enforce_root: bool,
    #[cfg(test)]
    fail_after: Option<&'static str>,
}
impl Default for FileMonitorIntentStore {
    fn default() -> Self {
        Self::new()
    }
}
impl FileMonitorIntentStore {
    pub fn new() -> Self {
        Self {
            path: MONITOR_INTENT_PATH.into(),
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
    fn failing_after(mut self, point: &'static str) -> Self {
        self.fail_after = Some(point);
        self
    }
    fn fault(&self, _point: &'static str) -> Result<(), HardwareError> {
        #[cfg(test)]
        if self.fail_after == Some(_point) {
            return Err(fail(format!("injected monitor intent {_point} fault")));
        }
        Ok(())
    }
    fn parent(&self) -> Result<&Path, HardwareError> {
        self.path
            .parent()
            .ok_or_else(|| fail("monitor intent directory missing"))
    }
    fn validate_parent(&self, allow_absent: bool) -> Result<bool, HardwareError> {
        let parent = self.parent()?;
        // Check each component, not just the leaf directory, against symlink traversal.
        let mut ancestors: Vec<_> = parent
            .ancestors()
            .filter(|p| !p.as_os_str().is_empty())
            .collect();
        ancestors.reverse();
        for ancestor in ancestors {
            let meta = match fs::symlink_metadata(ancestor) {
                Ok(meta) => meta,
                Err(error)
                    if allow_absent
                        && ancestor == parent
                        && error.kind() == io::ErrorKind::NotFound =>
                {
                    return Ok(false);
                }
                Err(error) => return Err(io_failure("inspect monitor intent directory", error)),
            };
            if !meta.is_dir() {
                return Err(fail(
                    "monitor intent directory path must not contain symlinks",
                ));
            }
            if ancestor == parent
                && (meta.mode() & 0o7777 != 0o700 || (self.enforce_root && meta.uid() != 0))
            {
                return Err(fail(
                    "monitor intent directory must be root-owned mode 0700",
                ));
            }
        }
        Ok(true)
    }
    fn temp(&self) -> Result<PathBuf, HardwareError> {
        Ok(self.parent()?.join(TEMP))
    }
    fn no_temp(&self) -> Result<(), HardwareError> {
        match fs::symlink_metadata(self.temp()?) {
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(io_failure("inspect monitor intent temporary state", error)),
            Ok(_) => Err(fail("unresolved monitor intent temporary state")),
        }
    }
    fn validate_file(&self, file: &fs::File) -> Result<(), HardwareError> {
        let m = file
            .metadata()
            .map_err(|e| io_failure("inspect monitor intent file", e))?;
        if !m.is_file()
            || m.nlink() != 1
            || m.mode() & 0o7777 != 0o600
            || (self.enforce_root && m.uid() != 0)
        {
            return Err(fail(
                "monitor intent file must be root-owned regular single-link mode 0600",
            ));
        }
        Ok(())
    }
    pub fn load(&self) -> Result<Option<SavedMonitorIntent>, HardwareError> {
        if !self.validate_parent(true)? {
            return Ok(None);
        }
        self.no_temp()?;
        let file = match OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(&self.path)
        {
            Ok(file) => file,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(None),
            Err(e) => return Err(io_failure("open monitor intent", e)),
        };
        self.validate_file(&file)?;
        let mut bytes = Vec::new();
        file.take(LIMIT + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| io_failure("read monitor intent", e))?;
        if bytes.len() as u64 > LIMIT {
            return Err(fail("monitor intent exceeds 64KiB"));
        }
        let saved: SavedMonitorIntent = serde_json::from_slice(&bytes)
            .map_err(|e| fail(format!("invalid monitor intent: {e}")))?;
        saved.validate()?;
        Ok(Some(saved))
    }
    /// Save opt-in/auto-resume and Ready targets. Never clears or edits an
    /// InFlightUnknown target; only confirmed success permits `mark_ready`.
    pub fn persist(&mut self, intent: &SavedMonitorIntent) -> Result<(), HardwareError> {
        intent.validate()?;
        let old = self.load()?;
        for added in &intent.targets {
            if added.state == WriteState::InFlightUnknown
                && !old
                    .as_ref()
                    .is_some_and(|previous| previous.targets.iter().any(|item| item == added))
            {
                return Err(fail("use mark_in_flight for a write transition"));
            }
        }
        if let Some(old) = old {
            for saved in &old.targets {
                let key = target_key(&saved.target)?;
                let matching = intent
                    .targets
                    .iter()
                    .find(|item| target_key(&item.target).ok() == Some(key));
                if saved.state == WriteState::InFlightUnknown && matching != Some(saved) {
                    return Err(fail("cannot change or remove an in-flight monitor target"));
                }
                if saved.state == WriteState::Ready
                    && matching.is_some_and(|item| item.state != WriteState::Ready)
                {
                    return Err(fail("use mark_in_flight for a write transition"));
                }
            }
        }
        self.write(intent)
    }
    /// Durably block the exact target *before* a firmware write. No write may
    /// proceed if this call returns an error, including a post-rename error.
    pub fn mark_in_flight(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
        self.transition(target, WriteState::Ready, WriteState::InFlightUnknown)
    }
    /// Call only after a definite confirmed successful write of this exact target.
    pub fn mark_ready(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
        self.transition(target, WriteState::InFlightUnknown, WriteState::Ready)
    }
    fn transition(
        &mut self,
        target: &MonitorTarget,
        from: WriteState,
        to: WriteState,
    ) -> Result<(), HardwareError> {
        let mut intent = self.load()?.ok_or_else(|| fail("monitor intent absent"))?;
        if !intent.opted_in {
            return Err(fail("monitor intent not opted in"));
        }
        let saved = intent
            .targets
            .iter_mut()
            .find(|item| &item.target == target)
            .ok_or_else(|| fail("exact monitor target not saved"))?;
        if saved.state != from {
            return Err(fail("invalid monitor target write transition"));
        }
        saved.state = to;
        self.write(&intent)
    }
    fn write(&self, intent: &SavedMonitorIntent) -> Result<(), HardwareError> {
        intent.validate()?;
        self.validate_parent(false)?;
        self.no_temp()?;
        // Refuse replacing an unsafe/malformed file even on a transition.
        self.load()?;
        let bytes = serde_json::to_vec(intent)
            .map_err(|e| fail(format!("serialize monitor intent: {e}")))?;
        if bytes.len() as u64 > LIMIT {
            return Err(fail("monitor intent exceeds 64KiB"));
        }
        let temp = self.temp()?;
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(&temp)
            .map_err(|e| io_failure("create monitor intent temporary file", e))?;
        file.set_permissions(fs::Permissions::from_mode(0o600))
            .map_err(|e| io_failure("chmod monitor intent", e))?;
        self.validate_file(&file)?;
        self.fault("before_write")?;
        file.write_all(&bytes)
            .map_err(|e| io_failure("write monitor intent", e))?;
        self.fault("after_write")?;
        file.sync_all()
            .map_err(|e| io_failure("sync monitor intent", e))?;
        self.fault("after_file_sync")?;
        drop(file);
        self.fault("before_rename")?;
        fs::rename(&temp, &self.path).map_err(|e| io_failure("install monitor intent", e))?;
        self.fault("after_rename")?;
        OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(self.parent()?)
            .and_then(|f| f.sync_all())
            .map_err(|e| io_failure("sync monitor intent directory", e))?;
        self.fault("after_parent_sync")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::{ffi::OsStrExt, fs::symlink};

    fn curve(device: &str, channel: &str) -> MonitorTarget {
        MonitorTarget::AioCurve {
            device_id: DeviceId::new(device),
            channel_id: ChannelId::new(channel),
            points: vec![
                CurvePoint {
                    temperature: 20,
                    duty: 45,
                },
                CurvePoint {
                    temperature: 21,
                    duty: 50,
                },
            ],
        }
    }
    fn display(device: &str) -> MonitorTarget {
        MonitorTarget::Display {
            device_id: DeviceId::new(device),
            mode: KrakenDisplayMode::CpuLiquid,
        }
    }
    fn saved(targets: Vec<MonitorTarget>) -> SavedMonitorIntent {
        SavedMonitorIntent {
            version: 1,
            opted_in: true,
            auto_resume: true,
            targets: targets
                .into_iter()
                .map(|target| SavedTarget {
                    target,
                    state: WriteState::Ready,
                })
                .collect(),
        }
    }
    fn store(dir: &tempfile::TempDir) -> FileMonitorIntentStore {
        fs::set_permissions(dir.path(), fs::Permissions::from_mode(0o700)).unwrap();
        FileMonitorIntentStore::at(dir.path().join("monitor-intent.json"))
    }

    #[test]
    fn round_trip_exact_targets_and_independent_host_policy() {
        let dir = tempfile::tempdir().unwrap();
        let policy = dir.path().join("active-policy.json");
        fs::write(&policy, b"host policy must stay unchanged").unwrap();
        let mut store = store(&dir);
        assert_eq!(store.load().unwrap(), None);
        let record = saved(vec![
            curve("aio-a", "pump"),
            curve("aio-a", "fan"),
            display("aio-a"),
        ]);
        store.persist(&record).unwrap();
        assert_eq!(store.load().unwrap(), Some(record.clone()));
        let m = fs::metadata(dir.path().join("monitor-intent.json")).unwrap();
        assert_eq!(m.mode() & 0o7777, 0o600);
        assert_eq!(m.nlink(), 1);
        assert_eq!(
            fs::read(&policy).unwrap(),
            b"host policy must stay unchanged"
        );

        let target = curve("aio-a", "pump");
        store.mark_in_flight(&target).unwrap();
        assert_eq!(
            store.load().unwrap().unwrap().targets[0].state,
            WriteState::InFlightUnknown
        );
        let restarted = FileMonitorIntentStore::at(dir.path().join("monitor-intent.json"));
        assert!(restarted.load().unwrap().unwrap().auto_resume);
        assert_eq!(
            restarted.load().unwrap().unwrap().targets[0].state,
            WriteState::InFlightUnknown
        );
        assert!(store.mark_in_flight(&target).is_err());
        assert!(store.mark_ready(&curve("aio-a", "fan")).is_err());
        assert!(store.mark_ready(&curve("aio-b", "pump")).is_err());
        let mut different = target.clone();
        if let MonitorTarget::AioCurve { points, .. } = &mut different {
            points[0].duty = 99;
        }
        assert!(store.mark_ready(&different).is_err());
        store.mark_ready(&target).unwrap(); // Only the caller can attest definite success.
        assert_eq!(
            store.load().unwrap().unwrap().targets[0].state,
            WriteState::Ready
        );
        assert!(store.mark_ready(&target).is_err());
        assert_eq!(
            fs::read(&policy).unwrap(),
            b"host policy must stay unchanged"
        );
    }

    #[test]
    fn blocked_target_cannot_be_replaced_removed_or_reset_by_persist() {
        let dir = tempfile::tempdir().unwrap();
        let mut store = store(&dir);
        let target = display("lcd");
        store
            .persist(&saved(vec![target.clone(), curve("aio", "fan")]))
            .unwrap();
        store.mark_in_flight(&target).unwrap();
        let original = fs::read(dir.path().join("monitor-intent.json")).unwrap();
        let mut record = store.load().unwrap().unwrap();
        record.targets[0].state = WriteState::Ready;
        assert!(store.persist(&record).is_err());
        record.targets.remove(0);
        assert!(store.persist(&record).is_err());
        record.targets.insert(
            0,
            SavedTarget {
                target: MonitorTarget::Display {
                    device_id: DeviceId::new("lcd"),
                    mode: KrakenDisplayMode::Gpu,
                },
                state: WriteState::InFlightUnknown,
            },
        );
        assert!(store.persist(&record).is_err());
        record.targets[0].target = display("other");
        assert!(store.persist(&record).is_err());
        assert_eq!(
            fs::read(dir.path().join("monitor-intent.json")).unwrap(),
            original
        );
        assert!(!dir.path().join(TEMP).exists());
        store.mark_ready(&target).unwrap(); // Caller attests definite confirmed success.
        assert!(store.mark_ready(&target).is_err());
        record = store.load().unwrap().unwrap();
        record.auto_resume = false;
        record.opted_in = false;
        store.persist(&record).unwrap();
        assert!(store.mark_in_flight(&target).is_err());
    }

    #[test]
    fn validates_versions_enums_counts_identities_and_unknown_fields() {
        let dir = tempfile::tempdir().unwrap();
        let mut store = store(&dir);
        let path = dir.path().join("monitor-intent.json");
        let mut record = saved(vec![curve("aio", "fan")]);
        record.version = 2;
        assert!(store.persist(&record).is_err());
        record.version = 1;
        record.auto_resume = true;
        record.opted_in = false;
        assert!(store.persist(&record).is_err());
        record.opted_in = true;
        record.targets.push(record.targets[0].clone());
        assert!(store.persist(&record).is_err());
        record.targets.pop();
        record.targets = vec![
            SavedTarget {
                target: display("lcd"),
                state: WriteState::Ready,
            },
            SavedTarget {
                target: MonitorTarget::Display {
                    device_id: DeviceId::new("lcd"),
                    mode: KrakenDisplayMode::Gpu,
                },
                state: WriteState::Ready,
            },
        ];
        assert!(store.persist(&record).is_err());
        record.targets = (0..=MAX_TARGETS)
            .map(|n| SavedTarget {
                target: curve("aio", &format!("fan{n}")),
                state: WriteState::Ready,
            })
            .collect();
        assert!(store.persist(&record).is_err());
        record.targets = vec![SavedTarget {
            target: curve(" ", "fan"),
            state: WriteState::Ready,
        }];
        assert!(store.persist(&record).is_err());
        record.targets[0].target = curve("x".repeat(MAX_ID_BYTES + 1).as_str(), "fan");
        assert!(store.persist(&record).is_err());
        record.targets[0].target = curve("aio", "bad\nchannel");
        assert!(store.persist(&record).is_err());
        record.targets[0].target = curve("aio", "fan");
        record.targets[0].state = WriteState::InFlightUnknown;
        assert!(store.persist(&record).is_err());
        record.targets[0].state = WriteState::Ready;
        if let MonitorTarget::AioCurve { points, .. } = &mut record.targets[0].target {
            points.clear();
        }
        assert!(store.persist(&record).is_err());
        if let MonitorTarget::AioCurve { points, .. } = &mut record.targets[0].target {
            points.resize(
                MAX_POINTS + 1,
                CurvePoint {
                    temperature: 20,
                    duty: 40,
                },
            );
        }
        assert!(store.persist(&record).is_err());
        for bad in [
            r#"{"version":2,"opted_in":true,"auto_resume":false,"targets":[]}"#,
            r#"{"version":1,"opted_in":true,"auto_resume":false,"targets":[],"extra":0}"#,
            r#"{"version":1,"opted_in":true,"auto_resume":false,"targets":[{"target":{"kind":"display","device_id":"lcd","mode":"surprise"},"state":"ready"}]}"#,
            r#"{"version":1,"opted_in":true,"auto_resume":false,"targets":[{"target":{"kind":"display","device_id":"lcd","mode":"cpu","extra":1},"state":"ready"}]}"#,
            r#"{"version":1,"opted_in":true,"auto_resume":false,"targets":[{"target":{"kind":"display","device_id":"lcd","mode":"cpu"},"state":"unknown"}]}"#,
        ] {
            fs::write(&path, bad).unwrap();
            fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
            assert!(store.load().is_err(), "{bad}");
        }
    }

    #[test]
    fn rejects_unsafe_files_and_orphan_temp_without_touching_them() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("monitor-intent.json");
        let mut store = store(&dir);
        fs::write(&path, vec![b'X'; LIMIT as usize + 1]).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        assert!(store.load().is_err());
        assert!(store.persist(&saved(vec![])).is_err());
        fs::remove_file(&path).unwrap();
        symlink(dir.path().join("absent"), &path).unwrap();
        assert!(store.load().is_err());
        fs::remove_file(&path).unwrap();
        let c_path = std::ffi::CString::new(path.as_os_str().as_bytes()).unwrap();
        assert_eq!(unsafe { libc::mkfifo(c_path.as_ptr(), 0o600) }, 0);
        assert!(store.load().is_err());
        fs::remove_file(&path).unwrap();
        store.persist(&saved(vec![])).unwrap();
        let link = dir.path().join("second-link");
        fs::hard_link(&path, &link).unwrap();
        assert!(store.load().is_err());
        fs::remove_file(link).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o644)).unwrap();
        assert!(store.load().is_err());
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        let temp = dir.path().join(TEMP);
        symlink(&path, &temp).unwrap();
        assert!(store.load().is_err());
        assert!(store.persist(&saved(vec![])).is_err());
        assert!(
            fs::symlink_metadata(&temp)
                .unwrap()
                .file_type()
                .is_symlink()
        );
    }

    #[test]
    fn rejects_parent_symlinks_modes_and_missing_directory() {
        let dir = tempfile::tempdir().unwrap();
        let parent = dir.path().join("private");
        fs::create_dir(&parent).unwrap();
        fs::set_permissions(&parent, fs::Permissions::from_mode(0o700)).unwrap();
        let path = parent.join("monitor-intent.json");
        let mut store = FileMonitorIntentStore::at(path.clone());
        store.persist(&saved(vec![])).unwrap();
        fs::set_permissions(&parent, fs::Permissions::from_mode(0o755)).unwrap();
        assert!(store.load().is_err());
        fs::set_permissions(&parent, fs::Permissions::from_mode(0o700)).unwrap();
        symlink(&parent, dir.path().join("link")).unwrap();
        assert!(
            FileMonitorIntentStore::at(dir.path().join("link/monitor-intent.json"))
                .load()
                .is_err()
        );
        assert!(
            FileMonitorIntentStore::at(dir.path().join("link/child/monitor-intent.json"))
                .load()
                .is_err()
        );
        let mut absent = FileMonitorIntentStore::at(dir.path().join("missing/monitor-intent.json"));
        assert_eq!(absent.load().unwrap(), None);
        assert!(absent.persist(&saved(vec![])).is_err());
    }

    #[test]
    fn failed_ready_transition_never_silently_rearms_an_uncertain_write() {
        for point in [
            "before_write",
            "after_write",
            "after_file_sync",
            "before_rename",
            "after_rename",
            "after_parent_sync",
        ] {
            let dir = tempfile::tempdir().unwrap();
            let target = display("lcd");
            let mut clean = store(&dir);
            clean.persist(&saved(vec![target.clone()])).unwrap();
            clean.mark_in_flight(&target).unwrap();
            let mut broken = store(&dir).failing_after(point);
            assert!(broken.mark_ready(&target).is_err(), "{point}");
            if matches!(point, "after_rename" | "after_parent_sync") {
                // Ready is only possible when mark_ready was invoked after confirmed success.
                assert_eq!(
                    store(&dir).load().unwrap().unwrap().targets[0].state,
                    WriteState::Ready
                );
            } else {
                assert_eq!(
                    fs::read(dir.path().join("monitor-intent.json")).unwrap(),
                    serde_json::to_vec(&saved_in_flight(&target)).unwrap()
                );
                assert!(store(&dir).load().is_err()); // Orphan prevents any replay.
            }
        }
    }

    fn saved_in_flight(target: &MonitorTarget) -> SavedMonitorIntent {
        let mut intent = saved(vec![target.clone()]);
        intent.targets[0].state = WriteState::InFlightUnknown;
        intent
    }

    #[test]
    fn crash_boundaries_preserve_old_file_or_block_on_orphan() {
        for point in [
            "before_write",
            "after_write",
            "after_file_sync",
            "before_rename",
            "after_rename",
            "after_parent_sync",
        ] {
            let dir = tempfile::tempdir().unwrap();
            let target = curve("aio", "pump");
            let mut clean = store(&dir);
            clean.persist(&saved(vec![target.clone()])).unwrap();
            let before = fs::read(dir.path().join("monitor-intent.json")).unwrap();
            let mut broken = store(&dir).failing_after(point);
            assert!(broken.mark_in_flight(&target).is_err(), "{point}");
            let after = fs::read(dir.path().join("monitor-intent.json")).unwrap();
            if matches!(point, "after_rename" | "after_parent_sync") {
                assert!(!dir.path().join(TEMP).exists(), "{point}");
                assert_eq!(
                    store(&dir).load().unwrap().unwrap().targets[0].state,
                    WriteState::InFlightUnknown,
                    "{point}"
                );
            } else {
                assert_eq!(before, after, "{point}");
                assert!(dir.path().join(TEMP).exists(), "{point}");
                assert!(store(&dir).load().is_err(), "{point}");
                assert!(clean.mark_in_flight(&target).is_err(), "{point}");
            }
        }
    }
}
