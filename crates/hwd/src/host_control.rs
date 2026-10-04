//! Recoverable motherboard host-control engine and its dedicated worker.
//!
//! The worker is the service-side ownership boundary: one standard thread owns
//! the synchronous engine, receives concrete commands, and publishes snapshots
//! without exposing sysfs access to callers. Production adapters use only the
//! compiled IT8689 identity and fixed system paths; the small engine traits
//! exist so control behavior can be exercised with deterministic fakes.

use std::{
    any::Any,
    collections::{HashMap, HashSet},
    fs::{self, File, OpenOptions},
    io::{self, Read, Write},
    os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt},
    panic::{AssertUnwindSafe, catch_unwind},
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use nzxt_cam_core::{
    HostChannelPolicy, HostControlPolicy, HostControlSnapshot, HostControlState, HostCurve,
    HostTemperatureSource,
};
use serde::{Deserialize, Serialize};

use crate::{
    config::{HardwareConfig, HostChannelConfig, HostControlConfig},
    hardware::{HardwareError, HardwareErrorKind},
    it8689::{self, BOARD_NAME, BOARD_VENDOR, CHIP_ADDRESS, CHIP_NAME, PLATFORM_COMPONENT},
    it8689_control as control,
    nvidia::{NvidiaSource, NvmlSource},
    ownership::HostControlLock,
    policy_store::{ConfigIdentity, FilePolicyStore, PolicyStore, SavedPolicy},
    telemetry::sample_k10temp,
};

pub const HOST_CONTROL_TICK_INTERVAL: Duration = Duration::from_secs(1);
pub const HOST_CONTROL_SENSOR_FRESHNESS: Duration = Duration::from_secs(3);
pub const HOST_CONTROL_DOWNWARD_HYSTERESIS_MILLIDEGREES: i64 = 2_000;
pub const HOST_CONTROL_RECOVERY_PATH: &str = "/run/nzxt-cam/host-control.json";
const NORMAL_RECOVERY_VERSION: u8 = 3;
const MAX_OBSERVATION_GAP: Duration = Duration::from_secs(2);
const MAX_RECOVERY_BYTES: usize = 4 * 1024;
const RECOVERY_TEMP_FILE: &str = "host-control.json.tmp";
const RECOVERY_MARKER_FILE: &str = "host-control.restored";
const SYSTEM_HWMON_ROOT: &str = "/sys/class/hwmon";
const SYSTEM_DMI_ROOT: &str = "/sys/class/dmi/id";
const MIN_SENSOR_MILLIDEGREES: i64 = -20_000;
const MAX_SENSOR_MILLIDEGREES: i64 = 150_000;
// Board-specific safety floor; do not apply it to arbitrary fan mappings.
const FAN_RPM_FLOOR: u64 = 700;

pub(crate) fn fan_rpm_meets_baseline(baseline: u64, rpm: u64) -> bool {
    baseline >= FAN_RPM_FLOOR
        && u128::from(rpm)
            >= (u128::from(baseline) * 3)
                .div_ceil(4)
                .max(u128::from(FAN_RPM_FLOOR))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TimedTemperature {
    pub millidegrees: i64,
    pub sampled_at: Duration,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HostSensorSample {
    pub cpu: Option<TimedTemperature>,
    pub gpu: Option<TimedTemperature>,
}

/// Narrow sensor seam. Implementations return independently timestamped values.
pub trait HostSensorSource: Send {
    fn sample(&mut self, now: Duration) -> HostSensorSample;
}

/// Narrow monotonic-time seam used for worker tick timestamps.
pub trait MonotonicTimeSource: Send {
    fn now(&mut self) -> Result<Duration, HardwareError>;
}

/// Narrow IT8689 seam. Channel numbers, never paths, cross this boundary.
pub trait HostControlSysfs: Send {
    fn discover_exact(&mut self) -> Result<(), HardwareError>;
    fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError>;
    fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError>;
    fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError>;
    fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError>;
    fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError>;
}

/// Typed recovery boundary. Production parsing and serialization are bounded
/// and strict; test fakes can retain the typed record directly.
pub trait RecoveryStore: Send {
    fn load(&mut self) -> Result<Option<RecoveryRecord>, HardwareError>;
    fn persist(&mut self, record: &RecoveryRecord) -> Result<(), HardwareError>;
    fn remove(&mut self) -> Result<(), HardwareError>;
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RecoveryRecord {
    version: u8,
    board_vendor: String,
    board_name: String,
    chip_name: String,
    chip_address: u16,
    platform_component: String,
    pub(crate) channels: Vec<RecoveryChannel>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct RecoveryChannel {
    pub(crate) pwm_channel: u8,
    pub(crate) fan_channel: u8,
    pub(crate) original_pwm: u8,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) controlled: Option<bool>,
}

impl RecoveryRecord {
    fn from_config(
        config: &HostControlConfig,
        original_pwm: &[u8; 2],
    ) -> Result<Self, HardwareError> {
        let record = Self {
            version: NORMAL_RECOVERY_VERSION,
            board_vendor: BOARD_VENDOR.into(),
            board_name: BOARD_NAME.into(),
            chip_name: CHIP_NAME.into(),
            chip_address: CHIP_ADDRESS,
            platform_component: PLATFORM_COMPONENT.into(),
            channels: [3, 4]
                .into_iter()
                .map(|number| RecoveryChannel {
                    pwm_channel: number,
                    fan_channel: number,
                    original_pwm: original_pwm[usize::from(number - 3)],
                    controlled: Some(
                        config
                            .channels
                            .iter()
                            .any(|entry| entry.pwm_channel == number),
                    ),
                })
                .collect(),
        };
        record.validate_normal()?;
        Ok(record)
    }

    fn validate_normal(&self) -> Result<(), HardwareError> {
        self.validate()?;
        if self.version != NORMAL_RECOVERY_VERSION
            || self.channels.len() != 2
            || self.channels[0].pwm_channel != 3
            || self.channels[1].pwm_channel != 4
            || !self
                .channels
                .iter()
                .any(|entry| entry.controlled == Some(true))
            || self
                .channels
                .iter()
                .any(|entry| entry.controlled.is_none() || entry.original_pwm != 63)
        {
            return Err(invalid(
                "normal recovery requires version 3, both BIOS PWM63 channels and explicit controlled markers",
            ));
        }
        Ok(())
    }

    fn validate_for_resume(&self, config: &HostControlConfig) -> Result<(), HardwareError> {
        self.validate_normal()?;
        validate_config_identity(config)?;
        if self.channels.iter().any(|entry| {
            entry.controlled
                != Some(config.channels.iter().any(|channel| {
                    channel.pwm_channel == entry.pwm_channel
                        && channel.fan_channel == entry.fan_channel
                }))
        }) {
            return Err(invalid(
                "recovery channel ownership does not match the enabled policy configuration",
            ));
        }
        Ok(())
    }

    fn validate(&self) -> Result<(), HardwareError> {
        if self.version != NORMAL_RECOVERY_VERSION
            || self.board_vendor != BOARD_VENDOR
            || self.board_name != BOARD_NAME
            || self.chip_name != CHIP_NAME
            || self.chip_address != CHIP_ADDRESS
            || self.platform_component != PLATFORM_COMPONENT
        {
            return Err(invalid(
                "recovery record does not match the compiled hardware identity",
            ));
        }
        if !(1..=2).contains(&self.channels.len()) {
            return Err(invalid("recovery record must contain one or two channels"));
        }
        let mut pwm = HashSet::new();
        let mut fan = HashSet::new();
        for channel in &self.channels {
            if !matches!((channel.pwm_channel, channel.fan_channel), (3, 3) | (4, 4)) {
                return Err(invalid(
                    "recovery record contains an unsupported channel mapping",
                ));
            }
            if !pwm.insert(channel.pwm_channel) || !fan.insert(channel.fan_channel) {
                return Err(invalid(
                    "recovery record contains duplicate channel mappings",
                ));
            }
        }
        if self.channels.iter().any(|entry| entry.controlled.is_none()) {
            return Err(invalid("recovery version and channel markers disagree"));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RecoveryMetadata {
    is_regular: bool,
    owner: u32,
    links: u64,
    mode: u32,
}

fn validate_recovery_metadata(
    metadata: RecoveryMetadata,
    enforce_root_owner: bool,
) -> Result<(), HardwareError> {
    if !metadata.is_regular {
        return Err(invalid("recovery state is not a regular file"));
    }
    if enforce_root_owner && metadata.owner != 0 {
        return Err(invalid("recovery state is not owned by root"));
    }
    if metadata.links != 1 {
        return Err(invalid("recovery state must have exactly one hard link"));
    }
    if metadata.mode & 0o7777 != 0o600 {
        return Err(invalid("recovery state must have mode 0600"));
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RecoveryFaultPoint {
    PersistTempWrite,
    PersistFileSync,
    PersistRename,
    PersistParentSync,
    RemoveRecordRename,
    RemoveMarkerSync,
    RemoveMarkerUnlink,
    RemoveFinalSync,
}

trait RecoveryFaultInjector: Send {
    fn after(&mut self, point: RecoveryFaultPoint) -> Result<(), HardwareError>;
}

struct NoRecoveryFaults;

impl RecoveryFaultInjector for NoRecoveryFaults {
    fn after(&mut self, _point: RecoveryFaultPoint) -> Result<(), HardwareError> {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RecoveryFsOperation {
    PersistTempWrite,
    PersistFileChmod,
    PersistFileMetadata,
    PersistFileSync,
    PersistRename,
    PersistParentSync,
    RemoveRecordRename,
    RemoveMarkerSync,
    RemoveMarkerUnlink,
    RemoveFinalSync,
}

/// Narrow direct seam around only the filesystem operations whose failure
/// boundaries affect the recovery protocol. Production remains ordinary
/// `std::fs`; tests can fail an operation before its syscall has side effects.
trait RecoveryFsOps: Send {
    fn write_temp(
        &mut self,
        operation: RecoveryFsOperation,
        file: &mut File,
        bytes: &[u8],
    ) -> io::Result<()>;
    fn set_file_mode(
        &mut self,
        operation: RecoveryFsOperation,
        file: &File,
        mode: u32,
    ) -> io::Result<()>;
    fn file_metadata(
        &mut self,
        operation: RecoveryFsOperation,
        file: &File,
    ) -> io::Result<RecoveryMetadata>;
    fn sync_file(&mut self, operation: RecoveryFsOperation, file: &File) -> io::Result<()>;
    fn rename(&mut self, operation: RecoveryFsOperation, from: &Path, to: &Path) -> io::Result<()>;
    fn sync_parent(&mut self, operation: RecoveryFsOperation, parent: &Path) -> io::Result<()>;
    fn unlink_marker(&mut self, operation: RecoveryFsOperation, marker: &Path) -> io::Result<()>;
}

struct StdRecoveryFs;

impl RecoveryFsOps for StdRecoveryFs {
    fn write_temp(
        &mut self,
        _operation: RecoveryFsOperation,
        file: &mut File,
        bytes: &[u8],
    ) -> io::Result<()> {
        file.write_all(bytes)
    }

    fn set_file_mode(
        &mut self,
        _operation: RecoveryFsOperation,
        file: &File,
        mode: u32,
    ) -> io::Result<()> {
        file.set_permissions(fs::Permissions::from_mode(mode))
    }

    fn file_metadata(
        &mut self,
        _operation: RecoveryFsOperation,
        file: &File,
    ) -> io::Result<RecoveryMetadata> {
        let metadata = file.metadata()?;
        Ok(RecoveryMetadata {
            is_regular: metadata.file_type().is_file(),
            owner: metadata.uid(),
            links: metadata.nlink(),
            mode: metadata.mode(),
        })
    }

    fn sync_file(&mut self, _operation: RecoveryFsOperation, file: &File) -> io::Result<()> {
        file.sync_all()
    }

    fn rename(
        &mut self,
        _operation: RecoveryFsOperation,
        from: &Path,
        to: &Path,
    ) -> io::Result<()> {
        fs::rename(from, to)
    }

    fn sync_parent(&mut self, _operation: RecoveryFsOperation, parent: &Path) -> io::Result<()> {
        OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(parent)?
            .sync_all()
    }

    fn unlink_marker(&mut self, _operation: RecoveryFsOperation, marker: &Path) -> io::Result<()> {
        fs::remove_file(marker)
    }
}

/// Fixed-path, atomically durable production recovery store.
pub(crate) struct FileRecoveryStore {
    path: PathBuf,
    enforce_root_owner: bool,
    faults: Box<dyn RecoveryFaultInjector>,
    fs_ops: Box<dyn RecoveryFsOps>,
}

impl FileRecoveryStore {
    #[must_use]
    pub(crate) fn new() -> Self {
        Self {
            path: PathBuf::from(HOST_CONTROL_RECOVERY_PATH),
            enforce_root_owner: true,
            faults: Box::new(NoRecoveryFaults),
            fs_ops: Box::new(StdRecoveryFs),
        }
    }

    #[cfg(test)]
    fn at(path: PathBuf) -> Self {
        Self {
            path,
            enforce_root_owner: false,
            faults: Box::new(NoRecoveryFaults),
            fs_ops: Box::new(StdRecoveryFs),
        }
    }

    #[cfg(test)]
    fn at_with_faults(path: PathBuf, faults: Box<dyn RecoveryFaultInjector>) -> Self {
        Self {
            faults,
            ..Self::at(path)
        }
    }

    #[cfg(test)]
    fn at_with_fs(path: PathBuf, fs_ops: Box<dyn RecoveryFsOps>) -> Self {
        Self {
            fs_ops,
            ..Self::at(path)
        }
    }

    fn parent(&self) -> Result<&Path, HardwareError> {
        self.path
            .parent()
            .ok_or_else(|| HardwareError::new("recovery path has no parent"))
    }

    fn temp_path(&self) -> Result<PathBuf, HardwareError> {
        Ok(self.parent()?.join(RECOVERY_TEMP_FILE))
    }

    fn marker_path(&self) -> Result<PathBuf, HardwareError> {
        Ok(self.parent()?.join(RECOVERY_MARKER_FILE))
    }

    fn read_bounded_path(
        &self,
        path: &Path,
        description: &str,
    ) -> Result<Option<Vec<u8>>, HardwareError> {
        let file = match OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(path)
        {
            Ok(file) => file,
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(io_error(&format!("open {description}"), error)),
        };
        let metadata = file
            .metadata()
            .map_err(|error| io_error(&format!("inspect {description}"), error))?;
        validate_recovery_metadata(
            RecoveryMetadata {
                is_regular: metadata.file_type().is_file(),
                owner: metadata.uid(),
                links: metadata.nlink(),
                mode: metadata.mode(),
            },
            self.enforce_root_owner,
        )?;

        let mut bytes = Vec::with_capacity(MAX_RECOVERY_BYTES + 1);
        file.take((MAX_RECOVERY_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(|error| io_error(&format!("read {description}"), error))?;
        if bytes.len() > MAX_RECOVERY_BYTES {
            return Err(invalid("recovery state exceeds the 4096-byte limit"));
        }
        Ok(Some(bytes))
    }

    fn read_record(&self) -> Result<Option<Vec<u8>>, HardwareError> {
        self.read_bounded_path(&self.path, "recovery record")
    }

    fn read_marker(&self) -> Result<Option<Vec<u8>>, HardwareError> {
        self.read_bounded_path(&self.marker_path()?, "recovery cleared marker")
    }

    fn parse_record(bytes: &[u8], description: &str) -> Result<RecoveryRecord, HardwareError> {
        let record: RecoveryRecord = serde_json::from_slice(bytes)
            .map_err(|error| invalid(format!("invalid {description}: {error}")))?;
        record.validate_normal()?;
        Ok(record)
    }

    fn validate_removal_record(
        &self,
        bytes: &[u8],
        description: &str,
    ) -> Result<(), HardwareError> {
        // Cleared markers are recovery evidence too. Unsupported records must
        // remain intact rather than being acknowledged as restored.
        Self::parse_record(bytes, description).map(|_| ())
    }

    fn fault_after(&mut self, point: RecoveryFaultPoint) -> Result<(), HardwareError> {
        self.faults.after(point)
    }

    fn sync_parent(
        &mut self,
        fs_operation: RecoveryFsOperation,
        description: &str,
    ) -> Result<(), HardwareError> {
        let parent = self.parent()?.to_path_buf();
        self.fs_ops
            .sync_parent(fs_operation, &parent)
            .map_err(|error| io_error(description, error))
    }

    fn finish_marker_removal(&mut self) -> Result<(), HardwareError> {
        self.sync_parent(
            RecoveryFsOperation::RemoveMarkerSync,
            "sync installed recovery cleared marker",
        )?;
        self.fault_after(RecoveryFaultPoint::RemoveMarkerSync)?;

        let marker = self.marker_path()?;
        self.fs_ops
            .unlink_marker(RecoveryFsOperation::RemoveMarkerUnlink, &marker)
            .map_err(|error| io_error("remove recovery cleared marker", error))?;
        self.fault_after(RecoveryFaultPoint::RemoveMarkerUnlink)?;

        self.sync_parent(
            RecoveryFsOperation::RemoveFinalSync,
            "sync removed recovery cleared marker",
        )?;
        self.fault_after(RecoveryFaultPoint::RemoveFinalSync)
    }
}

impl Default for FileRecoveryStore {
    fn default() -> Self {
        Self::new()
    }
}

impl RecoveryStore for FileRecoveryStore {
    fn load(&mut self) -> Result<Option<RecoveryRecord>, HardwareError> {
        let record = self.read_record()?;
        let marker = self.read_marker()?;
        match (record, marker) {
            (Some(_), Some(_)) => Err(invalid("recovery record and cleared marker both exist")),
            (Some(bytes), None) => Ok(Some(Self::parse_record(&bytes, "recovery record")?)),
            (None, Some(bytes)) => {
                self.validate_removal_record(&bytes, "recovery cleared marker")?;
                self.finish_marker_removal().map_err(|error| {
                    restore_required(format!(
                        "could not durably clean an already-restored marker: {error}"
                    ))
                })?;
                Ok(None)
            }
            (None, None) => Ok(None),
        }
    }

    fn persist(&mut self, record: &RecoveryRecord) -> Result<(), HardwareError> {
        record.validate_normal()?;
        if self.read_record()?.is_some() || self.read_marker()?.is_some() {
            return Err(invalid("host-control recovery state already exists"));
        }
        let bytes = serde_json::to_vec(record)
            .map_err(|error| HardwareError::new(format!("serialize recovery record: {error}")))?;
        if bytes.len() > MAX_RECOVERY_BYTES {
            return Err(invalid("recovery record exceeds the 4096-byte limit"));
        }

        let temp = self.temp_path()?;
        match fs::remove_file(&temp) {
            Ok(()) => {}
            Err(error) if error.kind() == io::ErrorKind::NotFound => {}
            Err(error) => return Err(io_error("remove stale recovery temporary file", error)),
        }

        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(&temp)
            .map_err(|error| io_error("create recovery temporary file", error))?;
        // `OpenOptionsExt::mode` is filtered through the process umask. Set the
        // descriptor mode explicitly, then fstat and validate that same open
        // descriptor before any recovery bytes are written.
        self.fs_ops
            .set_file_mode(RecoveryFsOperation::PersistFileChmod, &file, 0o600)
            .map_err(|error| io_error("set recovery temporary file mode", error))?;
        let metadata = self
            .fs_ops
            .file_metadata(RecoveryFsOperation::PersistFileMetadata, &file)
            .map_err(|error| io_error("inspect recovery temporary file", error))?;
        validate_recovery_metadata(metadata, self.enforce_root_owner)?;

        self.fs_ops
            .write_temp(RecoveryFsOperation::PersistTempWrite, &mut file, &bytes)
            .map_err(|error| io_error("write recovery record", error))?;
        self.fault_after(RecoveryFaultPoint::PersistTempWrite)?;
        self.fs_ops
            .sync_file(RecoveryFsOperation::PersistFileSync, &file)
            .map_err(|error| io_error("sync recovery record", error))?;
        self.fault_after(RecoveryFaultPoint::PersistFileSync)?;
        drop(file);

        self.fs_ops
            .rename(RecoveryFsOperation::PersistRename, &temp, &self.path)
            .map_err(|error| io_error("install recovery record", error))?;
        self.fault_after(RecoveryFaultPoint::PersistRename)?;
        self.sync_parent(
            RecoveryFsOperation::PersistParentSync,
            "sync recovery directory",
        )?;
        self.fault_after(RecoveryFaultPoint::PersistParentSync)
    }

    fn remove(&mut self) -> Result<(), HardwareError> {
        let record = self.read_record()?;
        let marker = self.read_marker()?;
        match (&record, &marker) {
            (Some(_), Some(_)) => {
                return Err(invalid("recovery record and cleared marker both exist"));
            }
            (Some(bytes), None) => {
                self.validate_removal_record(bytes, "recovery record")?;
                let marker = self.marker_path()?;
                self.fs_ops
                    .rename(RecoveryFsOperation::RemoveRecordRename, &self.path, &marker)
                    .map_err(|error| io_error("mark recovery record as restored", error))?;
                self.fault_after(RecoveryFaultPoint::RemoveRecordRename)?;
            }
            (None, Some(bytes)) => {
                self.validate_removal_record(bytes, "recovery cleared marker")?;
            }
            (None, None) => {
                self.sync_parent(
                    RecoveryFsOperation::RemoveFinalSync,
                    "sync completed recovery removal",
                )?;
                return self.fault_after(RecoveryFaultPoint::RemoveFinalSync);
            }
        }
        self.finish_marker_removal()
    }
}

/// Production sysfs adapter. Discovery is always repeated against fixed roots,
/// and only attributes below the exact discovered IT8689 are accessed.
pub(crate) struct It8689Sysfs {
    device: Option<PathBuf>,
}

impl It8689Sysfs {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self { device: None }
    }

    fn attribute(&self, name: impl AsRef<Path>) -> Result<PathBuf, HardwareError> {
        let root = self
            .device
            .as_ref()
            .ok_or_else(|| unavailable("IT8689 has not been discovered"))?;
        Ok(root.join(name))
    }

    fn read_integer(&self, name: impl AsRef<Path>) -> Result<u64, HardwareError> {
        let path = self.attribute(name)?;
        let text =
            fs::read_to_string(path).map_err(|error| io_error("read IT8689 attribute", error))?;
        text.trim()
            .parse::<u64>()
            .map_err(|_| invalid("IT8689 attribute is not an unsigned integer"))
    }

    fn write_integer(&self, name: impl AsRef<Path>, value: u8) -> Result<(), HardwareError> {
        fs::write(self.attribute(name)?, value.to_string())
            .map_err(|error| io_error("write IT8689 attribute", error))
    }
}

impl Default for It8689Sysfs {
    fn default() -> Self {
        Self::new()
    }
}

impl HostControlSysfs for It8689Sysfs {
    fn discover_exact(&mut self) -> Result<(), HardwareError> {
        self.device = it8689::discover(Path::new(SYSTEM_HWMON_ROOT), Path::new(SYSTEM_DMI_ROOT))
            .map(|device| device.path().to_owned());
        self.device
            .as_ref()
            .map(|_| ())
            .ok_or_else(|| unavailable("unique exact IT8689 was not discovered"))
    }

    fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError> {
        validate_it8689_channel(channel)?;
        u8::try_from(self.read_integer(format!("pwm{channel}"))?)
            .map_err(|_| invalid("IT8689 PWM value is outside 0..=255"))
    }

    fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError> {
        validate_it8689_channel(channel)?;
        self.read_integer(format!("fan{channel}_input"))
    }

    fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError> {
        validate_it8689_channel(channel)?;
        u8::try_from(self.read_integer(format!("pwm{channel}_enable"))?)
            .map_err(|_| invalid("IT8689 mode value is outside 0..=255"))
    }

    fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
        validate_it8689_channel(channel)?;
        self.write_integer(format!("pwm{channel}"), value)
    }

    fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
        validate_it8689_channel(channel)?;
        self.write_integer(format!("pwm{channel}_enable"), value)
    }
}

fn validate_it8689_channel(channel: u8) -> Result<(), HardwareError> {
    matches!(channel, 3 | 4)
        .then_some(())
        .ok_or_else(|| invalid(format!("IT8689 channel {channel} is not allowlisted")))
}

/// Production CPU/NVIDIA source using the existing k10temp and UUID-only NVML
/// implementations. Successful reads are stamped with the supplied monotonic
/// observation time.
pub struct ProductionHostSensors {
    nvidia_uuid: Option<String>,
    nvidia_pci_bus_id: Option<String>,
    nvidia: NvmlSource,
}

impl ProductionHostSensors {
    #[must_use]
    pub fn new(nvidia_uuid: Option<String>, nvidia_pci_bus_id: Option<String>) -> Self {
        Self {
            nvidia_uuid,
            nvidia_pci_bus_id,
            nvidia: NvmlSource::new(),
        }
    }
}

impl HostSensorSource for ProductionHostSensors {
    fn sample(&mut self, now: Duration) -> HostSensorSample {
        let cpu =
            sample_k10temp(Path::new(SYSTEM_HWMON_ROOT)).map(|millidegrees| TimedTemperature {
                millidegrees,
                sampled_at: now,
            });
        let gpu = self
            .nvidia_uuid
            .as_deref()
            .and_then(|uuid| {
                self.nvidia
                    .temperature_celsius(uuid, self.nvidia_pci_bus_id.as_deref())
            })
            .and_then(|celsius| {
                let millidegrees = (celsius * 1_000.0).round();
                (millidegrees.is_finite()
                    && millidegrees >= MIN_SENSOR_MILLIDEGREES as f64
                    && millidegrees <= MAX_SENSOR_MILLIDEGREES as f64)
                    .then_some(TimedTemperature {
                        millidegrees: millidegrees as i64,
                        sampled_at: now,
                    })
            });
        HostSensorSample { cpu, gpu }
    }
}

pub struct SystemMonotonicTime {
    origin: Instant,
}

impl SystemMonotonicTime {
    #[must_use]
    pub fn new() -> Self {
        Self {
            origin: Instant::now(),
        }
    }
}

impl Default for SystemMonotonicTime {
    fn default() -> Self {
        Self::new()
    }
}

impl MonotonicTimeSource for SystemMonotonicTime {
    fn now(&mut self) -> Result<Duration, HardwareError> {
        Ok(self.origin.elapsed())
    }
}

#[derive(Clone)]
struct ActiveChannel {
    config: HostChannelConfig,
    policy: HostChannelPolicy,
    duty_millipercent: i64,
    commanded_pwm: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RecoveryBlockReason {
    LoadFailed,
    PersistUncertain,
}

#[cfg(test)]
#[derive(Default)]
pub(crate) struct MemoryPolicyStore(Option<SavedPolicy>);
#[cfg(test)]
impl PolicyStore for MemoryPolicyStore {
    fn load(&mut self) -> Result<Option<SavedPolicy>, HardwareError> {
        Ok(self.0.clone())
    }
    fn persist(&mut self, saved: &SavedPolicy) -> Result<(), HardwareError> {
        self.0 = Some(saved.clone());
        Ok(())
    }
}

/// Synchronous deterministic engine owned by the dedicated service worker.
pub struct HostControlEngine {
    config: Option<HostControlConfig>,
    nvidia_uuid: Option<String>,
    sysfs: Box<dyn HostControlSysfs>,
    recovery: Box<dyn RecoveryStore>,
    sensors: Box<dyn HostSensorSource>,
    clock: Box<dyn MonotonicTimeSource>,
    state: HostControlState,
    recoverable_channels: Vec<RecoveryChannel>,
    active: Vec<ActiveChannel>,
    active_policy: Option<HostControlPolicy>,
    policy_store: Box<dyn PolicyStore>,
    nvidia_pci_bus_id: Option<String>,
    policy_uncertain: bool,
    restore_authorized: bool,
    last_tick: Option<Duration>,
    baselines: [u64; 2],
    last_time: Option<Duration>,
    recovery_block: Option<RecoveryBlockReason>,
}

impl HostControlEngine {
    /// Constructs production adapters without performing sysfs/NVML access.
    /// Bootstrap performs all durable-state inspection explicitly.
    #[must_use]
    pub fn new(config: HardwareConfig) -> Self {
        let sensors = ProductionHostSensors::new(
            config.nvidia_uuid.clone(),
            config.nvidia_pci_bus_id.clone(),
        );
        let pci = config.nvidia_pci_bus_id.clone();
        let mut engine = Self::with_dependencies(
            config.host_control,
            config.nvidia_uuid,
            Box::new(It8689Sysfs::new()),
            Box::new(FileRecoveryStore::new()),
            Box::new(sensors),
            Box::new(SystemMonotonicTime::new()),
            Box::new(FilePolicyStore::new()),
        );
        engine.nvidia_pci_bus_id = pci;
        engine
    }

    pub fn with_dependencies(
        config: Option<HostControlConfig>,
        nvidia_uuid: Option<String>,
        sysfs: Box<dyn HostControlSysfs>,
        recovery: Box<dyn RecoveryStore>,
        sensors: Box<dyn HostSensorSource>,
        clock: Box<dyn MonotonicTimeSource>,
        policy_store: Box<dyn PolicyStore>,
    ) -> Self {
        Self {
            state: if config.is_some() {
                HostControlState::Available
            } else {
                HostControlState::Disabled
            },
            config,
            nvidia_uuid,
            nvidia_pci_bus_id: None,
            sysfs,
            recovery,
            sensors,
            clock,
            policy_store,
            policy_uncertain: false,
            restore_authorized: false,
            active_policy: None,
            recoverable_channels: Vec::new(),
            active: Vec::new(),
            last_tick: None,
            baselines: [0; 2],
            last_time: None,
            recovery_block: None,
        }
    }

    #[cfg(test)]
    fn with_policy_store(mut self, store: Box<dyn PolicyStore>) -> Self {
        self.policy_store = store;
        self
    }

    fn identity(&self) -> Option<ConfigIdentity> {
        self.config.clone().map(|host_control| ConfigIdentity {
            host_control,
            nvidia_uuid: self.nvidia_uuid.clone(),
            nvidia_pci_bus_id: self.nvidia_pci_bus_id.clone(),
        })
    }

    /// Called by the service before accepting clients. Invalid intent or
    /// recovery evidence never authorizes an automatic motherboard write.
    pub fn initialize(&mut self) -> Result<(), HardwareError> {
        self.initialize_with_resume(true)
    }

    /// Always resolves guarded recovery; only a permitted startup re-enters
    /// manual PWM control. Disabling auto-resume never disarms saved intent.
    pub fn initialize_with_resume(&mut self, resume_saved: bool) -> Result<(), HardwareError> {
        self.initialize_with_check(resume_saved, || Ok(()))
    }

    fn initialize_with_check(
        &mut self,
        resume_saved: bool,
        mut check: impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(), HardwareError> {
        let result = self.initialize_inner(resume_saved, &mut check);
        if result.is_err() && !result.as_ref().err().is_some_and(is_service_shutdown) {
            self.state = HostControlState::RestoreRequired;
        }
        result
    }

    fn initialize_inner(
        &mut self,
        resume_saved: bool,
        mut check: impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(), HardwareError> {
        check()?;
        let saved = self.policy_store.load()?;
        check()?;
        let record = match self.recovery.load() {
            Ok(record) => record,
            Err(error) => {
                self.recovery_block = Some(RecoveryBlockReason::LoadFailed);
                return Err(error);
            }
        };
        check()?;
        let enabled = match saved {
            Some(SavedPolicy::Enabled {
                complete_policy,
                exact_config_identity,
                ..
            }) => {
                let identity = self.identity().ok_or_else(|| {
                    restore_required("saved policy has no configured host control")
                })?;
                if exact_config_identity != identity {
                    return Err(restore_required(
                        "saved policy configuration identity changed",
                    ));
                }
                validate_config_identity(&identity.host_control)?;
                validate_policy(
                    &identity.host_control,
                    identity.nvidia_uuid.as_deref(),
                    &complete_policy,
                )?;
                Some(complete_policy)
            }
            None | Some(SavedPolicy::Disabled { .. }) => None,
        };
        if let Some(record) = record {
            if let Err(error) = record.validate_normal() {
                self.recovery_block = Some(RecoveryBlockReason::LoadFailed);
                return Err(error);
            }
            if enabled.is_some() {
                record.validate_for_resume(
                    self.config
                        .as_ref()
                        .expect("enabled policy configuration validated"),
                )?;
            }
            self.recoverable_channels = record.channels;
            self.state = HostControlState::RestoreRequired;
            if enabled.is_none() {
                return Err(restore_required(
                    "recovery state without matching enabled intent requires operator restore",
                ));
            }
            self.restore_authorized = true;
            // Only matching enabled intent authorizes automatic recovery.
            check()?;
            if let Err(error) = self.finalize() {
                let disarm = self.disarm();
                self.state = HostControlState::RestoreRequired;
                return Err(match disarm {
                    Ok(()) => error,
                    Err(disarm) => restore_required(format!(
                        "bootstrap restoration: {error}; policy disarm: {disarm}"
                    )),
                });
            }
        }
        if resume_saved && let Some(policy) = enabled {
            // Verified matching intent authorizes restoration of this resumed
            // session too. Recovery of a pending v3 record above calls
            // set_inactive_state(), which clears the prior authorization.
            self.restore_authorized = true;
            // Startup failure disarms intent; there is no retry loop.
            self.start_committed(&policy, check)?;
        }
        Ok(())
    }

    fn disarm(&mut self) -> Result<(), HardwareError> {
        if self.policy_uncertain {
            return Err(restore_required("policy durability is uncertain"));
        }
        if let Err(error) = self
            .policy_store
            .persist(&SavedPolicy::Disabled { version: 1 })
        {
            self.policy_uncertain = true;
            self.state = HostControlState::RestoreRequired;
            return Err(restore_required(format!(
                "durable policy disarm is uncertain: {error}"
            )));
        }
        Ok(())
    }

    #[must_use]
    pub fn snapshot(&self) -> HostControlSnapshot {
        HostControlSnapshot {
            state: self.state,
            last_error: None,
            active_policy: if self.state == HostControlState::Running {
                self.active_policy.clone()
            } else {
                None
            },
            channels: self
                .config
                .as_ref()
                .map_or_else(Vec::new, HostControlConfig::capabilities),
        }
    }

    pub fn monotonic_now(&mut self) -> Result<Duration, HardwareError> {
        self.clock.now()
    }

    pub fn start(&mut self, policy: &HostControlPolicy) -> Result<(), HardwareError> {
        self.start_with_abort(policy, || Ok(()))
    }

    /// Starts host control while checking an in-process cancellation source at
    /// every blocking boundary. The callback is deliberately local to the
    /// worker and is never represented in IPC or recovery state.
    fn start_with_abort(
        &mut self,
        policy: &HostControlPolicy,
        mut check_abort: impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(), HardwareError> {
        check_abort()?;
        if self.state != HostControlState::Available {
            return Err(unavailable("host control is not available"));
        }
        let config = self
            .config
            .clone()
            .ok_or_else(|| unavailable("host control is disabled"))?;
        validate_config_identity(&config)?;
        validate_policy(&config, self.nvidia_uuid.as_deref(), policy)?;
        if self.policy_uncertain {
            return Err(restore_required("policy durability is uncertain"));
        }
        if let Err(error) = self.policy_store.persist(&SavedPolicy::Enabled {
            version: 1,
            complete_policy: policy.clone(),
            exact_config_identity: self.identity().expect("validated configured host control"),
        }) {
            // A write can have reached rename even when its durability result
            // is unknown. Attempt a durable tombstone, but never claim success
            // or retry Start in this process after the ambiguous commit.
            let disarm = self
                .policy_store
                .persist(&SavedPolicy::Disabled { version: 1 });
            self.policy_uncertain = true;
            self.state = HostControlState::RestoreRequired;
            return Err(restore_required(format!(
                "durable enabled intent is uncertain: {error}; best-effort disarm: {disarm:?}"
            )));
        }
        self.restore_authorized = true;
        self.start_committed(policy, check_abort)
    }

    fn start_committed(
        &mut self,
        policy: &HostControlPolicy,
        mut check_abort: impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(), HardwareError> {
        let result = self.start_committed_inner(policy, &mut check_abort);
        match result {
            Ok(()) => {
                self.active_policy = Some(policy.clone());
                Ok(())
            }
            Err(error) if is_service_shutdown(&error) => Err(self.finalize_after_shutdown(error)),
            Err(error) => Err(self.finalize_after_error(error)),
        }
    }

    fn start_committed_inner(
        &mut self,
        policy: &HostControlPolicy,
        mut check_abort: impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(), HardwareError> {
        let config = self
            .config
            .clone()
            .ok_or_else(|| unavailable("host control is disabled"))?;
        let ordered_policy = validate_policy(&config, self.nvidia_uuid.as_deref(), policy)?;
        abortable(&mut check_abort, || self.sysfs.discover_exact())?;
        let started = abortable(&mut check_abort, || self.clock.now())?;
        // Both channels must be known even when only one is configured.
        let mut original_pwm = [0; 2];
        let mut baselines = [0; 2];
        for number in [3, 4] {
            abortable(&mut check_abort, || {
                control::verify_mode(self.sysfs.as_mut(), number, 2)
            })?;
            original_pwm[usize::from(number - 3)] =
                abortable(&mut check_abort, || self.sysfs.read_pwm(number))?;
            if original_pwm[usize::from(number - 3)] != 63 {
                return Err(unavailable("both BIOS channels must have dormant PWM63"));
            }
            baselines[usize::from(number - 3)] = abortable(&mut check_abort, || {
                control::rpm(self.sysfs.as_mut(), number, None)
            })?;
        }
        let mut initial = Vec::with_capacity(config.channels.len());
        let now = self.guard_sample(&mut check_abort, started, &ordered_policy)?;
        for (channel, channel_policy) in config.channels.iter().zip(&ordered_policy) {
            let temperature = required_temperature(channel_policy.curve.source, now.1, now.0)?;
            initial.push(ActiveChannel {
                config: channel.clone(),
                policy: (*channel_policy).clone(),
                duty_millipercent: evaluate_curve(&channel_policy.curve, temperature),
                commanded_pwm: control::ENTRY_PWM,
            });
        }
        let record = RecoveryRecord::from_config(&config, &original_pwm)?;
        check_abort()?;
        if let Err(error) = self.recovery.persist(&record) {
            // A failed fsync/rename is ambiguous. No sysfs writes in this engine.
            self.recoverable_channels.clear();
            self.recovery_block = Some(RecoveryBlockReason::PersistUncertain);
            self.active.clear();
            self.last_tick = None;
            self.last_time = None;
            self.state = HostControlState::RestoreRequired;
            return Err(restore_required(format!(
                "recovery persistence is uncertain; no hardware writes were attempted: {error}"
            )));
        }
        self.recoverable_channels = record.channels.clone();
        let start_result = (|| {
            let mut boundary = abortable(&mut check_abort, || self.clock.now())?;
            if boundary < started || boundary - started > MAX_OBSERVATION_GAP {
                return Err(unavailable(
                    "host-control persistence exceeded observation gap",
                ));
            }
            for channel in &config.channels {
                let number = channel.pwm_channel;
                // Reobserve all fans and real CPU after persistence and before
                // each independent mode transition. No mutation on stale observations.
                let (observed, _) =
                    self.guard_sample(&mut check_abort, boundary, &ordered_policy)?;
                for other in [3, 4] {
                    abortable(&mut check_abort, || {
                        control::rpm(
                            self.sysfs.as_mut(),
                            other,
                            Some(baselines[usize::from(other - 3)]),
                        )
                    })?;
                    let mode = abortable(&mut check_abort, || self.sysfs.read_enable(other))?;
                    let pwm = abortable(&mut check_abort, || self.sysfs.read_pwm(other))?;
                    if config
                        .channels
                        .iter()
                        .take_while(|c| c.pwm_channel != number)
                        .any(|c| c.pwm_channel == other)
                    {
                        if !control::manual_observed(other, mode, pwm) || pwm != control::ENTRY_PWM
                        {
                            return Err(unavailable("prior manual channel changed during startup"));
                        }
                    } else if mode != 2 || pwm != 63 {
                        return Err(unavailable("BIOS channel changed during startup"));
                    }
                }
                self.check_gap(observed, &mut check_abort)?;
                control::take_manual(self.sysfs.as_mut(), number, &mut check_abort)?;
                let after = self.check_gap(observed, &mut check_abort)?;
                boundary = after;
            }
            let (observed, _) = self.guard_sample(&mut check_abort, boundary, &ordered_policy)?;
            for number in [3, 4] {
                abortable(&mut check_abort, || {
                    control::rpm(
                        self.sysfs.as_mut(),
                        number,
                        Some(baselines[usize::from(number - 3)]),
                    )
                })?;
                let mode = abortable(&mut check_abort, || self.sysfs.read_enable(number))?;
                let pwm = abortable(&mut check_abort, || self.sysfs.read_pwm(number))?;
                if config.channels.iter().any(|c| c.pwm_channel == number) {
                    if mode != 1 || pwm != control::ENTRY_PWM {
                        return Err(unavailable("manual entry readback changed"));
                    }
                } else if mode != 2 || pwm != 63 {
                    return Err(unavailable("untouched BIOS channel changed"));
                }
            }
            self.check_gap(observed, &mut check_abort)?;
            Ok(())
        })();
        start_result?;

        self.active = initial;
        let now = abortable(&mut check_abort, || self.clock.now())?;
        self.baselines = baselines;
        self.last_tick = Some(now);
        self.last_time = Some(now);
        self.state = HostControlState::Running;
        Ok(())
    }

    fn check_gap(
        &mut self,
        since: Duration,
        check: &mut impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<Duration, HardwareError> {
        let now = abortable(check, || self.clock.now())?;
        if now < since || now - since > MAX_OBSERVATION_GAP {
            return Err(unavailable(
                "host-control observation gap exceeded two seconds",
            ));
        }
        Ok(now)
    }

    fn guard_sample(
        &mut self,
        check: &mut impl FnMut() -> Result<(), HardwareError>,
        since: Duration,
        policies: &[&HostChannelPolicy],
    ) -> Result<(Duration, HostSensorSample), HardwareError> {
        let now = self.check_gap(since, check)?;
        check()?;
        let sample = self.sensors.sample(now);
        required_temperature(HostTemperatureSource::Cpu, sample, now)?;
        for policy in policies {
            required_temperature(policy.curve.source, sample, now)?;
        }
        check()?;
        self.check_gap(since, check)?;
        Ok((now, sample))
    }

    /// Observe the existing running session before considering a new policy.
    /// This is the same read-only guard used by the tick before PWM writes.
    fn observe_running_health(
        &mut self,
        now: Duration,
        check: &mut impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(Duration, HostSensorSample), HardwareError> {
        check()?;
        self.observe_time(now)?;
        let previous = self
            .last_tick
            .ok_or_else(|| unavailable("missing last observation"))?;
        if now < previous || now - previous > MAX_OBSERVATION_GAP {
            return Err(unavailable("host-control missed observation deadline"));
        }
        let policies: Vec<_> = self.active.iter().map(|c| c.policy.clone()).collect();
        let refs: Vec<_> = policies.iter().collect();
        let (sample_time, sample) = self.guard_sample(check, previous, &refs)?;
        if sample_time < now {
            return Err(invalid(
                "monotonic clock moved backwards during observation",
            ));
        }
        for number in [3, 4] {
            check()?;
            let mode = abortable(check, || self.sysfs.read_enable(number))?;
            let pwm = abortable(check, || self.sysfs.read_pwm(number))?;
            let expected = self
                .active
                .iter()
                .find(|c| c.config.pwm_channel == number)
                .map(|c| c.commanded_pwm);
            if let Some(expected) = expected {
                if pwm != expected || !control::manual_observed(number, mode, pwm) {
                    return Err(unavailable(format!("fan{number} manual mode/PWM changed")));
                }
            } else if mode != 2 || pwm != 63 {
                return Err(unavailable(format!(
                    "untouched fan{number} BIOS mode/PWM changed"
                )));
            }
            abortable(check, || {
                control::rpm(
                    self.sysfs.as_mut(),
                    number,
                    Some(self.baselines[usize::from(number - 3)]),
                )
            })?;
            self.check_gap(previous, check)?;
        }
        Ok((sample_time, sample))
    }

    pub fn update(&mut self, channel_policies: &[HostChannelPolicy]) -> Result<(), HardwareError> {
        self.update_with_abort(channel_policies, || Ok(()))
    }

    fn update_with_abort(
        &mut self,
        channel_policies: &[HostChannelPolicy],
        mut check_abort: impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(), HardwareError> {
        check_abort()?;
        self.require_running()?;
        // Validate the whole proposal before observing, persisting or modifying
        // any part of the running session. A duplicate cannot silently win.
        if channel_policies.is_empty() {
            return Err(invalid("host update must contain at least one channel"));
        }
        let config = self.config.as_ref().expect("running configuration");
        let mut seen = HashSet::new();
        for policy in channel_policies {
            if !seen.insert(&policy.channel_id) {
                return Err(invalid(format!(
                    "host policy channel {:?} is duplicated",
                    policy.channel_id.0
                )));
            }
            let channel = config
                .channels
                .iter()
                .find(|c| c.id == policy.channel_id)
                .ok_or_else(|| {
                    invalid(format!(
                        "host policy channel {:?} is not configured",
                        policy.channel_id.0
                    ))
                })?;
            validate_curve(
                &policy.curve,
                channel.minimum_duty_percent,
                self.nvidia_uuid.as_deref(),
            )?;
        }
        let mut merged = self.active_policy.clone().expect("running policy exists");
        for policy in channel_policies {
            let changed = merged
                .channels
                .iter_mut()
                .find(|c| c.channel_id == policy.channel_id)
                .expect("active channel is configured");
            *changed = policy.clone();
        }
        validate_policy(config, self.nvidia_uuid.as_deref(), &merged)?;
        if self.policy_uncertain {
            return Err(restore_required("policy durability is uncertain"));
        }
        // Existing-source, CPU, RPM, readback and deadline faults disarm and
        // restore. Missing *proposed* sources reject the whole batch before
        // durable intent changes, leaving healthy old control running.
        match self.observe_running_health_for_update(&mut check_abort) {
            Ok((observed, sample)) => {
                for policy in &merged.channels {
                    required_temperature(policy.curve.source, sample, observed)?;
                }
            }
            Err(error) if is_service_shutdown(&error) => return Err(error),
            Err(error) => return Err(self.finalize_after_error(error)),
        }
        check_abort()?;
        if let Err(error) = self.policy_store.persist(&SavedPolicy::Enabled {
            version: 1,
            complete_policy: merged.clone(),
            exact_config_identity: self.identity().expect("running identity"),
        }) {
            self.policy_uncertain = true;
            let disarm = self
                .policy_store
                .persist(&SavedPolicy::Disabled { version: 1 });
            let restore = self.finalize();
            self.state = HostControlState::RestoreRequired;
            return Err(restore_required(format!(
                "live policy durability is uncertain: {error}; best-effort disarm: {disarm:?}; restoration: {restore:?}"
            )));
        }
        // Durable intent precedes acceptance. Collect ALL temperatures and
        // finish abort checks before modifying even the first active channel.
        let temperatures = match self.observe_running_health_for_update(&mut check_abort) {
            Ok((observed, sample)) => merged
                .channels
                .iter()
                .map(|policy| required_temperature(policy.curve.source, sample, observed))
                .collect::<Result<Vec<_>, _>>(),
            Err(error) => Err(error),
        };
        let temperatures = match temperatures {
            Ok(values) => values,
            Err(error) if is_service_shutdown(&error) => {
                return Err(self.finalize_after_shutdown(error));
            }
            Err(error) => return Err(self.finalize_after_error(error)),
        };
        if let Err(error) = check_abort() {
            return Err(if is_service_shutdown(&error) {
                self.finalize_after_shutdown(error)
            } else {
                self.finalize_after_error(error)
            });
        }
        for (policy, temperature) in merged.channels.iter().zip(temperatures) {
            if seen.contains(&policy.channel_id) {
                let active = self
                    .active
                    .iter_mut()
                    .find(|c| c.config.id == policy.channel_id)
                    .expect("active channel is configured");
                active.duty_millipercent = evaluate_curve(&policy.curve, temperature);
                active.policy = policy.clone();
            }
        }
        self.active_policy = Some(merged);
        Ok(())
    }

    fn observe_running_health_for_update(
        &mut self,
        check: &mut impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(Duration, HostSensorSample), HardwareError> {
        let previous = self
            .last_tick
            .ok_or_else(|| unavailable("missing last observation"))?;
        let now = abortable(check, || self.clock.now())?;
        let (sampled_at, sample) = self.observe_running_health(now, check)?;
        // The entire read-only guard must meet the previous observation's
        // deadline. Only a fully successful observation advances that deadline;
        // the next tick applies the updated policy without writing during update.
        let completed = self.check_gap(previous, check)?;
        if completed < sampled_at {
            return Err(invalid(
                "monotonic clock moved backwards during update observation",
            ));
        }
        self.last_tick = Some(completed);
        self.last_time = Some(completed);
        Ok((completed, sample))
    }

    pub fn tick(&mut self, now: Duration) -> Result<(), HardwareError> {
        self.tick_with_abort(now, || Ok(()))
    }

    fn tick_with_abort(
        &mut self,
        now: Duration,
        mut check_abort: impl FnMut() -> Result<(), HardwareError>,
    ) -> Result<(), HardwareError> {
        self.require_running()?;
        let result = (|| {
            // Observe all channels and old-policy guards before any target writes.
            let (sample_time, sample) = self.observe_running_health(now, &mut check_abort)?;
            let previous = self.last_tick.expect("running tick has a deadline");
            let mut duties = Vec::with_capacity(self.active.len());
            for channel in &self.active {
                let temperature =
                    required_temperature(channel.policy.curve.source, sample, sample_time)?;
                let direct = evaluate_curve(&channel.policy.curve, temperature);
                let next = if direct >= channel.duty_millipercent {
                    direct
                } else {
                    evaluate_curve(
                        &channel.policy.curve,
                        temperature.saturating_add(HOST_CONTROL_DOWNWARD_HYSTERESIS_MILLIDEGREES),
                    )
                    .min(channel.duty_millipercent)
                };
                duties.push(next);
            }
            for (index, duty) in duties.iter().copied().enumerate() {
                let number = self.active[index].config.pwm_channel;
                let target = duty_to_pwm(duty);
                let current = self.active[index].commanded_pwm;
                if target != current {
                    self.check_gap(previous, &mut check_abort)?;
                    check_abort()?;
                    abortable(&mut check_abort, || self.sysfs.write_pwm(number, target))?;
                    abortable(&mut check_abort, || {
                        control::verify_pwm(self.sysfs.as_mut(), number, target)
                    })?;
                    let mode = abortable(&mut check_abort, || self.sysfs.read_enable(number))?;
                    if !control::manual_observed(number, mode, target) {
                        return Err(unavailable(format!("fan{number} manual readback changed")));
                    }
                    self.active[index].commanded_pwm = target;
                    self.check_gap(previous, &mut check_abort)?;
                }
                self.check_gap(previous, &mut check_abort)?;
            }
            for (channel, duty) in self.active.iter_mut().zip(duties) {
                channel.duty_millipercent = duty;
            }
            let completed = self.check_gap(previous, &mut check_abort)?;
            self.last_tick = Some(completed);
            self.last_time = Some(completed);
            Ok(())
        })();
        result.map_err(|error| {
            if is_service_shutdown(&error) {
                self.finalize_after_shutdown(error)
            } else {
                self.finalize_after_error(error)
            }
        })
    }

    pub fn stop(&mut self) -> Result<(), HardwareError> {
        let disarm = self.disarm();
        // Bootstrap can reject a mismatched/corrupt policy before recording
        // which channels need recovery. Unlike automatic shutdown, an explicit
        // Stop must inspect and verify any pending normal record before ACK.
        let pending = if self.recoverable_channels.is_empty() && self.recovery_block.is_none() {
            match self.recovery.load() {
                Ok(Some(record)) => match record.validate_normal() {
                    Ok(()) => {
                        self.recoverable_channels = record.channels;
                        Ok(())
                    }
                    Err(error) => {
                        self.recovery_block = Some(RecoveryBlockReason::LoadFailed);
                        Err(restore_required(format!(
                            "invalid pending recovery on Stop: {error}"
                        )))
                    }
                },
                Ok(None) => Ok(()),
                Err(error) => {
                    self.recovery_block = Some(RecoveryBlockReason::LoadFailed);
                    Err(restore_required(format!(
                        "pending recovery could not be loaded on Stop: {error}"
                    )))
                }
            }
        } else {
            Ok(())
        };
        let restore = pending.and_then(|()| self.finalize());
        if disarm.is_err() || restore.is_err() {
            self.state = HostControlState::RestoreRequired;
        }
        disarm.and(restore)
    }

    pub fn shutdown(&mut self) -> Result<(), HardwareError> {
        // Merely loading a pending record does not authorize automatic writes.
        // A valid, matching Enabled intent must have been verified by bootstrap
        // or committed by Start in this service process.
        if !self.restore_authorized {
            if self.state == HostControlState::RestoreRequired {
                return Err(restore_required(
                    "unresolved recovery without authorized service intent",
                ));
            }
            return Ok(());
        }
        self.finalize()
    }

    /// Explicit restoration path after durable disarm, or when this service
    /// previously verified matching enabled intent. Every channel is attempted.
    pub fn finalize(&mut self) -> Result<(), HardwareError> {
        if self.recoverable_channels.is_empty() {
            match self.recovery_block {
                Some(RecoveryBlockReason::PersistUncertain) => {
                    self.state = HostControlState::RestoreRequired;
                    return Err(restore_required(
                        "recovery persistence is uncertain; restart or standalone restoration is required",
                    ));
                }
                Some(RecoveryBlockReason::LoadFailed) => match self.recovery.load() {
                    Ok(Some(record)) => {
                        if let Err(error) = record.validate_normal() {
                            self.state = HostControlState::RestoreRequired;
                            return Err(restore_required(format!(
                                "recovery state validation still fails: {error}"
                            )));
                        }
                        self.recoverable_channels = record.channels;
                        self.recovery_block = None;
                    }
                    Ok(None) => {
                        self.recovery_block = None;
                        self.set_inactive_state();
                        return Ok(());
                    }
                    Err(error) => {
                        self.state = HostControlState::RestoreRequired;
                        return Err(restore_required(format!(
                            "recovery state still cannot be loaded safely: {error}"
                        )));
                    }
                },
                None => {
                    self.set_inactive_state();
                    return Ok(());
                }
            }
        }
        self.state = HostControlState::Restoring;
        let channels = self.recoverable_channels.clone();

        if let Err(error) = self.sysfs.discover_exact() {
            self.active.clear();
            self.last_tick = None;
            self.last_time = None;
            self.state = HostControlState::RestoreRequired;
            return Err(restore_required(format!(
                "host-control restoration discovery failed: {error}"
            )));
        }

        let mut first_error = None;
        restore_channels_to_bios(self.sysfs.as_mut(), &channels, &mut first_error);

        if first_error.is_none() && !self.policy_uncertain {
            collect_result(&mut first_error, self.recovery.remove());
        }
        if self.policy_uncertain {
            collect_result(
                &mut first_error,
                Err(restore_required(
                    "policy durability is uncertain; retaining recovery evidence",
                )),
            );
        }
        self.active.clear();
        self.active_policy = None;
        self.last_tick = None;
        self.last_time = None;
        if let Some(error) = first_error {
            self.state = HostControlState::RestoreRequired;
            Err(restore_required(format!(
                "host-control restoration did not complete: {error}"
            )))
        } else {
            self.recoverable_channels.clear();
            self.recovery_block = None;
            self.set_inactive_state();
            Ok(())
        }
    }

    fn set_inactive_state(&mut self) {
        self.restore_authorized = false;
        self.active.clear();
        self.active_policy = None;
        self.last_tick = None;
        self.last_time = None;
        self.state = if self.config.is_some() {
            HostControlState::Available
        } else {
            HostControlState::Disabled
        };
    }

    fn finalize_after_shutdown(&mut self, initiating: HardwareError) -> HardwareError {
        match self.finalize() {
            Ok(()) => initiating,
            Err(error) => restore_required(format!(
                "service shutdown: {initiating}; restoration: {error}"
            )),
        }
    }

    fn finalize_after_error(&mut self, initiating: HardwareError) -> HardwareError {
        let disarm = self.disarm();
        let restore = self.finalize();
        if disarm.is_err() {
            self.state = HostControlState::RestoreRequired;
        }
        match disarm.and(restore) {
            Ok(()) => initiating,
            Err(restoration) => restore_required(format!(
                "initiating failure: {}; restoration failure: {}",
                initiating.message(),
                restoration.message()
            )),
        }
    }

    fn require_running(&self) -> Result<(), HardwareError> {
        (self.state == HostControlState::Running)
            .then_some(())
            .ok_or_else(|| unavailable("host control is not running"))
    }

    fn observe_time(&mut self, now: Duration) -> Result<(), HardwareError> {
        if self.last_time.is_some_and(|previous| now < previous) {
            return Err(invalid("monotonic time moved backwards"));
        }
        self.last_time = Some(now);
        Ok(())
    }
}

/// Permanently stops the service worker. Safe to signal without the manager lock.
#[derive(Clone)]
pub struct HostControlShutdownHandle {
    sender: Option<mpsc::Sender<HostControlCommand>>,
    stopping: Arc<AtomicBool>,
    #[cfg(test)]
    count: Option<Arc<std::sync::atomic::AtomicUsize>>,
}
impl HostControlShutdownHandle {
    #[must_use]
    pub fn disabled() -> Self {
        Self {
            sender: None,
            stopping: Arc::new(AtomicBool::new(false)),
            #[cfg(test)]
            count: None,
        }
    }
    pub fn request_shutdown(&self) {
        if !self.stopping.swap(true, Ordering::AcqRel) {
            #[cfg(test)]
            if let Some(count) = &self.count {
                count.fetch_add(1, Ordering::AcqRel);
            }
            if let Some(sender) = &self.sender {
                let _ = sender.send(HostControlCommand::Wake);
            }
        }
    }
    #[cfg(test)]
    pub fn test_handle() -> (Self, Arc<std::sync::atomic::AtomicUsize>) {
        let count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        (
            Self {
                count: Some(count.clone()),
                ..Self::disabled()
            },
            count,
        )
    }
}
impl Default for HostControlShutdownHandle {
    fn default() -> Self {
        Self::disabled()
    }
}

type HostControlResponse = mpsc::SyncSender<Result<(), HardwareError>>;
enum HostControlCommand {
    Initialize {
        resume_saved: bool,
        response: HostControlResponse,
    },
    Start {
        policy: HostControlPolicy,
        response: HostControlResponse,
    },
    Stop {
        response: HostControlResponse,
    },
    Update {
        channel_policies: Vec<HostChannelPolicy>,
        response: HostControlResponse,
    },
    Wake,
    Shutdown {
        response: HostControlResponse,
    },
}

#[derive(Clone)]
struct PublishedWorkerState {
    snapshot: HostControlSnapshot,
    last_error: Option<HardwareError>,
    termination_result: Option<Result<(), HardwareError>>,
}

#[derive(Clone, Copy)]
struct WorkerTiming {
    tick_interval: Duration,
}

impl WorkerTiming {
    const PRODUCTION: Self = Self {
        tick_interval: HOST_CONTROL_TICK_INTERVAL,
    };
}

/// Owns one [`HostControlEngine`] on one standard thread.
///
/// Dropping this value always requests shutdown, finalizes any recoverable
/// session, and joins the thread. It is therefore intentionally not clonable.
pub struct HostControlWorker {
    sender: mpsc::Sender<HostControlCommand>,
    stopping: Arc<AtomicBool>,
    published: Arc<Mutex<PublishedWorkerState>>,
    join: Option<JoinHandle<()>>,
}
impl HostControlWorker {
    #[must_use]
    pub fn new(engine: HostControlEngine) -> Self {
        Self::with_timing(engine, WorkerTiming::PRODUCTION)
    }
    #[cfg(test)]
    pub(crate) fn with_test_tick_interval(
        engine: HostControlEngine,
        tick_interval: Duration,
    ) -> Self {
        Self::with_timing(engine, WorkerTiming { tick_interval })
    }
    fn with_timing(engine: HostControlEngine, timing: WorkerTiming) -> Self {
        let published = Arc::new(Mutex::new(PublishedWorkerState {
            snapshot: engine.snapshot(),
            last_error: None,
            termination_result: None,
        }));
        let (sender, receiver) = mpsc::channel();
        let stopping = Arc::new(AtomicBool::new(false));
        let thread_stopping = stopping.clone();
        let thread_published = published.clone();
        let join = thread::Builder::new()
            .name("nzxt-host-control".into())
            .spawn(move || {
                run_worker(engine, receiver, thread_stopping, thread_published, timing);
            })
            .expect("could not spawn host-control worker");
        Self {
            sender,
            stopping,
            published,
            join: Some(join),
        }
    }
    pub fn initialize(&self) -> Result<(), HardwareError> {
        self.initialize_with_resume(true)
    }
    pub fn initialize_with_resume(&self, resume_saved: bool) -> Result<(), HardwareError> {
        let (response, receiver) = mpsc::sync_channel(1);
        self.request(
            HostControlCommand::Initialize {
                resume_saved,
                response,
            },
            receiver,
        )
    }
    pub fn start(&self, policy: &HostControlPolicy) -> Result<(), HardwareError> {
        let (response, receiver) = mpsc::sync_channel(1);
        self.request(
            HostControlCommand::Start {
                policy: policy.clone(),
                response,
            },
            receiver,
        )
    }
    pub fn update(&self, channel_policies: &[HostChannelPolicy]) -> Result<(), HardwareError> {
        let (response, receiver) = mpsc::sync_channel(1);
        self.request(
            HostControlCommand::Update {
                channel_policies: channel_policies.to_vec(),
                response,
            },
            receiver,
        )
    }
    pub fn stop(&self) -> Result<(), HardwareError> {
        let (response, receiver) = mpsc::sync_channel(1);
        self.request(HostControlCommand::Stop { response }, receiver)
    }
    #[must_use]
    pub fn snapshot(&self) -> HostControlSnapshot {
        lock_published(&self.published).snapshot.clone()
    }
    #[must_use]
    pub fn shutdown_handle(&self) -> HostControlShutdownHandle {
        HostControlShutdownHandle {
            sender: Some(self.sender.clone()),
            stopping: self.stopping.clone(),
            #[cfg(test)]
            count: None,
        }
    }
    pub fn shutdown(&mut self) -> Result<(), HardwareError> {
        self.shutdown_handle().request_shutdown();
        let Some(join) = self.join.take() else {
            return terminal_worker_result(&self.published);
        };
        let (response, receiver) = mpsc::sync_channel(1);
        let result = if self
            .sender
            .send(HostControlCommand::Shutdown { response })
            .is_ok()
        {
            receiver
                .recv()
                .unwrap_or_else(|_| terminal_worker_result(&self.published))
        } else {
            terminal_worker_result(&self.published)
        };
        let joined = join.join().map_err(|panic| {
            restore_required(format!(
                "worker escaped panic guard: {}",
                panic_description(panic.as_ref())
            ))
        });
        result
            .and(joined)
            .and(terminal_worker_result(&self.published))
    }
    fn request(
        &self,
        command: HostControlCommand,
        receiver: mpsc::Receiver<Result<(), HardwareError>>,
    ) -> Result<(), HardwareError> {
        if self.stopping.load(Ordering::Acquire)
            && !matches!(command, HostControlCommand::Stop { .. })
        {
            return Err(service_shutdown_error());
        }
        self.sender
            .send(command)
            .map_err(|_| terminal_worker_error(&self.published))?;
        receiver
            .recv()
            .unwrap_or_else(|_| Err(terminal_worker_error(&self.published)))
    }
}
impl Drop for HostControlWorker {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}

fn run_worker(
    mut engine: HostControlEngine,
    receiver: mpsc::Receiver<HostControlCommand>,
    stopping: Arc<AtomicBool>,
    published: Arc<Mutex<PublishedWorkerState>>,
    timing: WorkerTiming,
) {
    let loop_result = catch_unwind(AssertUnwindSafe(|| {
        worker_loop(&mut engine, &receiver, &stopping, &published, timing)
    }));
    if let Err(panic) = loop_result {
        let message = panic_description(panic.as_ref());
        let result = catch_unwind(AssertUnwindSafe(|| {
            if engine.restore_authorized {
                let disarm = engine.disarm();
                let restore = engine.finalize();
                disarm.and(restore)
            } else {
                engine.shutdown()
            }
        }));
        let error = match result {
            Ok(Ok(())) => restore_required(format!("host-control worker panicked: {message}")),
            Ok(Err(error)) => restore_required(format!(
                "host-control worker panicked: {message}; restoration: {error}"
            )),
            Err(error) => restore_required(format!(
                "host-control worker panicked: {message}; finalizer panicked: {}",
                panic_description(error.as_ref())
            )),
        };
        let mut snapshot = engine.snapshot();
        snapshot.state = HostControlState::RestoreRequired;
        snapshot.active_policy = None;
        publish(&published, snapshot, Some(error.clone()));
        fail_pending_commands(&receiver, &error);
    }
}

fn worker_loop(
    engine: &mut HostControlEngine,
    receiver: &mpsc::Receiver<HostControlCommand>,
    stopping: &AtomicBool,
    published: &Arc<Mutex<PublishedWorkerState>>,
    timing: WorkerTiming,
) {
    let mut next_tick = Instant::now() + timing.tick_interval;
    let mut quiesced = false;
    loop {
        if stopping.load(Ordering::Acquire) && !quiesced {
            // Cancel ongoing Start/tick via their callbacks. Restore once, but
            // retain the receiver: an already accepted Stop may not reach this
            // queue until the server has drained its manager dispatch.
            let result = engine.shutdown();
            update_after_result(engine, published, &result, false);
            quiesced = true;
        }
        if !quiesced && Instant::now() >= next_tick {
            if engine.state == HostControlState::Running {
                let check = || {
                    if stopping.load(Ordering::Acquire) {
                        Err(service_shutdown_error())
                    } else {
                        Ok(())
                    }
                };
                let result = match engine.monotonic_now() {
                    Ok(now) => engine.tick_with_abort(now, check),
                    Err(error) => finalize_control_error(engine, error),
                };
                update_after_result(engine, published, &result, false);
            }
            let now = Instant::now();
            while next_tick <= now {
                next_tick += timing.tick_interval;
            }
        }
        let wait = next_tick.saturating_duration_since(Instant::now());
        let command = if quiesced {
            receiver
                .recv()
                .map_err(|_| mpsc::RecvTimeoutError::Disconnected)
        } else {
            receiver.recv_timeout(wait)
        };
        match command {
            Ok(HostControlCommand::Initialize {
                resume_saved,
                response,
            }) => {
                let result = if stopping.load(Ordering::Acquire) {
                    Err(service_shutdown_error())
                } else {
                    engine.initialize_with_check(resume_saved, || {
                        if stopping.load(Ordering::Acquire) {
                            Err(service_shutdown_error())
                        } else {
                            Ok(())
                        }
                    })
                };
                update_after_result(engine, published, &result, false);
                let _ = response.send(result);
            }
            Ok(HostControlCommand::Start { policy, response }) => {
                let check = || {
                    if stopping.load(Ordering::Acquire) {
                        Err(service_shutdown_error())
                    } else {
                        Ok(())
                    }
                };
                let result = engine.start_with_abort(&policy, check);
                update_after_result(engine, published, &result, true);
                let _ = response.send(result);
            }
            Ok(HostControlCommand::Update {
                channel_policies,
                response,
            }) => {
                let result = engine.update_with_abort(&channel_policies, || {
                    if stopping.load(Ordering::Acquire) {
                        Err(service_shutdown_error())
                    } else {
                        Ok(())
                    }
                });
                update_after_result(engine, published, &result, true);
                let _ = response.send(result);
            }
            Ok(HostControlCommand::Stop { response }) => {
                let result = engine.stop();
                update_after_result(engine, published, &result, true);
                let _ = response.send(result);
            }
            Ok(HostControlCommand::Wake) => {}
            Ok(HostControlCommand::Shutdown { response }) => {
                let result = engine.shutdown();
                update_after_result(engine, published, &result, false);
                let mut state = lock_published(published);
                state.termination_result = Some(result.clone());
                drop(state);
                let _ = response.send(result);
                return;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                let result = engine.shutdown();
                update_after_result(engine, published, &result, false);
                let mut state = lock_published(published);
                state.termination_result = Some(result);
                return;
            }
        }
    }
}

fn finalize_control_error(
    engine: &mut HostControlEngine,
    initiating: HardwareError,
) -> Result<(), HardwareError> {
    Err(engine.finalize_after_error(initiating))
}

fn update_after_result(
    engine: &HostControlEngine,
    published: &Arc<Mutex<PublishedWorkerState>>,
    result: &Result<(), HardwareError>,
    clear_success: bool,
) {
    let mut state = lock_published(published);
    let prior = state.last_error.as_ref().map(|error| error.to_string());
    if let Err(error) = result {
        // Do not overwrite the guard failure with a subsequent rejected request
        // or quiescence notification. The original reason remains observable.
        if engine.state == HostControlState::Running
            || state.last_error.is_none()
            || (engine.state == HostControlState::RestoreRequired
                && !error.is_service_shutdown()
                && error.message() != "host control is not running")
        {
            state.last_error = Some(error.clone());
        }
    } else if clear_success {
        state.last_error = None;
    }
    let current = state.last_error.as_ref().map(|error| error.to_string());
    if current != prior
        && let Some(message) = &current
    {
        eprintln!("host-control: {}", bounded_error(message));
    }
    let mut snapshot = engine.snapshot();
    snapshot.last_error = current.map(|message| bounded_error(&message));
    state.snapshot = snapshot;
    // A later explicit shutdown publishes termination separately.
}

fn bounded_error(message: &str) -> String {
    let mut end = message.len().min(512);
    while !message.is_char_boundary(end) {
        end -= 1;
    }
    message[..end].to_owned()
}

fn publish(
    published: &Arc<Mutex<PublishedWorkerState>>,
    snapshot: HostControlSnapshot,
    last_error: Option<HardwareError>,
) {
    let mut state = lock_published(published);
    let mut snapshot = snapshot;
    snapshot.last_error = last_error
        .as_ref()
        .map(|error| bounded_error(&error.to_string()));
    if state.snapshot.last_error != snapshot.last_error
        && let Some(message) = &snapshot.last_error
    {
        eprintln!("host-control: {message}");
    }
    state.snapshot = snapshot;
    state.termination_result = last_error.clone().map(Err);
    state.last_error = last_error;
}

fn lock_published(
    published: &Arc<Mutex<PublishedWorkerState>>,
) -> std::sync::MutexGuard<'_, PublishedWorkerState> {
    published
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn terminal_worker_result(
    published: &Arc<Mutex<PublishedWorkerState>>,
) -> Result<(), HardwareError> {
    let state = lock_published(published);
    state
        .termination_result
        .clone()
        .unwrap_or_else(|| state.last_error.clone().map_or_else(|| Ok(()), Err))
}

fn terminal_worker_error(published: &Arc<Mutex<PublishedWorkerState>>) -> HardwareError {
    let state = lock_published(published);
    state.last_error.clone().unwrap_or_else(|| {
        HardwareError::with_kind(
            HardwareErrorKind::RestoreRequired,
            "host-control worker terminated unexpectedly",
        )
    })
}

fn fail_pending_commands(receiver: &mpsc::Receiver<HostControlCommand>, error: &HardwareError) {
    while let Ok(command) = receiver.try_recv() {
        let response = match command {
            HostControlCommand::Initialize { response, .. }
            | HostControlCommand::Start { response, .. }
            | HostControlCommand::Stop { response }
            | HostControlCommand::Update { response, .. }
            | HostControlCommand::Shutdown { response } => Some(response),
            HostControlCommand::Wake => None,
        };
        if let Some(response) = response {
            let _ = response.send(Err(error.clone()));
        }
    }
}

fn panic_description(panic: &(dyn Any + Send)) -> &str {
    panic
        .downcast_ref::<&'static str>()
        .copied()
        .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
        .unwrap_or("non-string panic")
}

/// Restores directly from the fixed-format recovery record. No configuration,
/// sensors, NVML, or module operations are involved.
pub fn restore_from_record(
    sysfs: &mut dyn HostControlSysfs,
    recovery: &mut dyn RecoveryStore,
) -> Result<(), HardwareError> {
    let Some(record) = recovery.load().map_err(|error| {
        restore_required(format!(
            "host-control recovery state could not be loaded: {error}"
        ))
    })?
    else {
        return Ok(());
    };
    record.validate_normal().map_err(|error| {
        restore_required(format!(
            "host-control recovery state validation failed: {error}"
        ))
    })?;
    sysfs.discover_exact().map_err(|error| {
        restore_required(format!(
            "host-control restoration discovery failed: {error}"
        ))
    })?;

    let mut first_error = None;
    restore_channels_to_bios(sysfs, &record.channels, &mut first_error);
    if let Some(error) = first_error {
        return Err(restore_required(format!(
            "host-control restoration did not complete: {error}"
        )));
    }
    recovery.remove().map_err(|error| {
        restore_required(format!(
            "host-control restoration record removal did not complete: {error}"
        ))
    })
}

/// Explicit operator restore disarms intent before attempting motherboard writes.
/// If disarming is uncertain, BIOS restoration is best effort but recovery
/// evidence is never removed and success is never reported.
pub fn restore_production() -> Result<(), HardwareError> {
    let _lock = HostControlLock::acquire_restore()?;
    let mut store = FilePolicyStore::new();
    let mut recovery = FileRecoveryStore::new();
    let record = recovery.load()?;
    if let Some(record) = &record {
        record.validate_normal()?;
    }
    if record.is_none() && store.load()?.is_none() {
        return Ok(());
    }
    if let Err(error) = store.persist(&SavedPolicy::Disabled { version: 1 }) {
        if let Some(record) = record {
            let mut sysfs = It8689Sysfs::new();
            sysfs.discover_exact()?;
            let mut restoration_error = None;
            restore_channels_to_bios(&mut sysfs, &record.channels, &mut restoration_error);
        }
        return Err(restore_required(format!(
            "policy disarm uncertain; recovery retained: {error}"
        )));
    }
    restore_from_record(&mut It8689Sysfs::new(), &mut recovery)
}

/// Crash backstop run after service exit. Never changes saved Enabled intent.
pub fn recover_service_production() -> Result<(), HardwareError> {
    let _lock = HostControlLock::acquire_restore()?;
    let mut recovery = FileRecoveryStore::new();
    let record = recovery.load()?;
    let Some(record) = record else {
        return Ok(());
    };
    record.validate_normal()?;
    let saved = FilePolicyStore::new().load()?;
    let Some(SavedPolicy::Enabled {
        complete_policy,
        exact_config_identity,
        ..
    }) = saved
    else {
        return Err(restore_required(
            "pending recovery lacks valid enabled policy",
        ));
    };
    let config = HardwareConfig::load_default().map_err(|error| {
        restore_required(format!(
            "cannot validate saved policy configuration: {error}"
        ))
    })?;
    let Some(host_control) = config.host_control else {
        return Err(restore_required(
            "pending recovery lacks configured host control",
        ));
    };
    let actual = ConfigIdentity {
        host_control,
        nvidia_uuid: config.nvidia_uuid,
        nvidia_pci_bus_id: config.nvidia_pci_bus_id,
    };
    if exact_config_identity != actual {
        return Err(restore_required(
            "saved policy configuration identity changed",
        ));
    }
    validate_config_identity(&actual.host_control)?;
    validate_policy(
        &actual.host_control,
        actual.nvidia_uuid.as_deref(),
        &complete_policy,
    )?;
    record.validate_for_resume(&actual.host_control)?;
    restore_from_record(&mut It8689Sysfs::new(), &mut recovery)
}

fn validate_config_identity(config: &HostControlConfig) -> Result<(), HardwareError> {
    config
        .validate()
        .map_err(|error| invalid(format!("invalid host-control configuration: {error}")))
}

fn validate_policy<'a>(
    config: &HostControlConfig,
    nvidia_uuid: Option<&str>,
    policy: &'a HostControlPolicy,
) -> Result<Vec<&'a HostChannelPolicy>, HardwareError> {
    if policy.channels.len() != config.channels.len() {
        return Err(invalid(
            "complete host policy must contain every configured channel exactly once",
        ));
    }
    let configured = config
        .channels
        .iter()
        .map(|channel| (&channel.id, channel))
        .collect::<HashMap<_, _>>();
    let mut supplied = HashMap::new();
    for channel in &policy.channels {
        if supplied.insert(&channel.channel_id, channel).is_some() {
            return Err(invalid(format!(
                "host policy channel {:?} is duplicated",
                channel.channel_id.0
            )));
        }
        let Some(candidate) = configured.get(&channel.channel_id) else {
            return Err(invalid(format!(
                "host policy channel {:?} is not configured",
                channel.channel_id.0
            )));
        };
        validate_curve(&channel.curve, candidate.minimum_duty_percent, nvidia_uuid)?;
    }
    config
        .channels
        .iter()
        .map(|channel| {
            supplied.get(&channel.id).copied().ok_or_else(|| {
                invalid(format!("host policy is missing channel {:?}", channel.id.0))
            })
        })
        .collect()
}

fn validate_curve(
    curve: &HostCurve,
    minimum: u8,
    nvidia_uuid: Option<&str>,
) -> Result<(), HardwareError> {
    if matches!(
        curve.source,
        HostTemperatureSource::Gpu | HostTemperatureSource::CpuGpuMax
    ) && nvidia_uuid.is_none_or(str::is_empty)
    {
        return Err(invalid(
            "GPU host temperature sources require a configured NVIDIA UUID",
        ));
    }
    validate_curve_shape(curve, minimum)
}

/// Curve shape constraints independent of temperature-source availability.
fn validate_curve_shape(curve: &HostCurve, minimum: u8) -> Result<(), HardwareError> {
    if !(2..=64).contains(&curve.points.len()) {
        return Err(invalid("host curve must contain 2..=64 points"));
    }
    let mut previous_temperature = None;
    let mut previous_duty = None;
    for point in &curve.points {
        if !(0..=120_000).contains(&point.temperature_millidegrees) {
            return Err(invalid("host curve temperatures must be in 0..=120000 m°C"));
        }
        if previous_temperature.is_some_and(|previous| point.temperature_millidegrees <= previous) {
            return Err(invalid(
                "host curve temperatures must be strictly increasing",
            ));
        }
        if !(minimum..=100).contains(&point.duty_percent) {
            return Err(invalid(format!(
                "host curve duties must be in {minimum}..=100 percent"
            )));
        }
        if previous_duty.is_some_and(|previous| point.duty_percent < previous) {
            return Err(invalid("host curve duties must be nondecreasing"));
        }
        previous_temperature = Some(point.temperature_millidegrees);
        previous_duty = Some(point.duty_percent);
    }
    if previous_duty != Some(100) {
        return Err(invalid("host curve final duty must be exactly 100 percent"));
    }
    Ok(())
}

pub(crate) fn required_temperature(
    source: HostTemperatureSource,
    sample: HostSensorSample,
    now: Duration,
) -> Result<i64, HardwareError> {
    let get = |value: Option<TimedTemperature>, name: &str| {
        let value =
            value.ok_or_else(|| unavailable(format!("required {name} sensor is unavailable")))?;
        if value.sampled_at > now {
            return Err(invalid(format!(
                "required {name} sensor timestamp is in the future"
            )));
        }
        if now - value.sampled_at > HOST_CONTROL_SENSOR_FRESHNESS {
            return Err(unavailable(format!(
                "required {name} sensor value is stale"
            )));
        }
        if !(MIN_SENSOR_MILLIDEGREES..=MAX_SENSOR_MILLIDEGREES).contains(&value.millidegrees) {
            return Err(invalid(format!(
                "required {name} sensor value is out of range"
            )));
        }
        Ok(value.millidegrees)
    };
    match source {
        HostTemperatureSource::Cpu => get(sample.cpu, "CPU"),
        HostTemperatureSource::Gpu => get(sample.gpu, "GPU"),
        HostTemperatureSource::CpuGpuMax => {
            Ok(get(sample.cpu, "CPU")?.max(get(sample.gpu, "GPU")?))
        }
    }
}

/// Returns duty in thousandths of one percent so interpolation remains fixed
/// point through PWM conversion.
pub(crate) fn evaluate_curve(curve: &HostCurve, temperature: i64) -> i64 {
    let first = curve.points.first().expect("validated curve");
    if temperature <= i64::from(first.temperature_millidegrees) {
        return i64::from(first.duty_percent) * 1_000;
    }
    for points in curve.points.windows(2) {
        let low = points[0];
        let high = points[1];
        if temperature <= i64::from(high.temperature_millidegrees) {
            let temperature_span =
                i64::from(high.temperature_millidegrees - low.temperature_millidegrees);
            let offset = temperature - i64::from(low.temperature_millidegrees);
            let duty_span = i64::from(high.duty_percent - low.duty_percent) * 1_000;
            return i64::from(low.duty_percent) * 1_000 + duty_span * offset / temperature_span;
        }
    }
    i64::from(curve.points.last().expect("validated curve").duty_percent) * 1_000
}

pub(crate) fn duty_to_pwm(duty_millipercent: i64) -> u8 {
    let numerator = duty_millipercent * 255;
    u8::try_from((numerator + 100_000 - 1) / 100_000).expect("validated duty maps to PWM")
}

fn abortable<T>(
    check_abort: &mut impl FnMut() -> Result<(), HardwareError>,
    operation: impl FnOnce() -> Result<T, HardwareError>,
) -> Result<T, HardwareError> {
    check_abort()?;
    // A real hardware failure wins if shutdown races completion. Never mask
    // a failing sensor/driver operation with the global cancellation signal.
    let result = operation()?;
    check_abort()?;
    Ok(result)
}

fn restore_channels_to_bios(
    sysfs: &mut dyn HostControlSysfs,
    channels: &[RecoveryChannel],
    first_error: &mut Option<HardwareError>,
) {
    for entry in channels {
        if let Err(error) = control::restore_one(
            sysfs,
            entry.pwm_channel,
            entry.original_pwm,
            entry.controlled == Some(true),
        ) && first_error.is_none()
        {
            *first_error = Some(error);
        }
    }
}

fn collect_result(first_error: &mut Option<HardwareError>, result: Result<(), HardwareError>) {
    if let Err(error) = result
        && first_error.is_none()
    {
        *first_error = Some(error);
    }
}

fn invalid(message: impl Into<String>) -> HardwareError {
    HardwareError::with_kind(HardwareErrorKind::InvalidData, message)
}

fn unavailable(message: impl Into<String>) -> HardwareError {
    HardwareError::with_kind(HardwareErrorKind::Unavailable, message)
}

fn service_shutdown_error() -> HardwareError {
    HardwareError::service_shutdown()
}
fn is_service_shutdown(error: &HardwareError) -> bool {
    error.is_service_shutdown()
}

fn restore_required(message: impl Into<String>) -> HardwareError {
    HardwareError::with_kind(HardwareErrorKind::RestoreRequired, message)
}

fn io_error(operation: &str, error: io::Error) -> HardwareError {
    let kind = match error.kind() {
        io::ErrorKind::PermissionDenied => HardwareErrorKind::PermissionDenied,
        io::ErrorKind::NotFound => HardwareErrorKind::Unavailable,
        _ => HardwareErrorKind::Internal,
    };
    HardwareError::with_kind(kind, format!("{operation}: {error}"))
}

#[cfg(test)]
mod tests {
    use std::{
        collections::{BTreeMap, VecDeque},
        ffi::CString,
        os::unix::{
            ffi::OsStrExt,
            fs::{PermissionsExt, symlink},
        },
        sync::{Arc, Condvar, Mutex},
    };

    use nzxt_cam_core::{ChannelId, HostCurvePoint};
    use tempfile::TempDir;

    use super::*;

    #[derive(Default)]
    struct FakeState {
        log: Vec<String>,
        fail_at: Vec<usize>,
        pwm: BTreeMap<u8, u8>,
        enable: BTreeMap<u8, u8>,
        fan: BTreeMap<u8, u64>,
        record: Option<RecoveryRecord>,
        samples: VecDeque<HostSensorSample>,
        last_sample: HostSensorSample,
        now: Duration,
        panic_sample: bool,
        panic_writes: bool,
        fail_restore_writes: bool,
    }

    impl FakeState {
        fn operation(&mut self, operation: impl Into<String>) -> Result<(), HardwareError> {
            self.log.push(operation.into());
            if let Some(position) = self
                .fail_at
                .iter()
                .position(|operation| *operation == self.log.len())
            {
                self.fail_at.remove(position);
                return Err(HardwareError::new("injected failure"));
            }
            Ok(())
        }
    }

    struct FailOnce {
        point: RecoveryFaultPoint,
        fired: bool,
    }

    impl FailOnce {
        fn at(point: RecoveryFaultPoint) -> Self {
            Self {
                point,
                fired: false,
            }
        }
    }

    impl RecoveryFaultInjector for FailOnce {
        fn after(&mut self, point: RecoveryFaultPoint) -> Result<(), HardwareError> {
            if !self.fired && point == self.point {
                self.fired = true;
                return Err(HardwareError::new(format!(
                    "injected recovery fault after {point:?}"
                )));
            }
            Ok(())
        }
    }

    struct TestRecoveryFs {
        fail_before: Option<RecoveryFsOperation>,
        invalid_metadata: Option<RecoveryMetadata>,
        log: Arc<Mutex<Vec<RecoveryFsOperation>>>,
    }

    impl TestRecoveryFs {
        fn failing(point: RecoveryFsOperation) -> Self {
            Self {
                fail_before: Some(point),
                invalid_metadata: None,
                log: Arc::new(Mutex::new(Vec::new())),
            }
        }

        fn with_metadata(
            metadata: RecoveryMetadata,
            log: Arc<Mutex<Vec<RecoveryFsOperation>>>,
        ) -> Self {
            Self {
                fail_before: None,
                invalid_metadata: Some(metadata),
                log,
            }
        }

        fn before(&mut self, point: RecoveryFsOperation) -> io::Result<()> {
            self.log.lock().unwrap().push(point);
            if self.fail_before == Some(point) {
                self.fail_before = None;
                return Err(io::Error::new(
                    io::ErrorKind::PermissionDenied,
                    format!("injected filesystem failure before {point:?}"),
                ));
            }
            Ok(())
        }
    }

    impl RecoveryFsOps for TestRecoveryFs {
        fn write_temp(
            &mut self,
            operation: RecoveryFsOperation,
            file: &mut File,
            bytes: &[u8],
        ) -> io::Result<()> {
            self.before(operation)?;
            StdRecoveryFs.write_temp(operation, file, bytes)
        }

        fn set_file_mode(
            &mut self,
            operation: RecoveryFsOperation,
            file: &File,
            mode: u32,
        ) -> io::Result<()> {
            self.before(operation)?;
            StdRecoveryFs.set_file_mode(operation, file, mode)
        }

        fn file_metadata(
            &mut self,
            operation: RecoveryFsOperation,
            file: &File,
        ) -> io::Result<RecoveryMetadata> {
            self.before(operation)?;
            if let Some(metadata) = self.invalid_metadata {
                return Ok(metadata);
            }
            StdRecoveryFs.file_metadata(operation, file)
        }

        fn sync_file(&mut self, operation: RecoveryFsOperation, file: &File) -> io::Result<()> {
            self.before(operation)?;
            StdRecoveryFs.sync_file(operation, file)
        }

        fn rename(
            &mut self,
            operation: RecoveryFsOperation,
            from: &Path,
            to: &Path,
        ) -> io::Result<()> {
            self.before(operation)?;
            StdRecoveryFs.rename(operation, from, to)
        }

        fn sync_parent(&mut self, operation: RecoveryFsOperation, parent: &Path) -> io::Result<()> {
            self.before(operation)?;
            StdRecoveryFs.sync_parent(operation, parent)
        }

        fn unlink_marker(
            &mut self,
            operation: RecoveryFsOperation,
            marker: &Path,
        ) -> io::Result<()> {
            self.before(operation)?;
            StdRecoveryFs.unlink_marker(operation, marker)
        }
    }

    #[derive(Clone)]
    struct FakeSysfs(Arc<Mutex<FakeState>>);

    impl HostControlSysfs for FakeSysfs {
        fn discover_exact(&mut self) -> Result<(), HardwareError> {
            self.0.lock().unwrap().operation("discover")
        }

        fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError> {
            let mut state = self.0.lock().unwrap();
            state.operation(format!("read_pwm:{channel}"))?;
            state
                .pwm
                .get(&channel)
                .copied()
                .ok_or_else(|| unavailable("missing fake PWM"))
        }

        fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError> {
            let mut state = self.0.lock().unwrap();
            state.operation(format!("read_fan:{channel}"))?;
            state
                .fan
                .get(&channel)
                .copied()
                .ok_or_else(|| unavailable("missing fake fan"))
        }

        fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError> {
            let mut state = self.0.lock().unwrap();
            state.operation(format!("read_enable:{channel}"))?;
            let mode = state
                .enable
                .get(&channel)
                .copied()
                .ok_or_else(|| unavailable("missing fake mode"))?;
            // Match this kernel driver's mode alias: fan4 in manual mode at
            // PWM 255 reads as full-speed mode 0, not mode 1.
            Ok(
                if channel == 4 && mode == 1 && state.pwm.get(&channel) == Some(&255) {
                    0
                } else {
                    mode
                },
            )
        }

        fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            let mut state = self.0.lock().unwrap();
            if state.panic_writes {
                drop(state);
                panic!("injected finalizer write panic");
            }
            state.operation(format!("write_pwm:{channel}:{value}"))?;
            if state.enable.get(&channel) == Some(&2) {
                return Err(HardwareError::new(
                    "fake IT8689 rejects PWM writes while mode 2 owns the channel",
                ));
            }
            state.pwm.insert(channel, value);
            Ok(())
        }

        fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            let mut state = self.0.lock().unwrap();
            if state.panic_writes {
                drop(state);
                panic!("injected finalizer write panic");
            }
            state.operation(format!("write_enable:{channel}:{value}"))?;
            if value == 2 && state.fail_restore_writes {
                return Err(HardwareError::new(
                    "injected persistent restoration failure",
                ));
            }
            state.enable.insert(channel, value);
            if value == 0 {
                // Model mode 0 forcing full speed and replacing dormant PWM.
                state.pwm.insert(channel, 255);
            }
            Ok(())
        }
    }

    #[derive(Clone)]
    struct FakeRecovery(Arc<Mutex<FakeState>>);

    impl RecoveryStore for FakeRecovery {
        fn load(&mut self) -> Result<Option<RecoveryRecord>, HardwareError> {
            let mut state = self.0.lock().unwrap();
            state.operation("record_load")?;
            Ok(state.record.clone())
        }

        fn persist(&mut self, record: &RecoveryRecord) -> Result<(), HardwareError> {
            let mut state = self.0.lock().unwrap();
            state.operation("record_persist")?;
            if state.record.is_some() {
                return Err(invalid("record exists"));
            }
            state.record = Some(record.clone());
            Ok(())
        }

        fn remove(&mut self) -> Result<(), HardwareError> {
            let mut state = self.0.lock().unwrap();
            state.operation("record_remove")?;
            if state.record.take().is_none() {
                return Err(unavailable("record absent"));
            }
            Ok(())
        }
    }

    #[derive(Clone)]
    struct FakeSensors(Arc<Mutex<FakeState>>);

    impl HostSensorSource for FakeSensors {
        fn sample(&mut self, _now: Duration) -> HostSensorSample {
            let mut state = self.0.lock().unwrap();
            if state.panic_sample {
                drop(state);
                panic!("injected sensor panic");
            }
            // Sampling has no Result in the production seam. A fail-at sample
            // is represented by an unavailable sample.
            if state.operation("sensor_sample").is_err() {
                return HostSensorSample::default();
            }
            if let Some(sample) = state.samples.pop_front() {
                state.last_sample = sample;
            }
            state.last_sample
        }
    }

    #[derive(Clone)]
    struct FakeClock(Arc<Mutex<FakeState>>);

    impl MonotonicTimeSource for FakeClock {
        fn now(&mut self) -> Result<Duration, HardwareError> {
            let mut state = self.0.lock().unwrap();
            state.operation("clock_now")?;
            Ok(state.now)
        }
    }

    struct BlockingGate {
        state: Mutex<BlockingGateState>,
        changed: Condvar,
    }

    struct BlockingGateState {
        target: &'static str,
        entered: bool,
        released: bool,
        used: bool,
    }

    impl BlockingGate {
        fn new(target: &'static str) -> Arc<Self> {
            Arc::new(Self {
                state: Mutex::new(BlockingGateState {
                    target,
                    entered: false,
                    released: false,
                    used: false,
                }),
                changed: Condvar::new(),
            })
        }

        fn hit(&self, operation: &str) {
            let mut state = self.state.lock().unwrap();
            if state.target != operation || state.used {
                return;
            }
            state.used = true;
            state.entered = true;
            self.changed.notify_all();
            while !state.released {
                state = self.changed.wait(state).unwrap();
            }
        }

        fn wait_until_entered(&self) {
            let deadline = Instant::now() + Duration::from_secs(2);
            let mut state = self.state.lock().unwrap();
            while !state.entered {
                let remaining = deadline.saturating_duration_since(Instant::now());
                assert!(!remaining.is_zero(), "blocking fake was never entered");
                let (next, timeout) = self.changed.wait_timeout(state, remaining).unwrap();
                state = next;
                assert!(
                    state.entered || !timeout.timed_out(),
                    "blocking fake timed out"
                );
            }
        }

        fn release(&self) {
            let mut state = self.state.lock().unwrap();
            state.released = true;
            self.changed.notify_all();
        }
    }

    struct BlockingSysfs {
        inner: FakeSysfs,
        gate: Arc<BlockingGate>,
    }

    impl HostControlSysfs for BlockingSysfs {
        fn discover_exact(&mut self) -> Result<(), HardwareError> {
            self.gate.hit("discover");
            self.inner.discover_exact()
        }

        fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError> {
            self.gate.hit(&format!("read_pwm:{channel}"));
            self.inner.read_pwm(channel)
        }

        fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError> {
            self.gate.hit(&format!("read_fan:{channel}"));
            self.inner.read_fan(channel)
        }

        fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError> {
            self.gate.hit(&format!("read_enable:{channel}"));
            self.inner.read_enable(channel)
        }

        fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            self.gate.hit(&format!("write_pwm:{channel}:{value}"));
            self.inner.write_pwm(channel, value)
        }

        fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            self.gate.hit(&format!("write_enable:{channel}:{value}"));
            self.inner.write_enable(channel, value)
        }
    }

    fn config() -> HostControlConfig {
        HostControlConfig {
            board_vendor: BOARD_VENDOR.into(),
            board_name: BOARD_NAME.into(),
            chip_name: CHIP_NAME.into(),
            chip_address: CHIP_ADDRESS,
            platform_component: PLATFORM_COMPONENT.into(),
            channels: vec![
                HostChannelConfig {
                    id: ChannelId::new("front"),
                    name: "Front".into(),
                    pwm_channel: 3,
                    fan_channel: 3,
                    minimum_duty_percent: 30,
                },
                HostChannelConfig {
                    id: ChannelId::new("rear"),
                    name: "Rear".into(),
                    pwm_channel: 4,
                    fan_channel: 4,
                    minimum_duty_percent: 40,
                },
            ],
        }
    }

    fn recovery_record() -> RecoveryRecord {
        RecoveryRecord::from_config(&config(), &[63, 63]).unwrap()
    }

    fn curve(source: HostTemperatureSource, minimum: u8) -> HostCurve {
        HostCurve {
            source,
            points: vec![
                HostCurvePoint {
                    temperature_millidegrees: 20_000,
                    duty_percent: minimum,
                },
                HostCurvePoint {
                    temperature_millidegrees: 40_000,
                    duty_percent: 50,
                },
                HostCurvePoint {
                    temperature_millidegrees: 80_000,
                    duty_percent: 100,
                },
            ],
        }
    }

    fn policy(source: HostTemperatureSource) -> HostControlPolicy {
        HostControlPolicy {
            channels: vec![
                HostChannelPolicy {
                    channel_id: ChannelId::new("front"),
                    curve: curve(source, 30),
                },
                HostChannelPolicy {
                    channel_id: ChannelId::new("rear"),
                    curve: curve(source, 40),
                },
            ],
        }
    }

    fn sample(now: Duration, cpu: Option<i64>, gpu: Option<i64>) -> HostSensorSample {
        HostSensorSample {
            cpu: cpu.map(|millidegrees| TimedTemperature {
                millidegrees,
                sampled_at: now,
            }),
            gpu: gpu.map(|millidegrees| TimedTemperature {
                millidegrees,
                sampled_at: now,
            }),
        }
    }

    fn new_fake_state(record: Option<RecoveryRecord>) -> Arc<Mutex<FakeState>> {
        let now = Duration::from_secs(100);
        Arc::new(Mutex::new(FakeState {
            pwm: [(3, 63), (4, 63)].into(),
            enable: [(3, 2), (4, 2)].into(),
            fan: [(3, 900), (4, 1_000)].into(),
            record,
            last_sample: sample(now, Some(40_000), Some(45_000)),
            now,
            ..FakeState::default()
        }))
    }

    fn harness_with_recovery(
        state: Arc<Mutex<FakeState>>,
        recovery: Box<dyn RecoveryStore>,
    ) -> HostControlEngine {
        HostControlEngine::with_dependencies(
            Some(config()),
            Some("GPU-exact".into()),
            Box::new(FakeSysfs(state.clone())),
            recovery,
            Box::new(FakeSensors(state.clone())),
            Box::new(FakeClock(state)),
            Box::new(MemoryPolicyStore::default()),
        )
    }

    fn harness_with_record(
        record: Option<RecoveryRecord>,
    ) -> (HostControlEngine, Arc<Mutex<FakeState>>) {
        let state = new_fake_state(record);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())));
        if state.lock().unwrap().record.is_some() {
            let _ = engine.initialize();
        }
        (engine, state)
    }

    fn harness() -> (HostControlEngine, Arc<Mutex<FakeState>>) {
        harness_with_record(None)
    }

    fn set_fail_next(state: &Arc<Mutex<FakeState>>, offset: usize) {
        let mut state = state.lock().unwrap();
        let operation = state.log.len() + offset;
        state.fail_at.push(operation);
    }

    fn set_fail_absolute(state: &Arc<Mutex<FakeState>>, operation: usize) {
        state.lock().unwrap().fail_at.push(operation);
    }

    fn assert_finalizer_attempted(log: &[String]) {
        let discovery = log
            .iter()
            .rposition(|entry| entry == "discover")
            .expect("finalizer did not rediscover IT8689");
        let finalizer = &log[discovery + 1..];
        for channel in [3, 4] {
            assert!(
                finalizer
                    .iter()
                    .any(|op| op == &format!("read_enable:{channel}")),
                "independent fan{channel} restoration missing: {finalizer:?}"
            );
        }
        assert!(!finalizer.iter().any(
            |op| op.starts_with("write_enable:") && op.ends_with(":0") || op.ends_with(":255")
        ));
    }

    fn wait_for_worker_state(
        worker: &HostControlWorker,
        expected: HostControlState,
    ) -> HostControlSnapshot {
        for _ in 0..500 {
            let snapshot = worker.snapshot();
            if snapshot.state == expected {
                return snapshot;
            }
            thread::sleep(Duration::from_millis(2));
        }
        panic!(
            "timed out waiting for worker state {expected:?}; current state is {:?}",
            worker.snapshot().state
        );
    }

    #[test]
    fn injected_policy_store_drives_initialization_and_durable_lifecycle() {
        struct RecordingPolicyStore {
            saved: Arc<Mutex<Vec<SavedPolicy>>>,
            calls: Arc<Mutex<Vec<&'static str>>>,
        }
        impl PolicyStore for RecordingPolicyStore {
            fn load(&mut self) -> Result<Option<SavedPolicy>, HardwareError> {
                self.calls.lock().unwrap().push("load");
                Ok(self.saved.lock().unwrap().last().cloned())
            }
            fn persist(&mut self, saved: &SavedPolicy) -> Result<(), HardwareError> {
                self.calls.lock().unwrap().push("persist");
                self.saved.lock().unwrap().push(saved.clone());
                Ok(())
            }
        }

        let selected = policy(HostTemperatureSource::Cpu);
        let identity = ConfigIdentity {
            host_control: config(),
            nvidia_uuid: Some("GPU-exact".into()),
            nvidia_pci_bus_id: None,
        };
        let enabled = SavedPolicy::Enabled {
            version: 1,
            complete_policy: selected.clone(),
            exact_config_identity: identity.clone(),
        };
        let saved = Arc::new(Mutex::new(vec![enabled.clone()]));
        let calls = Arc::new(Mutex::new(Vec::new()));
        let state = new_fake_state(None);
        let mut engine = HostControlEngine::with_dependencies(
            Some(config()),
            Some("GPU-exact".into()),
            Box::new(FakeSysfs(state.clone())),
            Box::new(FakeRecovery(state.clone())),
            Box::new(FakeSensors(state.clone())),
            Box::new(FakeClock(state.clone())),
            Box::new(RecordingPolicyStore {
                saved: saved.clone(),
                calls: calls.clone(),
            }),
        );

        engine.initialize().unwrap();
        assert_eq!(engine.snapshot().active_policy, Some(selected.clone()));
        assert_eq!(*calls.lock().unwrap(), ["load"]);
        assert_eq!(
            saved.lock().unwrap().as_slice(),
            std::slice::from_ref(&enabled)
        );

        let changed = changed_policy("front", HostTemperatureSource::Cpu);
        let mut updated = selected.clone();
        updated.channels[0] = changed.clone();
        engine.update(&[changed]).unwrap();
        let updated_saved = SavedPolicy::Enabled {
            version: 1,
            complete_policy: updated,
            exact_config_identity: identity,
        };
        assert_eq!(saved.lock().unwrap().last(), Some(&updated_saved));

        engine.stop().unwrap();
        assert_eq!(
            saved.lock().unwrap().last(),
            Some(&SavedPolicy::Disabled { version: 1 })
        );
        engine.start(&selected).unwrap();
        engine.shutdown().unwrap();
        assert_eq!(
            *calls.lock().unwrap(),
            ["load", "persist", "persist", "persist"]
        );
        assert_eq!(
            *saved.lock().unwrap(),
            [
                enabled.clone(),
                updated_saved,
                SavedPolicy::Disabled { version: 1 },
                enabled,
            ]
        );
        let fake = state.lock().unwrap();
        assert_eq!(fake.pwm, [(3, 63), (4, 63)].into());
        assert_eq!(fake.enable, [(3, 2), (4, 2)].into());
        assert!(fake.record.is_none());
    }

    #[test]
    fn worker_start_ticks_stops_and_uses_exact_deadline() {
        let (engine, state) = harness();
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_millis(5));
        worker.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.last_sample = sample(fake.now, Some(50_000), Some(45_000));
        }
        for _ in 0..500 {
            if state
                .lock()
                .unwrap()
                .log
                .iter()
                .any(|op| op == "write_pwm:3:160")
            {
                break;
            }
            thread::sleep(Duration::from_millis(2));
        }
        assert!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .any(|op| op == "write_pwm:3:160")
        );
        assert_eq!(state.lock().unwrap().pwm[&3], 160); // direct target, no catch-up burst
        worker.stop().unwrap();
        assert_eq!(worker.snapshot().state, HostControlState::Available);
        assert!(state.lock().unwrap().record.is_none());
        worker.shutdown().unwrap();
    }

    #[test]
    fn failed_worker_stop_can_retry_successfully_and_clears_the_published_error() {
        let (engine, state) = harness();
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3_600));
        worker.start(&policy(HostTemperatureSource::Cpu)).unwrap();

        set_fail_next(&state, 1);
        let first_error = worker.stop().unwrap_err();
        assert_eq!(first_error.kind(), HardwareErrorKind::RestoreRequired);
        assert_eq!(worker.snapshot().state, HostControlState::RestoreRequired);
        assert!(state.lock().unwrap().record.is_some());
        assert!(lock_published(&worker.published).last_error.is_some());

        worker.stop().unwrap();
        assert_eq!(worker.snapshot().state, HostControlState::Available);
        assert!(state.lock().unwrap().record.is_none());
        assert!(lock_published(&worker.published).last_error.is_none());

        worker.stop().unwrap();
        assert_eq!(worker.snapshot().state, HostControlState::Available);
        assert!(lock_published(&worker.published).last_error.is_none());
        worker.shutdown().unwrap();
    }

    #[test]
    fn worker_tick_panic_finalizes_then_publishes_restore_required() {
        let (engine, state) = harness();
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_millis(5));
        worker.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        {
            let mut state = state.lock().unwrap();
            state.now = Duration::from_secs(101);
            state.panic_sample = true;
        }

        let snapshot = wait_for_worker_state(&worker, HostControlState::RestoreRequired);
        assert_eq!(snapshot.channels.len(), 2);
        assert!(state.lock().unwrap().record.is_none());
        let error = worker
            .start(&policy(HostTemperatureSource::Cpu))
            .unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
        assert!(error.message().contains("worker panicked"));
        assert_eq!(
            worker.shutdown().unwrap_err().kind(),
            HardwareErrorKind::RestoreRequired
        );
        assert!(worker.join.is_none());
    }

    #[test]
    fn constants_and_snapshot_states_are_fixed() {
        assert_eq!(HOST_CONTROL_TICK_INTERVAL, Duration::from_secs(1));
        assert_eq!(HOST_CONTROL_SENSOR_FRESHNESS, Duration::from_secs(3));
        assert_eq!(HOST_CONTROL_DOWNWARD_HYSTERESIS_MILLIDEGREES, 2_000);
        assert_eq!(
            HOST_CONTROL_RECOVERY_PATH,
            "/run/nzxt-cam/host-control.json"
        );

        let (engine, _) = harness();
        assert_eq!(engine.snapshot().state, HostControlState::Available);
        assert_eq!(engine.snapshot().channels.len(), 2);

        let (required, _) = harness_with_record(Some(recovery_record()));
        assert_eq!(required.snapshot().state, HostControlState::RestoreRequired);
    }

    #[test]
    fn policy_validation_is_complete_and_bounded_before_hardware_access() {
        let variants = {
            let mut values = Vec::new();
            let mut missing = policy(HostTemperatureSource::Cpu);
            missing.channels.pop();
            values.push(missing);
            let mut extra = policy(HostTemperatureSource::Cpu);
            extra.channels.push(extra.channels[0].clone());
            values.push(extra);
            let mut duplicate = policy(HostTemperatureSource::Cpu);
            duplicate.channels[1].channel_id = ChannelId::new("front");
            values.push(duplicate);
            let mut unknown = policy(HostTemperatureSource::Cpu);
            unknown.channels[1].channel_id = ChannelId::new("unknown");
            values.push(unknown);
            let mut one_point = policy(HostTemperatureSource::Cpu);
            one_point.channels[0].curve.points.pop();
            one_point.channels[0].curve.points.pop();
            values.push(one_point);
            let mut too_many = policy(HostTemperatureSource::Cpu);
            too_many.channels[0].curve.points = (0..65)
                .map(|index| HostCurvePoint {
                    temperature_millidegrees: index * 1_000,
                    duty_percent: if index == 64 { 100 } else { 30 },
                })
                .collect();
            values.push(too_many);
            let mut bad_temperature = policy(HostTemperatureSource::Cpu);
            bad_temperature.channels[0].curve.points[0].temperature_millidegrees = -1;
            values.push(bad_temperature);
            let mut duplicate_temperature = policy(HostTemperatureSource::Cpu);
            duplicate_temperature.channels[0].curve.points[1].temperature_millidegrees = 20_000;
            values.push(duplicate_temperature);
            let mut below_minimum = policy(HostTemperatureSource::Cpu);
            below_minimum.channels[0].curve.points[0].duty_percent = 29;
            values.push(below_minimum);
            let mut decreasing = policy(HostTemperatureSource::Cpu);
            decreasing.channels[0].curve.points[1].duty_percent = 31;
            decreasing.channels[0].curve.points[0].duty_percent = 40;
            values.push(decreasing);
            let mut bad_final = policy(HostTemperatureSource::Cpu);
            bad_final.channels[0]
                .curve
                .points
                .last_mut()
                .unwrap()
                .duty_percent = 99;
            values.push(bad_final);
            values
        };

        for invalid_policy in variants {
            let (mut engine, state) = harness();
            let baseline = state.lock().unwrap().log.len();
            let error = engine.start(&invalid_policy).unwrap_err();
            assert_eq!(error.kind(), HardwareErrorKind::InvalidData);
            assert!(error.message().len() <= 512);
            assert_eq!(state.lock().unwrap().log.len(), baseline);
        }

        let (mut engine, state) = harness();
        engine.nvidia_uuid = None;
        assert!(engine.start(&policy(HostTemperatureSource::Gpu)).is_err());
        assert!(state.lock().unwrap().log.is_empty());
    }

    #[test]
    fn start_read_only_preconditions_fail_without_record_or_mutation() {
        let (mut measured, measured_state) = harness();
        let baseline = measured_state.lock().unwrap().log.len();
        measured.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let operations = measured_state.lock().unwrap().log[baseline..].to_vec();
        let persist = operations
            .iter()
            .position(|operation| operation == "record_persist")
            .unwrap();

        for (index, operation) in operations[..persist].iter().enumerate() {
            let (mut engine, state) = harness();
            let baseline = state.lock().unwrap().log.len();
            set_fail_absolute(&state, baseline + index + 1);
            assert!(
                engine.start(&policy(HostTemperatureSource::Cpu)).is_err(),
                "{operation}"
            );
            let state = state.lock().unwrap();
            assert!(state.record.is_none(), "{operation}");
            assert!(
                state.log.iter().all(|entry| !entry.starts_with("write_")),
                "{operation}"
            );
        }

        let (mut engine, state) = harness();
        state.lock().unwrap().enable.insert(4, 1);
        assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());
        let state = state.lock().unwrap();
        assert!(state.record.is_none());
        assert!(state.log.iter().all(|entry| !entry.starts_with("write_")));
    }

    #[test]
    fn start_is_ordered_and_never_mutates_before_durable_record() {
        let (mut engine, state) = harness();
        engine
            .start(&policy(HostTemperatureSource::CpuGpuMax))
            .unwrap();
        {
            let state = state.lock().unwrap();
            let persist = state
                .log
                .iter()
                .position(|op| op == "record_persist")
                .unwrap();
            let writes: Vec<_> = state
                .log
                .iter()
                .enumerate()
                .filter(|(_, op)| op.starts_with("write_"))
                .collect();
            assert!(writes[0].0 > persist);
            assert_eq!(
                writes.iter().map(|(_, op)| op.as_str()).collect::<Vec<_>>(),
                [
                    "write_enable:3:1",
                    "write_pwm:3:128",
                    "write_enable:4:1",
                    "write_pwm:4:128"
                ]
            );
            assert_eq!(state.pwm, [(3, 128), (4, 128)].into());
            assert_eq!(
                state.record.as_ref().unwrap().version,
                NORMAL_RECOVERY_VERSION
            );
            assert_eq!(
                state
                    .record
                    .as_ref()
                    .unwrap()
                    .channels
                    .iter()
                    .map(|c| (c.original_pwm, c.controlled))
                    .collect::<Vec<_>>(),
                vec![(63, Some(true)); 2]
            );
        }
        engine.stop().unwrap();
        let state = state.lock().unwrap();
        assert_eq!(state.pwm, [(3, 63), (4, 63)].into());
        assert_eq!(state.enable, [(3, 2), (4, 2)].into());
        assert!(state.record.is_none());
        assert!(!state.log.iter().any(
            |op| op.starts_with("write_enable:") && op.ends_with(":0") || op.ends_with(":255")
        ));
    }

    #[test]
    fn exhaustive_start_operations_fail_conservatively_at_measured_boundaries() {
        let (mut successful, successful_state) = harness();
        let successful_baseline = successful_state.lock().unwrap().log.len();
        successful
            .start(&policy(HostTemperatureSource::Cpu))
            .unwrap();
        let operations = successful_state.lock().unwrap().log[successful_baseline..].to_vec();
        let persist = operations
            .iter()
            .position(|operation| operation == "record_persist")
            .unwrap();

        for (index, operation) in operations.iter().enumerate() {
            let (mut engine, state) = harness();
            let baseline = state.lock().unwrap().log.len();
            set_fail_absolute(&state, baseline + index + 1);
            let error = engine
                .start(&policy(HostTemperatureSource::Cpu))
                .expect_err(operation);
            let state = state.lock().unwrap();
            let first_write = state
                .log
                .iter()
                .position(|entry| entry.starts_with("write_"));

            if index <= persist {
                assert!(
                    first_write.is_none(),
                    "wrote before durable persist: {operation}"
                );
                assert!(state.record.is_none());
                if index == persist {
                    assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
                    assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
                }
            } else {
                assert_ne!(error.kind(), HardwareErrorKind::RestoreRequired);
                assert_finalizer_attempted(&state.log);
                assert!(state.record.is_none());
                assert_eq!(state.enable, [(3, 2), (4, 2)].into());
                assert_eq!(state.pwm, [(3, 63), (4, 63)].into());
                assert_eq!(engine.snapshot().state, HostControlState::Available);
            }
        }
    }

    #[test]
    fn exhaustive_tick_operations_finalize_at_measured_boundaries() {
        let (mut successful, successful_state) = harness();
        successful
            .start(&policy(HostTemperatureSource::Cpu))
            .unwrap();
        let now = Duration::from_secs(101);
        let successful_baseline = successful_state.lock().unwrap().log.len();
        {
            let mut fake = successful_state.lock().unwrap();
            fake.now = now;
            fake.last_sample = sample(now, Some(60_000), Some(45_000));
        }
        successful.tick(now).unwrap();
        let operations = successful_state.lock().unwrap().log[successful_baseline..].to_vec();
        // Force both targets to change: a steady 40°C tick would otherwise
        // measure only reads and never inject faults into the target writes.
        for number in [3, 4] {
            let write = operations
                .iter()
                .position(|op| op == &format!("write_pwm:{number}:192"))
                .unwrap();
            assert!(operations[write + 1..].contains(&format!("read_pwm:{number}")));
            assert!(operations[write + 1..].contains(&format!("read_enable:{number}")));
        }

        for (index, operation) in operations.iter().enumerate() {
            let (mut engine, state) = harness();
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            {
                let mut fake = state.lock().unwrap();
                fake.now = now;
                fake.last_sample = sample(now, Some(60_000), Some(45_000));
            }
            let baseline = state.lock().unwrap().log.len();
            set_fail_absolute(&state, baseline + index + 1);
            let error = engine.tick(now).expect_err(operation);
            let state = state.lock().unwrap();
            assert_ne!(error.kind(), HardwareErrorKind::RestoreRequired);
            assert_finalizer_attempted(&state.log);
            assert!(state.record.is_none());
            assert_eq!(state.pwm, [(3, 63), (4, 63)].into(), "{operation}");
            assert_eq!(state.enable, [(3, 2), (4, 2)].into(), "{operation}");
            assert_eq!(engine.snapshot().state, HostControlState::Available);
            assert!(engine.snapshot().active_policy.is_none());
        }

        assert!(!operations.iter().any(|op| op.ends_with(":255")
            || op == "write_enable:3:0"
            || op == "write_enable:4:0"));
    }

    #[test]
    fn exhaustive_finalizer_failures_use_measured_operations_and_retain_record() {
        let (mut successful, successful_state) = harness();
        successful
            .start(&policy(HostTemperatureSource::Cpu))
            .unwrap();
        let successful_baseline = successful_state.lock().unwrap().log.len();
        successful.stop().unwrap();
        let operations = successful_state.lock().unwrap().log[successful_baseline..].to_vec();

        for (index, operation) in operations.iter().enumerate() {
            let (mut engine, state) = harness();
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            let baseline = state.lock().unwrap().log.len();
            set_fail_absolute(&state, baseline + index + 1);
            let error = engine.stop().expect_err(operation);
            assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
            let state = state.lock().unwrap();
            let final_log = &state.log[baseline..];
            if operation != "discover" {
                let other = if operation.contains(":3") { 4 } else { 3 };
                assert!(
                    final_log
                        .iter()
                        .any(|op| op == &format!("read_enable:{other}")),
                    "independent other fan not inspected after {operation}: {final_log:?}"
                );
                assert_finalizer_attempted(&state.log);
            }
            assert!(state.record.is_some());
            assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        }
    }

    #[test]
    fn failed_restore_blocks_start_but_retrying_common_finalizer_can_recover() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        set_fail_next(&state, 1);
        assert!(engine.stop().is_err());
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());
        assert!(state.lock().unwrap().record.is_some());

        engine.stop().unwrap();
        assert_eq!(engine.snapshot().state, HostControlState::Available);
        assert!(state.lock().unwrap().record.is_none());
    }

    #[test]
    fn gpu_policy_still_requires_independently_healthy_cpu_at_start_tick_and_update() {
        for (name, cpu) in [
            ("missing", None),
            ("stale", Some(40_000)),
            ("implausible", Some(151_000)),
        ] {
            let bad_cpu = |now: Duration| {
                let mut reading = sample(now, cpu, Some(45_000));
                if name == "stale" {
                    reading.cpu.as_mut().unwrap().sampled_at = now - Duration::from_secs(4);
                }
                reading
            };
            let (mut engine, state) = harness();
            state.lock().unwrap().last_sample = bad_cpu(Duration::from_secs(100));
            assert!(
                engine.start(&policy(HostTemperatureSource::Gpu)).is_err(),
                "{name} start"
            );
            let fake = state.lock().unwrap();
            assert!(fake.record.is_none(), "{name} start");
            assert!(
                !fake.log.iter().any(|op| op.starts_with("write_")),
                "{name} start"
            );

            for operation in ["tick", "update"] {
                let (mut engine, state) = harness();
                engine.start(&policy(HostTemperatureSource::Gpu)).unwrap();
                {
                    let mut fake = state.lock().unwrap();
                    fake.now = Duration::from_secs(101);
                    fake.last_sample = bad_cpu(fake.now);
                }
                let error = if operation == "tick" {
                    engine.tick(Duration::from_secs(101)).unwrap_err()
                } else {
                    engine
                        .update(&[changed_policy("front", HostTemperatureSource::Gpu)])
                        .unwrap_err()
                };
                assert!(
                    error.message().contains("CPU sensor"),
                    "{name} {operation}: {error}"
                );
                assert_eq!(
                    engine.snapshot().state,
                    HostControlState::Available,
                    "{name} {operation}"
                );
                let fake = state.lock().unwrap();
                assert!(fake.record.is_none(), "{name} {operation}");
                assert_eq!(fake.enable, [(3, 2), (4, 2)].into(), "{name} {operation}");
            }
        }
    }

    #[test]
    fn sensor_missing_stale_future_and_max_failures_restore() {
        let cases = [
            (
                HostTemperatureSource::Cpu,
                sample(Duration::from_secs(101), None, Some(50_000)),
            ),
            (
                HostTemperatureSource::Gpu,
                sample(Duration::from_secs(101), Some(50_000), None),
            ),
            (
                HostTemperatureSource::CpuGpuMax,
                sample(Duration::from_secs(101), Some(50_000), None),
            ),
            (
                HostTemperatureSource::Cpu,
                sample(Duration::from_secs(97), Some(50_000), Some(50_000)),
            ),
            (
                HostTemperatureSource::Gpu,
                sample(Duration::from_secs(102), Some(50_000), Some(50_000)),
            ),
        ];
        for (source, bad_sample) in cases {
            let (mut engine, state) = harness();
            engine.start(&policy(source)).unwrap();
            state.lock().unwrap().last_sample = bad_sample;
            assert!(engine.tick(Duration::from_secs(101)).is_err());
            let state = state.lock().unwrap();
            assert_finalizer_attempted(&state.log);
            assert!(state.record.is_none());
        }
    }

    #[test]
    fn interpolation_ceiling_and_downward_hysteresis_are_fixed_point() {
        assert_eq!(duty_to_pwm(35_000), 90);
        let (mut engine, state) = harness();
        let selected = policy(HostTemperatureSource::Cpu);
        engine.start(&selected).unwrap();
        assert_eq!(state.lock().unwrap().pwm[&3], 128);
        for (time, cpu, expected) in [(101, 25_000, 95), (102, 20_000, 82), (103, 60_000, 192)] {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(time);
            fake.last_sample = sample(fake.now, Some(cpu), Some(45_000));
            drop(fake);
            engine.tick(Duration::from_secs(time)).unwrap();
            assert_eq!(state.lock().unwrap().pwm[&3], expected);
        }
        engine.stop().unwrap();
    }

    #[test]
    fn crash_states_restore_exact_original_pwm_for_engine_and_standalone() {
        for standalone in [false, true] {
            for (mode, pwm, safe) in [
                (2, 63, true),
                (1, 63, true),
                (1, 128, true),
                (1, 255, true),
                (2, 255, false),
                (7, 128, false),
                (0, 128, false),
            ] {
                let state = new_fake_state(Some(recovery_record()));
                let mut engine = (!standalone).then(|| {
                    let mut engine =
                        harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())));
                    let _ = engine.initialize();
                    engine
                });
                {
                    let mut fake = state.lock().unwrap();
                    fake.enable.insert(4, mode);
                    fake.pwm.insert(4, pwm);
                    fake.log.clear();
                }
                let result = if standalone {
                    restore_from_record(
                        &mut FakeSysfs(state.clone()),
                        &mut FakeRecovery(state.clone()),
                    )
                } else {
                    engine.as_mut().unwrap().finalize()
                };
                assert_eq!(
                    result.is_ok(),
                    safe,
                    "standalone={standalone} mode={mode} pwm={pwm}: {result:?}"
                );
                let fake = state.lock().unwrap();
                assert_eq!((fake.enable[&3], fake.pwm[&3]), (2, 63));
                assert_eq!(fake.record.is_none(), safe);
                assert!(
                    !fake
                        .log
                        .iter()
                        .any(|op| op.starts_with("write_enable:") && op.ends_with(":0")
                            || op.ends_with(":255"))
                );
                if safe {
                    assert_eq!((fake.enable[&4], fake.pwm[&4]), (2, 63));
                }
                if !safe {
                    assert!(!fake.log.iter().any(|op| op.starts_with("write_")));
                }
            }
        }
    }

    #[test]
    fn original_pwm_write_failure_still_attempts_bios_mode_and_later_channels() {
        for standalone in [false, true] {
            for fan in [3, 4] {
                let state = new_fake_state(Some(recovery_record()));
                {
                    let mut fake = state.lock().unwrap();
                    fake.enable = [(3, 1), (4, 1)].into();
                    fake.pwm = [(3, 255), (4, 255)].into();
                    fake.log.clear();
                }
                // Measure the original write boundary from the same initial state.
                let measured = new_fake_state(Some(recovery_record()));
                {
                    let mut fake = measured.lock().unwrap();
                    fake.enable = [(3, 1), (4, 1)].into();
                    fake.pwm = [(3, 255), (4, 255)].into();
                    fake.log.clear();
                }
                restore_from_record(
                    &mut FakeSysfs(measured.clone()),
                    &mut FakeRecovery(measured.clone()),
                )
                .unwrap();
                let position = measured
                    .lock()
                    .unwrap()
                    .log
                    .iter()
                    .position(|op| op == &format!("write_pwm:{fan}:63"))
                    .unwrap()
                    + 1;
                let mut engine = (!standalone).then(|| {
                    let mut engine =
                        harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())));
                    let _ = engine.initialize();
                    engine
                });
                let baseline = state.lock().unwrap().log.len();
                set_fail_absolute(&state, baseline + position);
                let result = if standalone {
                    restore_from_record(
                        &mut FakeSysfs(state.clone()),
                        &mut FakeRecovery(state.clone()),
                    )
                } else {
                    engine.as_mut().unwrap().finalize()
                };
                assert_eq!(
                    result.unwrap_err().kind(),
                    HardwareErrorKind::RestoreRequired
                );
                let fake = state.lock().unwrap();
                assert!(fake.log.contains(&format!("write_enable:{fan}:2")));
                assert!(fake.log.contains(&format!("write_enable:{}:2", 7 - fan)));
                assert!(fake.record.is_some());
                assert!(
                    !fake
                        .log
                        .iter()
                        .any(|op| op.starts_with("write_enable:") && op.ends_with(":0")
                            || op.ends_with(":255"))
                );
            }
        }
    }

    #[test]
    fn standalone_restore_no_record_is_noop_and_partial_failure_continues() {
        let (_, state) = harness();
        state.lock().unwrap().log.clear();
        restore_from_record(
            &mut FakeSysfs(state.clone()),
            &mut FakeRecovery(state.clone()),
        )
        .unwrap();
        assert_eq!(state.lock().unwrap().log, vec!["record_load"]);

        let record = recovery_record();
        state.lock().unwrap().record = Some(record);
        state.lock().unwrap().log.clear();
        set_fail_next(&state, 4); // a mode inspection fails after discovery
        let error = restore_from_record(
            &mut FakeSysfs(state.clone()),
            &mut FakeRecovery(state.clone()),
        )
        .unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
        let state = state.lock().unwrap();
        assert_finalizer_attempted(&state.log);
        assert!(state.record.is_some());
        assert!(!state.log.contains(&"record_remove".into()));
    }

    #[test]
    fn service_lifetime_lock_blocks_restore_before_recovery_or_sysfs_access() {
        let temp = TempDir::new().unwrap();
        let lock_path = temp.path().join("host-control.lock");
        let service_guard = HostControlLock::acquire_service_at(&lock_path).unwrap();
        let state = new_fake_state(Some(recovery_record()));
        state.lock().unwrap().log.clear();

        let blocked = (|| {
            let _restore_guard = HostControlLock::acquire_restore_at(&lock_path)?;
            restore_from_record(
                &mut FakeSysfs(state.clone()),
                &mut FakeRecovery(state.clone()),
            )
        })()
        .unwrap_err();
        assert_eq!(blocked.kind(), HardwareErrorKind::RestoreRequired);
        assert!(blocked.message().contains("service"));
        assert!(state.lock().unwrap().log.is_empty());
        assert!(state.lock().unwrap().record.is_some());

        drop(service_guard);
        let _restore_guard = HostControlLock::acquire_restore_at(&lock_path).unwrap();
        restore_from_record(
            &mut FakeSysfs(state.clone()),
            &mut FakeRecovery(state.clone()),
        )
        .unwrap();
        let state = state.lock().unwrap();
        assert!(state.record.is_none());
        assert_finalizer_attempted(&state.log);
    }

    #[test]
    fn standalone_restore_wraps_load_and_validation_failures_as_restore_required() {
        let state = new_fake_state(None);
        set_fail_absolute(&state, 1);
        let load_error = restore_from_record(
            &mut FakeSysfs(state.clone()),
            &mut FakeRecovery(state.clone()),
        )
        .unwrap_err();
        assert_eq!(load_error.kind(), HardwareErrorKind::RestoreRequired);
        assert!(load_error.message().contains("injected failure"));
        assert!(load_error.message().len() <= 512);
        assert!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .all(|operation| !operation.starts_with("write_"))
        );

        let mut invalid_record = recovery_record();
        invalid_record.version += 1;
        let state = new_fake_state(Some(invalid_record));
        let validation_error = restore_from_record(
            &mut FakeSysfs(state.clone()),
            &mut FakeRecovery(state.clone()),
        )
        .unwrap_err();
        assert_eq!(validation_error.kind(), HardwareErrorKind::RestoreRequired);
        assert!(
            validation_error
                .message()
                .contains("compiled hardware identity")
        );
        assert!(validation_error.message().len() <= 512);
        assert!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .all(|operation| !operation.starts_with("write_"))
        );
    }

    #[test]
    fn strict_recovery_parse_rejects_malformed_oversized_and_wrong_identity_without_sysfs() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("host-control.json");
        let valid = serde_json::to_value(recovery_record()).unwrap();
        let mut cases = vec![b"not json".to_vec(), vec![b'x'; MAX_RECOVERY_BYTES + 1]];
        // Mutate a valid synthetic v3 record so each constraint is exercised
        // independently, rather than masked by an unsupported version.
        for (pointer, value) in [
            ("/version", serde_json::json!(0)),
            ("/version", serde_json::json!(1)),
            ("/version", serde_json::json!(2)),
            ("/version", serde_json::json!(4)),
            ("/version", serde_json::json!(255)),
            ("/board_vendor", serde_json::json!("wrong")),
            ("/board_name", serde_json::json!("wrong")),
            ("/chip_name", serde_json::json!("wrong")),
            ("/chip_address", serde_json::json!(0)),
            ("/platform_component", serde_json::json!("wrong")),
            ("/channels/0/original_pwm", serde_json::json!(256)),
            ("/channels/0/original_pwm", serde_json::json!(90)),
            ("/channels/0/fan_channel", serde_json::json!(4)),
            ("/channels/1/pwm_channel", serde_json::json!(3)),
            ("/channels/0/controlled", serde_json::json!(null)),
            ("/channels", serde_json::json!([])),
            ("/channels", serde_json::json!([valid["channels"][0]])),
        ] {
            let mut changed = valid.clone();
            *changed.pointer_mut(pointer).unwrap() = value;
            cases.push(serde_json::to_vec(&changed).unwrap());
        }
        for field in ["controlled", "original_pwm"] {
            let mut changed = valid.clone();
            changed["channels"][0]
                .as_object_mut()
                .unwrap()
                .remove(field);
            cases.push(serde_json::to_vec(&changed).unwrap());
        }
        let mut no_owner = valid.clone();
        for channel in no_owner["channels"].as_array_mut().unwrap() {
            channel["controlled"] = serde_json::json!(false);
        }
        cases.push(serde_json::to_vec(&no_owner).unwrap());
        let mut wrong_order = valid.clone();
        wrong_order["channels"].as_array_mut().unwrap().reverse();
        cases.push(serde_json::to_vec(&wrong_order).unwrap());
        let mut unknown_field = valid.clone();
        unknown_field["path"] = serde_json::json!("/tmp/evil");
        cases.push(serde_json::to_vec(&unknown_field).unwrap());
        for bytes in cases {
            fs::write(&path, &bytes).unwrap();
            fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
            let (_, state) = harness();
            state.lock().unwrap().log.clear();
            let result = restore_from_record(
                &mut FakeSysfs(state.clone()),
                &mut FileRecoveryStore::at(path.clone()),
            );
            let error = result.unwrap_err();
            assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
            assert!(error.message().len() <= 512);
            assert!(state.lock().unwrap().log.is_empty());
            assert_eq!(fs::read(&path).unwrap(), bytes);
            assert!(FileRecoveryStore::at(path.clone()).remove().is_err());
            assert_eq!(fs::read(&path).unwrap(), bytes);
        }

        fs::write(&path, "malformed").unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        let (_, fake_state) = harness();
        fake_state.lock().unwrap().log.clear();
        let mut engine = HostControlEngine::with_dependencies(
            Some(config()),
            Some("GPU-exact".into()),
            Box::new(FakeSysfs(fake_state.clone())),
            Box::new(FileRecoveryStore::at(path)),
            Box::new(FakeSensors(fake_state.clone())),
            Box::new(FakeClock(fake_state.clone())),
            Box::new(MemoryPolicyStore::default()),
        );
        assert!(engine.initialize().is_err());
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert!(fake_state.lock().unwrap().log.is_empty());
    }

    #[test]
    fn production_recovery_store_is_atomic_mode_0600_and_cleans_stale_temp() {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("host-control.json");
        let stale = temp.path().join(RECOVERY_TEMP_FILE);
        fs::write(&stale, "stale").unwrap();
        let record = recovery_record();
        let mut store = FileRecoveryStore::at(path.clone());

        store.persist(&record).unwrap();
        assert!(!stale.exists());
        assert_eq!(
            fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o600
        );
        assert_eq!(store.load().unwrap(), Some(record));
        store.remove().unwrap();
        assert_eq!(store.load().unwrap(), None);
    }

    #[test]
    fn persist_explicitly_chmods_and_validates_the_open_descriptor_before_writing() {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("host-control.json");
        let operation_log = Arc::new(Mutex::new(Vec::new()));
        let invalid_metadata = RecoveryMetadata {
            is_regular: true,
            owner: 0,
            links: 1,
            mode: 0o100640,
        };
        let mut store = FileRecoveryStore::at_with_fs(
            path.clone(),
            Box::new(TestRecoveryFs::with_metadata(
                invalid_metadata,
                operation_log.clone(),
            )),
        );

        let error = store.persist(&recovery_record()).unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::InvalidData);
        assert_eq!(
            *operation_log.lock().unwrap(),
            vec![
                RecoveryFsOperation::PersistFileChmod,
                RecoveryFsOperation::PersistFileMetadata,
            ]
        );
        assert!(!path.exists());
        assert_eq!(
            fs::metadata(temp.path().join(RECOVERY_TEMP_FILE))
                .unwrap()
                .len(),
            0
        );
    }

    #[test]
    fn filesystem_persist_errors_happen_before_side_effect_and_require_restart_resolution() {
        for point in [
            RecoveryFsOperation::PersistTempWrite,
            RecoveryFsOperation::PersistFileChmod,
            RecoveryFsOperation::PersistFileMetadata,
            RecoveryFsOperation::PersistFileSync,
            RecoveryFsOperation::PersistRename,
            RecoveryFsOperation::PersistParentSync,
        ] {
            let temp = TempDir::new().unwrap();
            let path = temp.path().join("host-control.json");
            let temporary = temp.path().join(RECOVERY_TEMP_FILE);
            let store = FileRecoveryStore::at_with_fs(
                path.clone(),
                Box::new(TestRecoveryFs::failing(point)),
            );
            let state = new_fake_state(None);
            let mut engine = harness_with_recovery(state.clone(), Box::new(store));

            let error = engine
                .start(&policy(HostTemperatureSource::Cpu))
                .unwrap_err();
            assert_eq!(
                error.kind(),
                HardwareErrorKind::RestoreRequired,
                "{point:?}"
            );
            assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
            assert!(engine.finalize().is_err(), "{point:?}");
            assert!(
                state
                    .lock()
                    .unwrap()
                    .log
                    .iter()
                    .all(|operation| !operation.starts_with("write_")),
                "sysfs mutation after {point:?}"
            );

            let installed = point == RecoveryFsOperation::PersistParentSync;
            assert_eq!(path.exists(), installed, "record namespace after {point:?}");
            assert_eq!(
                temporary.exists(),
                !installed,
                "temp namespace after {point:?}"
            );

            let restarted_state = new_fake_state(None);
            let mut restarted = harness_with_recovery(
                restarted_state.clone(),
                Box::new(FileRecoveryStore::at(path.clone())),
            );
            assert_eq!(restarted.initialize().is_err(), installed);
            assert_eq!(
                restarted.snapshot().state,
                if installed {
                    HostControlState::RestoreRequired
                } else {
                    HostControlState::Available
                },
                "restart state after {point:?}"
            );
            restarted.finalize().unwrap();
            let wrote = restarted_state
                .lock()
                .unwrap()
                .log
                .iter()
                .any(|operation| operation.starts_with("write_"));
            assert!(
                !wrote,
                "an untouched mode-2/original restart was mutated after {point:?}"
            );
            assert!(!path.exists());
        }
    }

    #[test]
    fn every_persist_fault_is_restore_required_and_causes_zero_sysfs_writes() {
        for point in [
            RecoveryFaultPoint::PersistTempWrite,
            RecoveryFaultPoint::PersistFileSync,
            RecoveryFaultPoint::PersistRename,
            RecoveryFaultPoint::PersistParentSync,
        ] {
            let temp = TempDir::new().unwrap();
            let path = temp.path().join("host-control.json");
            let store =
                FileRecoveryStore::at_with_faults(path.clone(), Box::new(FailOnce::at(point)));
            let state = new_fake_state(None);
            let mut engine = harness_with_recovery(state.clone(), Box::new(store));

            let error = engine
                .start(&policy(HostTemperatureSource::Cpu))
                .unwrap_err();
            assert_eq!(
                error.kind(),
                HardwareErrorKind::RestoreRequired,
                "{point:?}"
            );
            assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
            assert!(
                state
                    .lock()
                    .unwrap()
                    .log
                    .iter()
                    .all(|operation| !operation.starts_with("write_")),
                "sysfs mutation after {point:?}"
            );
            assert!(engine.stop().is_err());
            assert!(
                state
                    .lock()
                    .unwrap()
                    .log
                    .iter()
                    .all(|operation| !operation.starts_with("write_")),
                "finalization mutation after {point:?}"
            );

            assert_eq!(
                path.exists(),
                matches!(
                    point,
                    RecoveryFaultPoint::PersistRename | RecoveryFaultPoint::PersistParentSync
                ),
                "unexpected restart-visible record state after {point:?}"
            );
        }
    }

    #[test]
    fn two_phase_removal_retries_every_fault_boundary_idempotently() {
        for point in [
            RecoveryFaultPoint::RemoveRecordRename,
            RecoveryFaultPoint::RemoveMarkerSync,
            RecoveryFaultPoint::RemoveMarkerUnlink,
            RecoveryFaultPoint::RemoveFinalSync,
        ] {
            let temp = TempDir::new().unwrap();
            let path = temp.path().join("host-control.json");
            let marker = temp.path().join(RECOVERY_MARKER_FILE);
            let record = recovery_record();
            FileRecoveryStore::at(path.clone())
                .persist(&record)
                .unwrap();
            let mut store =
                FileRecoveryStore::at_with_faults(path.clone(), Box::new(FailOnce::at(point)));

            assert!(store.remove().is_err(), "{point:?}");
            match point {
                RecoveryFaultPoint::RemoveRecordRename | RecoveryFaultPoint::RemoveMarkerSync => {
                    assert!(!path.exists());
                    assert!(marker.exists());
                }
                RecoveryFaultPoint::RemoveMarkerUnlink | RecoveryFaultPoint::RemoveFinalSync => {
                    assert!(!path.exists());
                    assert!(!marker.exists());
                }
                _ => unreachable!(),
            }

            drop(store);
            let mut restarted = FileRecoveryStore::at(path.clone());
            restarted.remove().unwrap();
            assert!(!path.exists());
            assert!(!marker.exists());
            assert_eq!(FileRecoveryStore::at(path).load().unwrap(), None);
        }
    }

    #[test]
    fn filesystem_removal_errors_preserve_namespace_and_restart_idempotently() {
        for point in [
            RecoveryFsOperation::RemoveRecordRename,
            RecoveryFsOperation::RemoveMarkerSync,
            RecoveryFsOperation::RemoveMarkerUnlink,
            RecoveryFsOperation::RemoveFinalSync,
        ] {
            let temp = TempDir::new().unwrap();
            let path = temp.path().join("host-control.json");
            let marker = temp.path().join(RECOVERY_MARKER_FILE);
            FileRecoveryStore::at(path.clone())
                .persist(&recovery_record())
                .unwrap();
            let mut store = FileRecoveryStore::at_with_fs(
                path.clone(),
                Box::new(TestRecoveryFs::failing(point)),
            );

            let error = store.remove().unwrap_err();
            assert_eq!(
                error.kind(),
                HardwareErrorKind::PermissionDenied,
                "{point:?}"
            );
            match point {
                RecoveryFsOperation::RemoveRecordRename => {
                    assert!(path.exists());
                    assert!(!marker.exists());
                }
                RecoveryFsOperation::RemoveMarkerSync | RecoveryFsOperation::RemoveMarkerUnlink => {
                    assert!(!path.exists());
                    assert!(marker.exists());
                }
                RecoveryFsOperation::RemoveFinalSync => {
                    assert!(!path.exists());
                    assert!(!marker.exists());
                }
                _ => unreachable!(),
            }

            drop(store);
            let mut restarted = FileRecoveryStore::at(path.clone());
            restarted.remove().unwrap();
            assert!(!path.exists());
            assert!(!marker.exists());
            assert_eq!(restarted.load().unwrap(), None);
        }
    }

    #[test]
    fn deletion_durability_faults_keep_engine_restore_required_until_retry() {
        for point in [
            RecoveryFaultPoint::RemoveRecordRename,
            RecoveryFaultPoint::RemoveMarkerSync,
            RecoveryFaultPoint::RemoveMarkerUnlink,
            RecoveryFaultPoint::RemoveFinalSync,
        ] {
            let temp = TempDir::new().unwrap();
            let path = temp.path().join("host-control.json");
            let store = FileRecoveryStore::at_with_faults(path, Box::new(FailOnce::at(point)));
            let state = new_fake_state(None);
            let mut engine = harness_with_recovery(state.clone(), Box::new(store));
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            let baseline = state.lock().unwrap().log.len();

            let error = engine.stop().unwrap_err();
            assert_eq!(
                error.kind(),
                HardwareErrorKind::RestoreRequired,
                "{point:?}"
            );
            assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
            assert_finalizer_attempted(&state.lock().unwrap().log[baseline..]);
            assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());

            engine.stop().unwrap();
            assert_eq!(engine.snapshot().state, HostControlState::Available);
        }
    }

    #[test]
    fn constructor_load_failure_is_retryable_and_restores_a_record_that_appears() {
        let state = new_fake_state(None);
        set_fail_absolute(&state, 1);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())));
        assert!(engine.initialize().is_err());
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);

        state.lock().unwrap().record = Some(recovery_record());
        engine.finalize().unwrap();

        assert_eq!(engine.snapshot().state, HostControlState::Available);
        let state = state.lock().unwrap();
        assert_eq!(
            state
                .log
                .iter()
                .filter(|entry| *entry == "record_load")
                .count(),
            2
        );
        assert_finalizer_attempted(&state.log);
        assert!(state.record.is_none());
    }

    #[test]
    fn constructor_load_failure_can_resolve_to_no_record_without_sysfs_writes() {
        let state = new_fake_state(None);
        set_fail_absolute(&state, 1);
        set_fail_absolute(&state, 2);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())));

        assert!(engine.initialize().is_err());
        let retry_error = engine.finalize().unwrap_err();
        assert_eq!(retry_error.kind(), HardwareErrorKind::RestoreRequired);
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        engine.stop().unwrap();

        assert_eq!(engine.snapshot().state, HostControlState::Available);
        let state = state.lock().unwrap();
        assert_eq!(state.log, vec!["record_load", "record_load", "record_load"]);
    }

    #[test]
    fn transient_constructor_marker_cleanup_failure_retries_without_sysfs_writes() {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("host-control.json");
        let marker = temp.path().join(RECOVERY_MARKER_FILE);
        FileRecoveryStore::at(path.clone())
            .persist(&recovery_record())
            .unwrap();
        fs::rename(&path, &marker).unwrap();
        let store = FileRecoveryStore::at_with_fs(
            path.clone(),
            Box::new(TestRecoveryFs::failing(
                RecoveryFsOperation::RemoveMarkerSync,
            )),
        );
        let state = new_fake_state(None);
        let mut engine = harness_with_recovery(state.clone(), Box::new(store));
        assert!(engine.initialize().is_err());
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert!(marker.exists());

        engine.stop().unwrap();

        assert_eq!(engine.snapshot().state, HostControlState::Available);
        assert!(!marker.exists());
        assert!(state.lock().unwrap().log.is_empty());
    }

    #[test]
    fn persist_uncertainty_never_reloads_or_finalizes_in_the_same_engine() {
        let (mut measured, measured_state) = harness();
        measured.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let persist_boundary = measured_state
            .lock()
            .unwrap()
            .log
            .iter()
            .position(|operation| operation == "record_persist")
            .unwrap()
            + 1;

        let state = new_fake_state(None);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())));
        set_fail_absolute(&state, persist_boundary);

        let error = engine
            .start(&policy(HostTemperatureSource::Cpu))
            .unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
        for result in [engine.finalize(), engine.stop(), engine.shutdown()] {
            assert_eq!(
                result.unwrap_err().kind(),
                HardwareErrorKind::RestoreRequired
            );
        }

        let state = state.lock().unwrap();
        assert_eq!(
            state
                .log
                .iter()
                .filter(|entry| *entry == "record_load")
                .count(),
            0
        );
        assert!(
            state
                .log
                .iter()
                .all(|operation| !operation.starts_with("write_"))
        );
    }

    #[test]
    fn marker_only_restart_cleans_without_hardware_writes_and_both_files_are_rejected() {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("host-control.json");
        let marker = temp.path().join(RECOVERY_MARKER_FILE);
        let record = recovery_record();
        FileRecoveryStore::at(path.clone())
            .persist(&record)
            .unwrap();
        fs::rename(&path, &marker).unwrap();

        let state = new_fake_state(None);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FileRecoveryStore::at(path.clone())));
        engine.initialize().unwrap();
        assert_eq!(engine.snapshot().state, HostControlState::Available);
        assert!(state.lock().unwrap().log.is_empty());
        assert!(!marker.exists());

        let mut store = FileRecoveryStore::at(path.clone());
        store.persist(&record).unwrap();
        fs::copy(&path, &marker).unwrap();
        let error = store.load().unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::InvalidData);
        assert!(path.exists());
        assert!(marker.exists());

        let state = new_fake_state(None);
        state.lock().unwrap().log.clear();
        let error = restore_from_record(&mut FakeSysfs(state.clone()), &mut store).unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
        assert!(error.message().contains("both exist"));
        assert!(state.lock().unwrap().log.is_empty());
    }

    #[test]
    fn recovery_metadata_validator_and_nofollow_nonblocking_opens_are_strict() {
        let valid = RecoveryMetadata {
            is_regular: true,
            owner: 0,
            links: 1,
            mode: 0o100600,
        };
        validate_recovery_metadata(valid, true).unwrap();
        for invalid_metadata in [
            RecoveryMetadata {
                is_regular: false,
                ..valid
            },
            RecoveryMetadata { owner: 1, ..valid },
            RecoveryMetadata { links: 2, ..valid },
            RecoveryMetadata {
                mode: 0o100640,
                ..valid
            },
            RecoveryMetadata {
                mode: 0o104600,
                ..valid
            },
        ] {
            assert!(validate_recovery_metadata(invalid_metadata, true).is_err());
        }
        validate_recovery_metadata(RecoveryMetadata { owner: 1, ..valid }, false).unwrap();

        let symlink_temp = TempDir::new().unwrap();
        let target = symlink_temp.path().join("target");
        let symlink_path = symlink_temp.path().join("host-control.json");
        fs::write(&target, b"{}").unwrap();
        fs::set_permissions(&target, fs::Permissions::from_mode(0o600)).unwrap();
        symlink(&target, &symlink_path).unwrap();
        assert!(FileRecoveryStore::at(symlink_path).load().is_err());

        let fifo_temp = TempDir::new().unwrap();
        let fifo_path = fifo_temp.path().join("host-control.json");
        let fifo_c = CString::new(fifo_path.as_os_str().as_bytes()).unwrap();
        assert_eq!(unsafe { libc::mkfifo(fifo_c.as_ptr(), 0o600) }, 0);
        assert!(FileRecoveryStore::at(fifo_path).load().is_err());

        let directory_temp = TempDir::new().unwrap();
        let directory_path = directory_temp.path().join("host-control.json");
        fs::create_dir(&directory_path).unwrap();
        fs::set_permissions(&directory_path, fs::Permissions::from_mode(0o600)).unwrap();
        assert!(FileRecoveryStore::at(directory_path).load().is_err());

        let mode_temp = TempDir::new().unwrap();
        let mode_path = mode_temp.path().join("host-control.json");
        fs::write(&mode_path, b"{}").unwrap();
        fs::set_permissions(&mode_path, fs::Permissions::from_mode(0o640)).unwrap();
        let state = new_fake_state(None);
        state.lock().unwrap().log.clear();
        let error = restore_from_record(
            &mut FakeSysfs(state.clone()),
            &mut FileRecoveryStore::at(mode_path),
        )
        .unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
        assert!(error.message().contains("mode 0600"));
        assert!(state.lock().unwrap().log.is_empty());
    }

    #[test]
    fn it8689_adapter_rejects_every_non_allowlisted_channel_before_attribute_access() {
        for channel in 0..=u8::MAX {
            let mut adapter = It8689Sysfs::new();
            if matches!(channel, 3 | 4) {
                assert_eq!(
                    adapter.read_pwm(channel).unwrap_err().kind(),
                    HardwareErrorKind::Unavailable
                );
                continue;
            }
            for error in [
                adapter.read_pwm(channel).unwrap_err(),
                adapter.read_fan(channel).unwrap_err(),
                adapter.read_enable(channel).unwrap_err(),
                adapter.write_pwm(channel, 1).unwrap_err(),
                adapter.write_enable(channel, 1).unwrap_err(),
            ] {
                assert_eq!(
                    error.kind(),
                    HardwareErrorKind::InvalidData,
                    "channel {channel}"
                );
            }
        }
    }

    #[test]
    fn constructor_with_stale_record_rediscovers_and_finalizes_all_channels() {
        let record = recovery_record();
        let (mut engine, state) = harness_with_record(Some(record));
        state.lock().unwrap().log.clear();

        engine.finalize().unwrap();

        let state = state.lock().unwrap();
        assert_eq!(state.log.first().map(String::as_str), Some("discover"));
        assert_finalizer_attempted(&state.log);
        assert_eq!(state.log.last().map(String::as_str), Some("record_remove"));
        assert_eq!(state.enable, [(3, 2), (4, 2)].into());
        assert!(state.record.is_none());
    }

    #[test]
    fn failed_restoration_overrides_initiating_error_with_bounded_diagnostics() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let baseline = state.lock().unwrap().log.len();
        // Tick sampling succeeds, its first write fails, then rediscovery fails.
        set_fail_absolute(&state, baseline + 2);
        set_fail_absolute(&state, baseline + 3);

        let error = engine.tick(Duration::from_secs(101)).unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::RestoreRequired);
        assert!(error.message().contains("initiating failure"));
        assert!(error.message().contains("restoration failure"));
        assert!(error.message().len() <= 512);
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert!(state.lock().unwrap().record.is_some());
        assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());
    }

    #[test]
    fn existing_record_blocks_start_and_standalone_restore_verifies_final_modes() {
        let record = recovery_record();
        let (mut engine, state) = harness_with_record(Some(record));
        let before = state.lock().unwrap().log.len();
        assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());
        assert_eq!(state.lock().unwrap().log.len(), before);

        restore_from_record(
            &mut FakeSysfs(state.clone()),
            &mut FakeRecovery(state.clone()),
        )
        .unwrap();
        let state = state.lock().unwrap();
        assert_eq!(state.enable, [(3, 2), (4, 2)].into());
        assert!(state.record.is_none());
        assert!(state.log.contains(&"read_enable:4".into()));
    }

    #[test]
    fn fan_rpm_cutoff_uses_u128_and_rounds_up() {
        assert!(!fan_rpm_meets_baseline(699, 980));
        assert!(!fan_rpm_meets_baseline(700, 699));
        assert!(fan_rpm_meets_baseline(700, 700));
        assert!(!fan_rpm_meets_baseline(980, 734));
        assert!(fan_rpm_meets_baseline(980, 735));
        let large_cutoff = ((u128::from(u64::MAX) * 3).div_ceil(4)) as u64;
        assert!(!fan_rpm_meets_baseline(u64::MAX, large_cutoff - 1));
        assert!(fan_rpm_meets_baseline(u64::MAX, large_cutoff));
    }

    #[test]
    fn regular_start_keeps_prior_fan_read_behavior() {
        for fan in [3, 4] {
            for rpm in [0, 699] {
                let (mut engine, state) = harness();
                state.lock().unwrap().fan.insert(fan, rpm);
                assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());
                let fake = state.lock().unwrap();
                assert!(!fake.log.iter().any(|op| op.starts_with("write_")));
                assert!(fake.record.is_none());
            }
        }
    }

    #[test]
    fn single_channel_both_orders_and_untouched_fan_are_guarded() {
        for (selected, reversed) in [(3, false), (4, false), (3, true), (4, true)] {
            let (mut engine, state) = harness();
            let mut config = config();
            if reversed {
                config.channels.reverse();
            }
            config
                .channels
                .retain(|entry| entry.pwm_channel == selected);
            let mut selected_policy = policy(HostTemperatureSource::Gpu);
            selected_policy
                .channels
                .retain(|entry| entry.channel_id == config.channels[0].id);
            engine.config = Some(config);
            engine.start(&selected_policy).unwrap();
            {
                let fake = state.lock().unwrap();
                let other = 7 - selected;
                assert_eq!((fake.enable[&selected], fake.pwm[&selected]), (1, 128));
                assert_eq!((fake.enable[&other], fake.pwm[&other]), (2, 63));
                let channels = &fake.record.as_ref().unwrap().channels;
                assert_eq!(
                    channels
                        .iter()
                        .map(|entry| (entry.pwm_channel, entry.controlled))
                        .collect::<Vec<_>>(),
                    vec![(3, Some(selected == 3)), (4, Some(selected == 4))]
                );
            }
            let other = 7 - selected;
            state.lock().unwrap().pwm.insert(other, 64);
            state.lock().unwrap().now = Duration::from_secs(101);
            state.lock().unwrap().last_sample =
                sample(Duration::from_secs(101), Some(45_000), Some(45_000));
            assert!(engine.tick(Duration::from_secs(101)).is_err());
            assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
            assert!(state.lock().unwrap().record.is_some());
            assert!(
                !state
                    .lock()
                    .unwrap()
                    .log
                    .iter()
                    .any(|op| op == &format!("write_pwm:{other}:63"))
            );
        }
        for reverse in [false, true] {
            let (mut engine, state) = harness();
            if reverse {
                engine.config.as_mut().unwrap().channels.reverse();
            }
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            let writes: Vec<_> = state
                .lock()
                .unwrap()
                .log
                .iter()
                .filter(|op| op.starts_with("write_"))
                .cloned()
                .collect();
            assert_eq!(
                writes[0],
                format!("write_enable:{}:1", if reverse { 4 } else { 3 })
            );
            engine.stop().unwrap();
        }
    }

    #[test]
    fn independent_cpu_sensor_rpm_and_mode_guards_restore_gpu_curve() {
        for reason in [
            "cpu_absent",
            "cpu_stale",
            "cpu_implausible",
            "rpm3",
            "rpm4",
            "mode3",
            "pwm4",
            "gap",
        ] {
            let (mut engine, state) = harness();
            engine.start(&policy(HostTemperatureSource::Gpu)).unwrap();
            {
                let mut fake = state.lock().unwrap();
                fake.now = Duration::from_secs(if reason == "gap" { 103 } else { 101 });
                fake.last_sample = match reason {
                    "cpu_absent" => sample(fake.now, None, Some(45_000)),
                    "cpu_stale" => sample(Duration::from_secs(97), Some(45_000), Some(45_000)),
                    "cpu_implausible" => sample(fake.now, Some(151_000), Some(45_000)),
                    _ => sample(fake.now, Some(45_000), Some(45_000)),
                };
                match reason {
                    "rpm3" => {
                        fake.fan.insert(3, 0);
                    }
                    "rpm4" => {
                        fake.fan.insert(4, 699);
                    }
                    "mode3" => {
                        fake.enable.insert(3, 7);
                    }
                    "pwm4" => {
                        fake.pwm.insert(4, 129);
                    }
                    _ => {}
                }
            }
            let now = state.lock().unwrap().now;
            assert!(engine.tick(now).is_err(), "{reason}");
            let fake = state.lock().unwrap();
            assert!(
                !fake
                    .log
                    .iter()
                    .any(|op| op == "write_pwm:3:144" || op == "write_pwm:4:144"),
                "{reason}"
            );
            assert_eq!(
                fake.record.is_some(),
                matches!(reason, "rpm3" | "rpm4" | "mode3")
            );
        }
    }

    #[test]
    fn unsupported_recovery_refused_by_engine_and_standalone_without_sysfs() {
        let mut record = recovery_record();
        record.version = 2;
        for channel in &mut record.channels {
            channel.controlled = None;
        }
        let (mut engine, state) = harness_with_record(Some(record));
        state.lock().unwrap().log.clear();
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert_eq!(
            engine.finalize().unwrap_err().kind(),
            HardwareErrorKind::RestoreRequired
        );
        assert_eq!(
            restore_from_record(
                &mut FakeSysfs(state.clone()),
                &mut FakeRecovery(state.clone())
            )
            .unwrap_err()
            .kind(),
            HardwareErrorKind::RestoreRequired
        );
        let fake = state.lock().unwrap();
        assert!(fake.record.is_some());
        assert!(
            !fake
                .log
                .iter()
                .any(|op| op == "discover" || op.starts_with("write_") || op == "record_remove")
        );
    }

    #[test]
    fn running_reaches_curve_255_directly_and_restores_without_forced_255() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Gpu)).unwrap();
        {
            let fake = state.lock().unwrap();
            assert_eq!(fake.pwm, [(3, 128), (4, 128)].into());
            assert!(
                !fake
                    .log
                    .iter()
                    .any(|op| op.ends_with(":255") || op == "write_enable:4:0")
            );
        }
        {
            let now = Duration::from_secs(101);
            {
                let mut fake = state.lock().unwrap();
                fake.now = now;
                fake.last_sample = sample(now, Some(60_000), Some(100_000));
            }
            engine.tick(now).unwrap();
            assert_eq!(state.lock().unwrap().pwm[&3], 255);
            assert_eq!(state.lock().unwrap().pwm[&4], 255);
        }
        assert_eq!(FakeSysfs(state.clone()).read_enable(4).unwrap(), 0);
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(102);
            fake.last_sample = sample(fake.now, Some(60_000), Some(100_000));
        }
        let writes = state
            .lock()
            .unwrap()
            .log
            .iter()
            .filter(|op| op.starts_with("write_pwm:"))
            .count();
        engine.tick(Duration::from_secs(102)).unwrap(); // accepts verified fan4 alias without redundant writes
        assert_eq!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .filter(|op| op.starts_with("write_pwm:"))
                .count(),
            writes
        );
        let before = state.lock().unwrap().log.len();
        state.lock().unwrap().fail_restore_writes = true;
        assert_eq!(
            engine.stop().unwrap_err().kind(),
            HardwareErrorKind::RestoreRequired
        );
        let fake = state.lock().unwrap();
        let restore_ops = &fake.log[before..];
        assert!(restore_ops.iter().any(|op| op == "write_pwm:3:128"));
        assert!(restore_ops.iter().any(|op| op == "write_pwm:4:128"));
        assert!(!restore_ops.iter().any(|op| op.ends_with(":255")
            || op == "write_enable:3:0"
            || op == "write_enable:4:0"));
        assert!(fake.record.is_some());
        assert_eq!((fake.pwm[&3], fake.pwm[&4]), (128, 128));
    }

    #[test]
    fn normal_ticks_apply_independent_up_and_down_targets_without_redundant_writes() {
        let (mut engine, state) = harness();
        let mut selected = policy(HostTemperatureSource::Cpu);
        selected.channels[1].curve.source = HostTemperatureSource::Gpu;
        engine.start(&selected).unwrap();
        for (time, cpu, gpu, front, rear) in [
            (101, 50_000, 45_000, 160, 144),
            (102, 80_000, 45_000, 255, 144),
            (103, 20_000, 80_000, 82, 255),
        ] {
            let now = Duration::from_secs(time);
            {
                let mut fake = state.lock().unwrap();
                fake.now = now;
                fake.last_sample = sample(now, Some(cpu), Some(gpu));
            }
            engine.tick(now).unwrap();
            assert_eq!(state.lock().unwrap().pwm, [(3, front), (4, rear)].into());
        }
        let writes_before = state
            .lock()
            .unwrap()
            .log
            .iter()
            .filter(|op| op.starts_with("write_pwm:"))
            .count();
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(104);
            fake.last_sample = sample(fake.now, Some(20_000), Some(80_000));
        }
        engine.tick(Duration::from_secs(104)).unwrap();
        let fake = state.lock().unwrap();
        assert_eq!(
            fake.log
                .iter()
                .filter(|op| op.starts_with("write_pwm:"))
                .count(),
            writes_before
        );
        assert_eq!(fake.pwm, [(3, 82), (4, 255)].into());
    }

    #[test]
    fn normal_tick_failed_target_readback_restores_both_channels() {
        struct CorruptOnce {
            inner: FakeSysfs,
            corrupt: bool,
        }
        impl HostControlSysfs for CorruptOnce {
            fn discover_exact(&mut self) -> Result<(), HardwareError> {
                self.inner.discover_exact()
            }
            fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError> {
                let value = self.inner.read_pwm(channel)?;
                if channel == 3 && self.corrupt && value == 255 {
                    self.corrupt = false;
                    Ok(254)
                } else {
                    Ok(value)
                }
            }
            fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError> {
                self.inner.read_fan(channel)
            }
            fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError> {
                self.inner.read_enable(channel)
            }
            fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
                self.inner.write_pwm(channel, value)
            }
            fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
                self.inner.write_enable(channel, value)
            }
        }
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        engine.sysfs = Box::new(CorruptOnce {
            inner: FakeSysfs(state.clone()),
            corrupt: true,
        });
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.last_sample = sample(fake.now, Some(80_000), Some(45_000));
        }
        assert!(engine.tick(Duration::from_secs(101)).is_err());
        let fake = state.lock().unwrap();
        assert!(fake.log.contains(&"write_pwm:3:255".into()));
        assert!(!fake.log.contains(&"write_pwm:4:255".into()));
        assert_eq!(fake.pwm, [(3, 63), (4, 63)].into());
        assert!(fake.record.is_none());
        assert_eq!(engine.snapshot().state, HostControlState::Available);
    }

    #[test]
    fn advancing_monotonic_clock_does_not_make_its_own_sample_future() {
        struct AdvancingClock(Arc<Mutex<FakeState>>);
        impl MonotonicTimeSource for AdvancingClock {
            fn now(&mut self) -> Result<Duration, HardwareError> {
                let mut fake = self.0.lock().unwrap();
                fake.now += Duration::from_millis(10);
                Ok(fake.now)
            }
        }
        struct FreshSensors;
        impl HostSensorSource for FreshSensors {
            fn sample(&mut self, now: Duration) -> HostSensorSample {
                sample(now, Some(50_000), Some(45_000))
            }
        }
        let state = new_fake_state(None);
        let mut engine = HostControlEngine::with_dependencies(
            Some(config()),
            Some("GPU-exact".into()),
            Box::new(FakeSysfs(state.clone())),
            Box::new(FakeRecovery(state.clone())),
            Box::new(FreshSensors),
            Box::new(AdvancingClock(state.clone())),
            Box::new(MemoryPolicyStore::default()),
        );
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        state.lock().unwrap().now += Duration::from_secs(1);
        let now = engine.monotonic_now().unwrap();
        engine.tick(now).unwrap();
        assert_eq!(state.lock().unwrap().pwm[&3], 160);
        engine.stop().unwrap();
    }

    #[test]
    fn blocking_read_can_exceed_absolute_previous_observation_deadline_without_writes() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let gate = BlockingGate::new("read_fan:4");
        engine.sysfs = Box::new(BlockingSysfs {
            inner: FakeSysfs(state.clone()),
            gate: gate.clone(),
        });
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.last_sample = sample(fake.now, Some(50_000), Some(45_000));
        }
        let thread = thread::spawn(move || {
            let result = engine.tick(Duration::from_secs(101));
            (engine, result)
        });
        gate.wait_until_entered();
        state.lock().unwrap().now = Duration::from_millis(102_100);
        gate.release();
        let (engine, result) = thread.join().unwrap();
        assert!(result.is_err());
        let fake = state.lock().unwrap();
        assert_eq!((fake.pwm[&3], fake.pwm[&4]), (63, 63));
        assert!(fake.record.is_none());
        assert_eq!(engine.snapshot().state, HostControlState::Available);
        assert!(
            !fake
                .log
                .iter()
                .any(|op| op == "write_pwm:3:160" || op == "write_pwm:4:160")
        );
    }

    #[test]
    fn late_write_completion_keeps_observation_deadline_without_pwm_pacing() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let gate = BlockingGate::new("write_pwm:3:160");
        engine.sysfs = Box::new(BlockingSysfs {
            inner: FakeSysfs(state.clone()),
            gate: gate.clone(),
        });
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.last_sample = sample(fake.now, Some(50_000), Some(45_000));
        }
        let thread = thread::spawn(move || {
            let result = engine.tick(Duration::from_secs(101));
            (engine, result)
        });
        gate.wait_until_entered();
        state.lock().unwrap().now = Duration::from_millis(101_900);
        gate.release();
        let (mut engine, result) = thread.join().unwrap();
        result.unwrap();
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(102);
            fake.last_sample = sample(fake.now, Some(50_000), Some(45_000));
        }
        engine.tick(Duration::from_secs(102)).unwrap();
        assert_eq!(state.lock().unwrap().pwm[&3], 160);
        assert_eq!(state.lock().unwrap().pwm[&4], 160);
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_millis(102_900);
            fake.last_sample = sample(fake.now, Some(50_000), Some(45_000));
        }
        // A new target is written even 900ms after the previous tick; no PWM pacing.
        state.lock().unwrap().last_sample =
            sample(Duration::from_millis(102_900), Some(80_000), Some(45_000));
        engine.tick(Duration::from_millis(102_900)).unwrap();
        assert_eq!(state.lock().unwrap().pwm[&3], 255);
        assert_eq!(state.lock().unwrap().pwm[&4], 255);
        engine.stop().unwrap();
    }

    #[test]
    fn slow_target_write_exceeding_observation_deadline_disarms_and_restores() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let gate = BlockingGate::new("write_pwm:3:160");
        engine.sysfs = Box::new(BlockingSysfs {
            inner: FakeSysfs(state.clone()),
            gate: gate.clone(),
        });
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.last_sample = sample(fake.now, Some(50_000), Some(45_000));
        }
        let thread = thread::spawn(move || {
            let result = engine.tick(Duration::from_secs(101));
            (engine, result)
        });
        gate.wait_until_entered();
        state.lock().unwrap().now = Duration::from_millis(102_100);
        gate.release();
        let (engine, result) = thread.join().unwrap();
        assert!(result.is_err());
        let fake = state.lock().unwrap();
        assert!(fake.log.contains(&"write_pwm:3:160".into()));
        assert!(!fake.log.contains(&"write_pwm:4:160".into()));
        assert_eq!(fake.pwm, [(3, 63), (4, 63)].into());
        assert!(fake.record.is_none());
        assert_eq!(engine.snapshot().state, HostControlState::Available);
    }

    #[test]
    fn slow_post_persistence_transition_finishes_protective_pair_then_restores() {
        let (mut engine, state) = harness();
        let gate = BlockingGate::new("write_enable:3:1");
        engine.sysfs = Box::new(BlockingSysfs {
            inner: FakeSysfs(state.clone()),
            gate: gate.clone(),
        });
        let thread = thread::spawn(move || {
            let result = engine.start(&policy(HostTemperatureSource::Cpu));
            (engine, result)
        });
        gate.wait_until_entered();
        state.lock().unwrap().now = Duration::from_secs(103);
        gate.release();
        let (engine, result) = thread.join().unwrap();
        assert!(result.is_err());
        let fake = state.lock().unwrap();
        let entry = fake
            .log
            .iter()
            .position(|op| op == "write_enable:3:1")
            .unwrap();
        let protected = fake
            .log
            .iter()
            .position(|op| op == "write_pwm:3:128")
            .unwrap();
        assert!(entry < protected);
        assert!(fake.log.contains(&"write_enable:3:2".into()));
        assert!(!fake.log.contains(&"write_enable:4:1".into()));
        assert_eq!(engine.snapshot().state, HostControlState::Available);
        assert!(fake.record.is_none());
    }

    #[test]
    fn early_tick_still_checks_required_cpu_instead_of_skipping_guards() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Gpu)).unwrap();
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_millis(100_100);
            fake.last_sample = sample(fake.now, None, Some(45_000));
        }
        assert!(engine.tick(Duration::from_millis(100_100)).is_err());
        let fake = state.lock().unwrap();
        assert!(!fake.log.contains(&"write_pwm:3:144".into()));
        assert!(fake.record.is_none());
    }

    #[test]
    fn normal_version_three_disk_recovery_restores_only_owned_channels() {
        let dir = TempDir::new().unwrap();
        let path = dir.path().join("host-control.json");
        let mut selected = config();
        selected.channels.retain(|channel| channel.pwm_channel == 3);
        let record = RecoveryRecord::from_config(&selected, &[63, 63]).unwrap();
        let mut store = FileRecoveryStore::at(path.clone());
        store.persist(&record).unwrap();
        assert_eq!(store.load().unwrap(), Some(record));
        let state = new_fake_state(None);
        {
            let mut fake = state.lock().unwrap();
            fake.enable.insert(3, 1);
            fake.pwm.insert(3, 128);
        }
        restore_from_record(&mut FakeSysfs(state.clone()), &mut store).unwrap();
        assert!(!path.exists());
        assert!(!store.marker_path().unwrap().exists());
        assert!(store.load().unwrap().is_none());
        let fake = state.lock().unwrap();
        assert_eq!(fake.enable, [(3, 2), (4, 2)].into());
        assert_eq!(fake.pwm, [(3, 63), (4, 63)].into());
        assert_eq!(
            fake.log
                .iter()
                .filter(|op| op.starts_with("write_"))
                .map(String::as_str)
                .collect::<Vec<_>>(),
            ["write_pwm:3:63", "write_enable:3:2"]
        );
    }

    #[test]
    fn on_disk_version_three_marks_both_channels_and_unsupported_file_is_left_intact() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("host-control.json");
        let mut store = FileRecoveryStore::at(path.clone());
        let record = RecoveryRecord::from_config(&config(), &[63, 63]).unwrap();
        store.persist(&record).unwrap();
        let encoded: serde_json::Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(encoded["version"], 3);
        assert_eq!(encoded["channels"][0]["controlled"], true);
        assert_eq!(encoded["channels"][1]["controlled"], true);
        store.remove().unwrap();

        let mut unsupported = record.clone();
        unsupported.version = 2;
        for channel in &mut unsupported.channels {
            channel.controlled = None;
        }
        assert!(store.persist(&unsupported).is_err());
        assert!(!path.exists());
        fs::write(&path, serde_json::to_vec(&unsupported).unwrap()).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        let before = fs::read(&path).unwrap();
        let state = new_fake_state(None);
        assert_eq!(
            restore_from_record(&mut FakeSysfs(state.clone()), &mut store)
                .unwrap_err()
                .kind(),
            HardwareErrorKind::RestoreRequired
        );
        assert_eq!(before, fs::read(&path).unwrap());
        assert!(
            !state
                .lock()
                .unwrap()
                .log
                .iter()
                .any(|op| op == "discover" || op.starts_with("write_"))
        );
    }

    #[test]
    fn unsupported_marker_is_preserved_by_startup_stop_and_standalone_restore() {
        let dir = TempDir::new().unwrap();
        let path = dir.path().join("host-control.json");
        let marker = dir.path().join(RECOVERY_MARKER_FILE);
        let mut store = FileRecoveryStore::at(path.clone());
        let mut unsupported = recovery_record();
        unsupported.version = 2;
        for channel in &mut unsupported.channels {
            channel.controlled = None;
        }
        let bytes = serde_json::to_vec(&unsupported).unwrap();
        fs::write(&path, &bytes).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        // Direct removal must not relabel unsupported data as a cleared record.
        assert!(store.remove().is_err());
        assert_eq!(fs::read(&path).unwrap(), bytes);
        assert!(!marker.exists());
        fs::rename(&path, &marker).unwrap();

        let state = new_fake_state(None);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FileRecoveryStore::at(path.clone())));
        assert!(engine.initialize().is_err());
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert_eq!(
            engine.stop().unwrap_err().kind(),
            HardwareErrorKind::RestoreRequired
        );
        assert_eq!(fs::read(&marker).unwrap(), bytes);
        assert_eq!(
            restore_from_record(&mut FakeSysfs(state.clone()), &mut store)
                .unwrap_err()
                .kind(),
            HardwareErrorKind::RestoreRequired,
        );
        assert!(store.remove().is_err());
        assert_eq!(fs::read(&marker).unwrap(), bytes);
        assert!(!path.exists());
        assert!(
            state.lock().unwrap().log.is_empty(),
            "unsupported marker must not cause any sysfs access"
        );
    }

    #[test]
    fn fan4_manual_pwm_255_alias_restores_original_with_normal_engine_record() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        // The driver reports mode 0 at PWM255 despite a manual control bit.
        // A restart must still recover the persisted original PWM on fan4.
        {
            let mut fake = state.lock().unwrap();
            fake.enable.insert(4, 1);
            fake.pwm.insert(4, 255);
        }
        let mut sysfs = FakeSysfs(state.clone());
        assert_eq!(sysfs.read_enable(4).unwrap(), 0);
        assert!(control::manual_observed(4, 0, 255));
        assert!(!control::manual_observed(3, 0, 255));
        drop(engine);
        restore_from_record(&mut sysfs, &mut FakeRecovery(state.clone())).unwrap();
        let fake = state.lock().unwrap();
        assert_eq!((fake.pwm[&4], fake.enable[&4]), (63, 2));
        assert_eq!((fake.pwm[&3], fake.enable[&3]), (63, 2));
        assert!(fake.record.is_none());
        assert!(fake.log.iter().any(|op| op == "write_pwm:4:63"));
    }
    fn persistent_engine(state: Arc<Mutex<FakeState>>, path: PathBuf) -> HostControlEngine {
        HostControlEngine::with_dependencies(
            Some(config()),
            Some("GPU-exact".into()),
            Box::new(FakeSysfs(state.clone())),
            Box::new(FakeRecovery(state.clone())),
            Box::new(FakeSensors(state.clone())),
            Box::new(FakeClock(state)),
            Box::new(FilePolicyStore::at(path)),
        )
    }

    #[test]
    fn automatic_recovery_rejects_channel_ownership_outside_the_enabled_policy() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let mut prior_config = config();
        prior_config
            .channels
            .retain(|channel| channel.pwm_channel == 3);
        let record = RecoveryRecord::from_config(&prior_config, &[63, 63]).unwrap();
        let state = new_fake_state(Some(record.clone()));
        {
            let mut fake = state.lock().unwrap();
            fake.enable.insert(3, 1);
            fake.pwm.insert(3, 128);
        }
        let mut engine = persistent_engine(state.clone(), path.clone());
        engine
            .config
            .as_mut()
            .unwrap()
            .channels
            .retain(|channel| channel.pwm_channel == 4);
        let identity = engine.identity().unwrap();
        let mut saved = policy(HostTemperatureSource::Cpu);
        saved
            .channels
            .retain(|channel| channel.channel_id == identity.host_control.channels[0].id);
        FilePolicyStore::at(path)
            .persist(&SavedPolicy::Enabled {
                version: 1,
                complete_policy: saved,
                exact_config_identity: identity.clone(),
            })
            .unwrap();
        assert!(record.validate_for_resume(&identity.host_control).is_err());
        assert!(record.validate_for_resume(&prior_config).is_ok());
        assert!(
            engine
                .initialize()
                .unwrap_err()
                .message()
                .contains("ownership")
        );
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert!(engine.shutdown().is_err());
        let fake = state.lock().unwrap();
        assert_eq!(fake.record, Some(record));
        assert!(!fake.log.iter().any(|operation| operation == "discover"
            || operation.starts_with("read_")
            || operation.starts_with("write_")
            || operation == "record_remove"));
        assert_eq!(fake.pwm[&3], 128);
        assert_eq!(fake.enable[&3], 1);
    }

    #[test]
    fn saved_policy_resumes_after_shutdown_or_crash_and_stop_disarms() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let selected = policy(HostTemperatureSource::Cpu);
        let mut first = persistent_engine(state.clone(), path.clone());
        first.initialize().unwrap();
        first.start(&selected).unwrap();
        assert_eq!(first.snapshot().active_policy, Some(selected.clone()));
        first.shutdown().unwrap();
        assert_eq!(first.snapshot().active_policy, None);
        assert!(state.lock().unwrap().record.is_none());
        let mut resumed = persistent_engine(state.clone(), path.clone());
        resumed.initialize().unwrap();
        assert_eq!(resumed.snapshot().active_policy, Some(selected.clone()));
        assert_eq!(state.lock().unwrap().enable, [(3, 1), (4, 1)].into());
        resumed.shutdown().unwrap();
        assert_eq!(state.lock().unwrap().enable, [(3, 2), (4, 2)].into());
        assert_eq!(state.lock().unwrap().pwm, [(3, 63), (4, 63)].into());
        assert!(state.lock().unwrap().record.is_none());
        assert!(matches!(
            FilePolicyStore::at(path.clone()).load().unwrap(),
            Some(SavedPolicy::Enabled { .. })
        ));

        let mut crashed = persistent_engine(state.clone(), path.clone());
        crashed.initialize().unwrap();
        // Simulated crash: no graceful finalizer; v3 evidence survives.
        drop(crashed);
        assert!(state.lock().unwrap().record.is_some());
        let mut recovered = persistent_engine(state.clone(), path.clone());
        recovered.initialize().unwrap();
        assert_eq!(recovered.snapshot().active_policy, Some(selected.clone()));
        assert!(state.lock().unwrap().record.is_some());
        recovered.shutdown().unwrap();
        assert_eq!(state.lock().unwrap().enable, [(3, 2), (4, 2)].into());
        assert_eq!(state.lock().unwrap().pwm, [(3, 63), (4, 63)].into());
        assert!(state.lock().unwrap().record.is_none());
        assert!(matches!(
            FilePolicyStore::at(path.clone()).load().unwrap(),
            Some(SavedPolicy::Enabled { .. })
        ));
        let mut disarming = persistent_engine(state.clone(), path.clone());
        disarming.initialize().unwrap();
        disarming.stop().unwrap();
        let writes_before = state
            .lock()
            .unwrap()
            .log
            .iter()
            .filter(|op| op.starts_with("write_"))
            .count();
        let mut stopped = persistent_engine(state.clone(), path.clone());
        stopped.initialize().unwrap();
        assert_eq!(stopped.snapshot().active_policy, None);
        assert_eq!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .filter(|op| op.starts_with("write_"))
                .count(),
            writes_before
        );
        assert_eq!(
            FilePolicyStore::at(path).load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
    }

    #[test]
    fn future_auto_resume_can_be_disabled_without_disarming_host_intent() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let selected = policy(HostTemperatureSource::Cpu);
        let mut first = persistent_engine(state.clone(), path.clone());
        first.initialize().unwrap();
        first.start(&selected).unwrap();
        first.shutdown().unwrap();
        let writes_before = state
            .lock()
            .unwrap()
            .log
            .iter()
            .filter(|event| event.starts_with("write_"))
            .count();

        let mut paused = persistent_engine(state.clone(), path.clone());
        paused.initialize_with_resume(false).unwrap();
        assert_eq!(paused.snapshot().state, HostControlState::Available);
        assert_eq!(paused.snapshot().active_policy, None);
        assert_eq!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .filter(|event| event.starts_with("write_"))
                .count(),
            writes_before
        );
        assert!(matches!(
            FilePolicyStore::at(path.clone()).load().unwrap(),
            Some(SavedPolicy::Enabled { .. })
        ));
        paused.shutdown().unwrap();

        let mut resumed = persistent_engine(state.clone(), path);
        resumed.initialize_with_resume(true).unwrap();
        assert_eq!(resumed.snapshot().active_policy, Some(selected));
        resumed.shutdown().unwrap();
    }

    #[test]
    fn disabling_future_resume_still_restores_pending_crash_record() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let selected = policy(HostTemperatureSource::Cpu);
        let mut first = persistent_engine(state.clone(), path.clone());
        first.initialize().unwrap();
        first.start(&selected).unwrap();
        drop(first); // Simulated unclean exit leaves the recovery record.
        assert!(state.lock().unwrap().record.is_some());

        let mut paused = persistent_engine(state.clone(), path.clone());
        paused.initialize_with_resume(false).unwrap();
        assert_eq!(paused.snapshot().state, HostControlState::Available);
        assert!(state.lock().unwrap().record.is_none());
        assert_eq!(state.lock().unwrap().enable, [(3, 2), (4, 2)].into());
        assert_eq!(state.lock().unwrap().pwm, [(3, 63), (4, 63)].into());
        assert!(matches!(
            FilePolicyStore::at(path).load().unwrap(),
            Some(SavedPolicy::Enabled { .. })
        ));
        paused.shutdown().unwrap();
    }

    #[test]
    fn cpu_above_65_allows_start_tick_live_update_and_restart_resume() {
        for source in [
            HostTemperatureSource::Cpu,
            HostTemperatureSource::Gpu,
            HostTemperatureSource::CpuGpuMax,
        ] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("active-policy.json");
            let state = new_fake_state(None);
            state.lock().unwrap().last_sample =
                sample(Duration::from_secs(100), Some(80_000), Some(45_000));
            let mut engine = persistent_engine(state.clone(), path.clone());
            engine.initialize().unwrap();
            engine.start(&policy(source)).unwrap();
            assert_eq!(
                engine.snapshot().state,
                HostControlState::Running,
                "{source:?}"
            );
            let expected = if source == HostTemperatureSource::Gpu {
                45_000
            } else {
                80_000
            };
            assert_eq!(
                engine.active[0].duty_millipercent,
                evaluate_curve(&curve(source, 30), expected)
            );
            {
                let mut fake = state.lock().unwrap();
                fake.now = Duration::from_secs(101);
                fake.last_sample = sample(fake.now, Some(80_000), Some(45_000));
            }
            engine.tick(Duration::from_secs(101)).unwrap();
            assert_eq!(
                state.lock().unwrap().pwm,
                [
                    (3, duty_to_pwm(evaluate_curve(&curve(source, 30), expected))),
                    (4, duty_to_pwm(evaluate_curve(&curve(source, 40), expected)))
                ]
                .into()
            );
            let changed = changed_policy("front", source);
            engine.update(std::slice::from_ref(&changed)).unwrap();
            assert_eq!(
                engine.active[0].duty_millipercent,
                evaluate_curve(&changed.curve, expected)
            );
            assert_eq!(engine.snapshot().state, HostControlState::Running);
            engine.shutdown().unwrap();
            assert_eq!(state.lock().unwrap().pwm, [(3, 63), (4, 63)].into());
            let mut resumed = persistent_engine(state.clone(), path.clone());
            resumed.initialize().unwrap();
            assert_eq!(
                resumed.snapshot().state,
                HostControlState::Running,
                "{source:?}"
            );
            assert_eq!(
                resumed.snapshot().active_policy.unwrap().channels[0],
                changed
            );
            {
                let mut fake = state.lock().unwrap();
                fake.now = Duration::from_secs(102);
                fake.last_sample = sample(fake.now, Some(80_000), Some(45_000));
            }
            resumed.tick(Duration::from_secs(102)).unwrap();
            assert_eq!(
                state.lock().unwrap().pwm,
                [
                    (3, duty_to_pwm(evaluate_curve(&changed.curve, expected))),
                    (4, duty_to_pwm(evaluate_curve(&curve(source, 40), expected)))
                ]
                .into()
            );
            resumed.stop().unwrap();
        }
    }

    #[test]
    fn resumed_worker_panic_guard_disarms_and_restores() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let mut first = persistent_engine(state.clone(), path.clone());
        first.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        first.shutdown().unwrap();
        let engine = persistent_engine(state.clone(), path.clone());
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_millis(5));
        worker.initialize().unwrap();
        assert_eq!(worker.snapshot().state, HostControlState::Running);
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.panic_sample = true;
        }
        wait_for_worker_state(&worker, HostControlState::RestoreRequired);
        assert_eq!(
            FilePolicyStore::at(path).load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
        let fake = state.lock().unwrap();
        assert_eq!(fake.enable, [(3, 2), (4, 2)].into());
        assert_eq!(fake.pwm, [(3, 63), (4, 63)].into());
        assert!(fake.record.is_none());
        drop(fake);
        assert!(worker.shutdown().is_err());
    }

    #[test]
    fn service_worker_runs_without_client_activity_and_shutdown_preserves_intent() {
        let (engine, state) = harness();
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_millis(5));
        let selected = policy(HostTemperatureSource::Cpu);
        worker.initialize().unwrap();
        worker.start(&selected).unwrap();
        for second in 101..=125 {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(second);
            fake.last_sample = sample(fake.now, Some(50_000), Some(45_000));
            drop(fake);
            thread::sleep(Duration::from_millis(6));
        }
        assert_eq!(worker.snapshot().state, HostControlState::Running);
        assert_eq!(worker.snapshot().active_policy, Some(selected));
        let handle = worker.shutdown_handle();
        handle.request_shutdown();
        handle.request_shutdown();
        worker.shutdown().unwrap();
        assert_eq!(worker.snapshot().active_policy, None);
        assert!(state.lock().unwrap().record.is_none());
    }

    #[test]
    fn bootstrap_rejects_unsafe_intent_or_recovery_without_fan_writes() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let enabled = SavedPolicy::Enabled {
            version: 1,
            complete_policy: policy(HostTemperatureSource::Cpu),
            exact_config_identity: ConfigIdentity {
                host_control: config(),
                nvidia_uuid: Some("GPU-exact".into()),
                nvidia_pci_bus_id: None,
            },
        };
        for scenario in ["absent", "disabled", "mismatch", "unsupported", "corrupt"] {
            let state = new_fake_state(None);
            if scenario == "unsupported" {
                let mut record = recovery_record();
                record.version = 2;
                state.lock().unwrap().record = Some(record);
            }
            let mut store = FilePolicyStore::at(path.clone());
            match scenario {
                "absent" => {
                    let _ = fs::remove_file(&path);
                }
                "disabled" => store
                    .persist(&SavedPolicy::Disabled { version: 1 })
                    .unwrap(),
                "mismatch" => {
                    let mut wrong = enabled.clone();
                    if let SavedPolicy::Enabled {
                        exact_config_identity,
                        ..
                    } = &mut wrong
                    {
                        exact_config_identity.nvidia_pci_bus_id = Some("different".into());
                    }
                    store.persist(&wrong).unwrap();
                }
                "corrupt" => {
                    fs::write(&path, b"invalid").unwrap();
                    fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
                }
                _ => store.persist(&enabled).unwrap(),
            }
            let mut engine = persistent_engine(state.clone(), path.clone());
            let result = engine.initialize();
            if ["mismatch", "unsupported", "corrupt"].contains(&scenario) {
                assert!(result.is_err(), "{scenario}");
            } else {
                assert!(result.is_ok(), "{scenario}: {result:?}");
            }
            assert!(
                state
                    .lock()
                    .unwrap()
                    .log
                    .iter()
                    .all(|op| !op.starts_with("write_")),
                "{scenario}"
            );
        }
    }

    #[test]
    fn shutdown_and_drop_never_restore_unverified_pending_v3() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let enabled = SavedPolicy::Enabled {
            version: 1,
            complete_policy: policy(HostTemperatureSource::Cpu),
            exact_config_identity: ConfigIdentity {
                host_control: config(),
                nvidia_uuid: Some("GPU-exact".into()),
                nvidia_pci_bus_id: Some("mismatch".into()),
            },
        };
        for scenario in [
            "no_bootstrap",
            "absent",
            "disabled",
            "mismatch",
            "no_config",
        ] {
            let state = new_fake_state(Some(recovery_record()));
            let mut store = FilePolicyStore::at(path.clone());
            match scenario {
                "absent" | "no_bootstrap" => {
                    let _ = fs::remove_file(&path);
                }
                "disabled" => store
                    .persist(&SavedPolicy::Disabled { version: 1 })
                    .unwrap(),
                _ => store.persist(&enabled).unwrap(),
            }
            let mut engine = persistent_engine(state.clone(), path.clone());
            if scenario == "no_config" {
                engine.config = None;
                engine.state = HostControlState::Disabled;
            }
            let mut worker =
                HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3600));
            if scenario != "no_bootstrap" {
                assert!(worker.initialize().is_err(), "{scenario}");
                assert_eq!(worker.snapshot().state, HostControlState::RestoreRequired);
                assert!(worker.shutdown().is_err(), "{scenario}");
            }
            drop(worker);
            let fake = state.lock().unwrap();
            assert!(fake.record.is_some(), "{scenario}");
            assert!(
                !fake.log.iter().any(|op| op == "discover"
                    || op.starts_with("write_")
                    || op == "record_remove"),
                "{scenario}: {:?}",
                fake.log
            );
        }
    }

    #[test]
    fn explicit_stop_after_rejected_bootstrap_does_not_ack_pending_unrestored_record() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let wrong = SavedPolicy::Enabled {
            version: 1,
            complete_policy: policy(HostTemperatureSource::Cpu),
            exact_config_identity: ConfigIdentity {
                host_control: config(),
                nvidia_uuid: Some("wrong-gpu".into()),
                nvidia_pci_bus_id: None,
            },
        };
        FilePolicyStore::at(path.clone()).persist(&wrong).unwrap();
        let state = new_fake_state(Some(recovery_record()));
        state.lock().unwrap().enable.insert(3, 7);
        let mut engine = persistent_engine(state.clone(), path.clone());
        assert!(engine.initialize().is_err());
        assert_eq!(
            engine.stop().unwrap_err().kind(),
            HardwareErrorKind::RestoreRequired
        );
        assert!(state.lock().unwrap().record.is_some());
        assert_eq!(
            FilePolicyStore::at(path).load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
    }

    #[test]
    fn unsafe_autostart_disarms_without_retries() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let mut first = persistent_engine(state.clone(), path.clone());
        first.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        first.shutdown().unwrap();
        state.lock().unwrap().pwm.insert(4, 62); // BIOS preflight no longer matches
        let writes = state
            .lock()
            .unwrap()
            .log
            .iter()
            .filter(|op| op.starts_with("write_"))
            .count();
        let mut restart = persistent_engine(state.clone(), path.clone());
        assert!(restart.initialize().is_err());
        assert_eq!(restart.snapshot().state, HostControlState::RestoreRequired);
        assert_eq!(
            FilePolicyStore::at(path).load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
        assert_eq!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .filter(|op| op.starts_with("write_"))
                .count(),
            writes
        );
    }

    #[test]
    fn failed_startup_recovery_disarms_but_keeps_v3_evidence() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let mut first = persistent_engine(state.clone(), path.clone());
        first.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        drop(first); // crash leaves mode 1 and normal v3 evidence
        state.lock().unwrap().fail_restore_writes = true;
        let mut restart = persistent_engine(state.clone(), path.clone());
        assert!(restart.initialize().is_err());
        assert_eq!(restart.snapshot().state, HostControlState::RestoreRequired);
        assert_eq!(
            FilePolicyStore::at(path).load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
        assert!(state.lock().unwrap().record.is_some());
    }

    #[test]
    fn implausible_cpu_disarms_policy_and_restart_does_not_resume() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let mut engine = persistent_engine(state.clone(), path.clone());
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.last_sample = sample(fake.now, Some(151_000), Some(45_000));
        }
        assert!(engine.tick(Duration::from_secs(101)).is_err());
        assert_eq!(engine.snapshot().active_policy, None);
        assert_eq!(
            FilePolicyStore::at(path.clone()).load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
        let writes = state
            .lock()
            .unwrap()
            .log
            .iter()
            .filter(|op| op.starts_with("write_"))
            .count();
        let mut restarted = persistent_engine(state.clone(), path);
        restarted.initialize().unwrap();
        assert_eq!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .filter(|op| op.starts_with("write_"))
                .count(),
            writes
        );
    }

    #[test]
    fn disarm_failure_restores_but_retains_recovery_and_blocks_starts() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let mut engine = persistent_engine(state.clone(), path.clone());
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        engine.policy_store =
            Box::new(FilePolicyStore::at(path.clone()).failing_after("after_rename"));
        assert_eq!(
            engine.stop().unwrap_err().kind(),
            HardwareErrorKind::RestoreRequired
        );
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());
        let state = state.lock().unwrap();
        assert_eq!(state.enable[&3], 2);
        assert_eq!(state.enable[&4], 2);
        assert!(state.record.is_some());
    }

    #[test]
    fn shutdown_signal_quiesces_start_but_delayed_stop_still_disarms() {
        for (target, queue_stop_first) in [("read_pwm:3", true), ("write_enable:3:1", false)] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("active-policy.json");
            let state = new_fake_state(None);
            let gate = BlockingGate::new(target);
            let mut engine = persistent_engine(state.clone(), path.clone());
            engine.sysfs = Box::new(BlockingSysfs {
                inner: FakeSysfs(state.clone()),
                gate: gate.clone(),
            });
            let mut worker =
                HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3600));
            let (start_response, start_rx) = mpsc::sync_channel(1);
            worker
                .sender
                .send(HostControlCommand::Start {
                    policy: policy(HostTemperatureSource::Cpu),
                    response: start_response,
                })
                .unwrap();
            gate.wait_until_entered();
            let (stop_response, stop_rx) = mpsc::sync_channel(1);
            if queue_stop_first {
                worker
                    .sender
                    .send(HostControlCommand::Stop {
                        response: stop_response.clone(),
                    })
                    .unwrap();
            }
            worker.shutdown_handle().request_shutdown();
            if !queue_stop_first {
                // Accepted by the manager before shutdown, but only dispatched
                // to the worker after the service shutdown signal.
                worker
                    .sender
                    .send(HostControlCommand::Stop {
                        response: stop_response,
                    })
                    .unwrap();
            }
            gate.release();
            let start_error = start_rx
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap_err();
            assert!(
                start_error.message().contains("shutdown"),
                "{target}: {start_error}"
            );
            stop_rx
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap();
            assert_eq!(
                FilePolicyStore::at(path).load().unwrap(),
                Some(SavedPolicy::Disabled { version: 1 })
            );
            assert_eq!(worker.snapshot().state, HostControlState::Available);
            worker.shutdown().unwrap();
        }
    }

    #[test]
    fn hardware_fault_wins_shutdown_race_and_disarms_durably() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let gate = BlockingGate::new("read_pwm:3");
        let mut engine = persistent_engine(state.clone(), path.clone());
        engine.sysfs = Box::new(BlockingSysfs {
            inner: FakeSysfs(state.clone()),
            gate: gate.clone(),
        });
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3600));
        let (response, receiver) = mpsc::sync_channel(1);
        worker
            .sender
            .send(HostControlCommand::Start {
                policy: policy(HostTemperatureSource::Cpu),
                response,
            })
            .unwrap();
        gate.wait_until_entered();
        set_fail_next(&state, 1); // actual read fails even though shutdown is signaled
        worker.shutdown_handle().request_shutdown();
        gate.release();
        let error = receiver
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap_err();
        assert!(error.message().contains("injected failure"), "{error}");
        assert_eq!(
            FilePolicyStore::at(path).load().unwrap(),
            Some(SavedPolicy::Disabled { version: 1 })
        );
        worker.shutdown().unwrap();
    }

    #[test]
    fn uncertain_policy_commit_blocks_hardware_and_retries() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("active-policy.json");
        let state = new_fake_state(None);
        let mut engine = persistent_engine(state.clone(), path.clone()).with_policy_store(
            Box::new(FilePolicyStore::at(path).failing_after("after_rename")),
        );
        engine.initialize().unwrap();
        assert_eq!(
            engine
                .start(&policy(HostTemperatureSource::Cpu))
                .unwrap_err()
                .kind(),
            HardwareErrorKind::RestoreRequired
        );
        assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
        assert!(engine.start(&policy(HostTemperatureSource::Cpu)).is_err());
        assert!(
            state
                .lock()
                .unwrap()
                .log
                .iter()
                .all(|op| !op.starts_with("write_"))
        );
    }
    #[derive(Clone)]
    struct UpdateStore {
        saved: Arc<Mutex<Vec<SavedPolicy>>>,
        fail: Arc<AtomicBool>,
        gate: Option<Arc<BlockingGate>>,
        state: Arc<Mutex<FakeState>>,
        stall_clock: bool,
        persist_time: Option<Duration>,
        post_cpu_fault: bool,
        post_gpu_missing: bool,
    }
    impl PolicyStore for UpdateStore {
        fn load(&mut self) -> Result<Option<SavedPolicy>, HardwareError> {
            Ok(self.saved.lock().unwrap().last().cloned())
        }
        fn persist(&mut self, saved: &SavedPolicy) -> Result<(), HardwareError> {
            if let Some(gate) = &self.gate {
                gate.hit("policy_persist");
            }
            if !self.saved.lock().unwrap().is_empty() {
                if self.stall_clock {
                    self.state.lock().unwrap().now += Duration::from_secs(3);
                }
                if let (Some(time), SavedPolicy::Enabled { .. }) = (self.persist_time, saved) {
                    self.state.lock().unwrap().now = time;
                }
                if self.post_cpu_fault {
                    self.state
                        .lock()
                        .unwrap()
                        .last_sample
                        .cpu
                        .as_mut()
                        .unwrap()
                        .millidegrees = 151_000;
                }
                if self.post_gpu_missing {
                    self.state.lock().unwrap().last_sample.gpu = None;
                }
            }
            if self.fail.swap(false, Ordering::AcqRel) {
                // Simulate an ambiguous commit: write may reach durable storage.
                self.saved.lock().unwrap().push(saved.clone());
                return Err(unavailable("injected uncertain policy fsync"));
            }
            self.saved.lock().unwrap().push(saved.clone());
            Ok(())
        }
    }
    fn update_store(state: &Arc<Mutex<FakeState>>) -> (UpdateStore, Arc<Mutex<Vec<SavedPolicy>>>) {
        let saved = Arc::new(Mutex::new(Vec::new()));
        (
            UpdateStore {
                saved: saved.clone(),
                fail: Arc::new(AtomicBool::new(false)),
                gate: None,
                state: state.clone(),
                stall_clock: false,
                persist_time: None,
                post_cpu_fault: false,
                post_gpu_missing: false,
            },
            saved,
        )
    }
    struct AdvancingReadSysfs {
        inner: FakeSysfs,
        state: Arc<Mutex<FakeState>>,
        trigger_at: Duration,
        advance_to: Duration,
    }
    impl HostControlSysfs for AdvancingReadSysfs {
        fn discover_exact(&mut self) -> Result<(), HardwareError> {
            self.inner.discover_exact()
        }
        fn read_pwm(&mut self, channel: u8) -> Result<u8, HardwareError> {
            if channel == 3 {
                let mut state = self.state.lock().unwrap();
                if state.now >= self.trigger_at && state.now < self.advance_to {
                    state.now = self.advance_to;
                }
            }
            self.inner.read_pwm(channel)
        }
        fn read_fan(&mut self, channel: u8) -> Result<u64, HardwareError> {
            self.inner.read_fan(channel)
        }
        fn read_enable(&mut self, channel: u8) -> Result<u8, HardwareError> {
            self.inner.read_enable(channel)
        }
        fn write_pwm(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            self.inner.write_pwm(channel, value)
        }
        fn write_enable(&mut self, channel: u8, value: u8) -> Result<(), HardwareError> {
            self.inner.write_enable(channel, value)
        }
    }

    fn changed_policy(id: &str, source: HostTemperatureSource) -> HostChannelPolicy {
        let minimum = if id == "front" { 30 } else { 40 };
        let mut curve = curve(source, minimum);
        curve.points[1].duty_percent = 75;
        HostChannelPolicy {
            channel_id: ChannelId::new(id),
            curve,
        }
    }

    #[test]
    fn batch_update_commits_once_without_writes_and_next_tick_targets_both() {
        let state = new_fake_state(None);
        let (store, saved) = update_store(&state);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                .with_policy_store(Box::new(store));
        let original = policy(HostTemperatureSource::Cpu);
        engine.start(&original).unwrap();
        let record = state.lock().unwrap().record.clone();
        let baseline = engine.baselines;
        let log_start = state.lock().unwrap().log.len();
        let changed = vec![
            changed_policy("front", HostTemperatureSource::Gpu),
            changed_policy("rear", HostTemperatureSource::CpuGpuMax),
        ];
        engine.update(&changed).unwrap();
        assert_eq!(engine.snapshot().active_policy.unwrap().channels, changed);
        assert_eq!(engine.baselines, baseline);
        assert_eq!(engine.active[0].commanded_pwm, 128);
        assert_eq!(engine.active[1].commanded_pwm, 128);
        assert_eq!(
            engine.active[0].duty_millipercent,
            evaluate_curve(&changed[0].curve, 45_000)
        );
        assert_eq!(
            engine.active[1].duty_millipercent,
            evaluate_curve(&changed[1].curve, 45_000)
        );
        assert_eq!(saved.lock().unwrap().len(), 2); // Start + ONE full Enabled batch.
        assert!(
            matches!(saved.lock().unwrap().last(), Some(SavedPolicy::Enabled { complete_policy, version: 1, .. }) if complete_policy.channels == changed)
        );
        {
            let fake = state.lock().unwrap();
            assert_eq!(fake.record, record);
            assert_eq!(fake.enable, [(3, 1), (4, 1)].into());
            assert_eq!(fake.pwm, [(3, 128), (4, 128)].into());
            assert!(
                !fake.log[log_start..]
                    .iter()
                    .any(|op| op.starts_with("write_") || op.starts_with("record_"))
            );
        }
        state.lock().unwrap().now = Duration::from_secs(101);
        engine.tick(Duration::from_secs(101)).unwrap();
        let fake = state.lock().unwrap();
        assert_eq!(
            fake.pwm[&3],
            duty_to_pwm(evaluate_curve(&changed[0].curve, 45_000))
        );
        assert_eq!(
            fake.pwm[&4],
            duty_to_pwm(evaluate_curve(&changed[1].curve, 45_000))
        );
    }

    #[test]
    fn rejected_batch_never_accepts_first_entry_or_persists_partial_intent() {
        let state = new_fake_state(None);
        let (store, saved) = update_store(&state);
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                .with_policy_store(Box::new(store));
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let original = engine.snapshot().active_policy;
        let original_record = state.lock().unwrap().record.clone();
        let first = changed_policy("front", HostTemperatureSource::Cpu);
        let second = changed_policy("rear", HostTemperatureSource::Cpu);
        let mut invalid = second.clone();
        invalid.curve.points[0].duty_percent = 0;
        let unknown = changed_policy("unknown", HostTemperatureSource::Cpu);
        for batch in [
            vec![],
            vec![first.clone(), first.clone()],
            vec![first.clone(), unknown],
            vec![first.clone(), invalid],
        ] {
            let before = state.lock().unwrap().log.len();
            assert_eq!(
                engine.update(&batch).unwrap_err().kind(),
                HardwareErrorKind::InvalidData
            );
            assert_eq!(engine.snapshot().active_policy, original);
            assert_eq!(state.lock().unwrap().log.len(), before);
            assert_eq!(saved.lock().unwrap().len(), 1);
        }
        state.lock().unwrap().last_sample.gpu = None;
        let before = state.lock().unwrap().log.len();
        assert!(
            engine
                .update(&[first, changed_policy("rear", HostTemperatureSource::Gpu)])
                .is_err()
        );
        assert_eq!(engine.snapshot().state, HostControlState::Running);
        assert_eq!(engine.snapshot().active_policy, original);
        assert_eq!(saved.lock().unwrap().len(), 1);
        let fake = state.lock().unwrap();
        assert_eq!(fake.record, original_record);
        assert!(!fake.log[before..].iter().any(|op| op.starts_with("write_")));
    }

    #[test]
    fn batch_guard_persist_and_postpersist_failures_never_ack_partial_commit() {
        for fault in [
            "cpu",
            "rpm",
            "pwm",
            "deadline",
            "persist",
            "post_persist",
            "post_cpu",
            "post_gpu",
        ] {
            let state = new_fake_state(None);
            let (mut store, saved) = update_store(&state);
            store.stall_clock = fault == "post_persist";
            store.post_cpu_fault = fault == "post_cpu";
            store.post_gpu_missing = fault == "post_gpu";
            let fail = store.fail.clone();
            let mut engine =
                harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                    .with_policy_store(Box::new(store));
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            let record = state.lock().unwrap().record.clone();
            {
                let mut fake = state.lock().unwrap();
                match fault {
                    "cpu" => fake.last_sample.cpu.as_mut().unwrap().millidegrees = 151_000,
                    "rpm" => {
                        fake.fan.insert(4, 100);
                    }
                    "pwm" => {
                        fake.pwm.insert(3, 142);
                    }
                    "deadline" => fake.now += Duration::from_secs(3),
                    "persist" => fail.store(true, Ordering::Release),
                    _ => {}
                }
            }
            let batch = [
                changed_policy("front", HostTemperatureSource::Cpu),
                changed_policy(
                    "rear",
                    if fault == "post_gpu" {
                        HostTemperatureSource::Gpu
                    } else {
                        HostTemperatureSource::Cpu
                    },
                ),
            ];
            assert!(engine.update(&batch).is_err(), "{fault}");
            assert_ne!(
                engine.snapshot().state,
                HostControlState::Running,
                "{fault}"
            );
            assert_eq!(
                state.lock().unwrap().record,
                if engine.snapshot().state == HostControlState::RestoreRequired {
                    record
                } else {
                    None
                },
                "{fault}"
            );
            if matches!(fault, "cpu" | "rpm" | "pwm" | "deadline") {
                assert_eq!(saved.lock().unwrap().len(), 2, "{fault}: start + disarm");
            }
        }
    }

    #[test]
    fn live_update_preserves_manual_pwm_baselines_other_group_and_record() {
        for (first, second) in [("front", "rear"), ("rear", "front")] {
            let state = new_fake_state(None);
            let (store, saved) = update_store(&state);
            let mut engine =
                harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                    .with_policy_store(Box::new(store));
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            {
                let mut fake = state.lock().unwrap();
                fake.now = Duration::from_secs(101);
                fake.last_sample.cpu.as_mut().unwrap().millidegrees = 60_000;
            }
            engine.tick(Duration::from_secs(101)).unwrap(); // both channels reach their 60C target
            let before = state.lock().unwrap();
            let record = before.record.clone();
            let writes = before
                .log
                .iter()
                .filter(|op| {
                    op.starts_with("write_") || *op == "record_remove" || *op == "record_persist"
                })
                .count();
            let pwm = before.pwm.clone();
            drop(before);
            let baseline = engine.baselines;
            let other_index = engine
                .active
                .iter()
                .position(|c| c.config.id == ChannelId::new(second))
                .unwrap();
            let other_duty = engine.active[other_index].duty_millipercent;
            let changed = changed_policy(first, HostTemperatureSource::Cpu);
            engine.update(std::slice::from_ref(&changed)).unwrap();
            assert_eq!(engine.baselines, baseline);
            assert_eq!(engine.active[other_index].duty_millipercent, other_duty);
            let fake = state.lock().unwrap();
            assert_eq!(fake.record, record);
            assert_eq!(fake.pwm, pwm);
            assert_eq!(fake.enable, [(3, 1), (4, 1)].into());
            assert_eq!(
                fake.log
                    .iter()
                    .filter(|op| op.starts_with("write_")
                        || *op == "record_remove"
                        || *op == "record_persist")
                    .count(),
                writes
            );
            drop(fake);
            let saved_policy = match saved.lock().unwrap().last().unwrap() {
                SavedPolicy::Enabled {
                    complete_policy,
                    exact_config_identity,
                    ..
                } => {
                    assert_eq!(*exact_config_identity, engine.identity().unwrap());
                    complete_policy.clone()
                }
                _ => panic!("update did not persist enabled intent"),
            };
            assert_eq!(saved_policy.channels.len(), 2);
            assert!(saved_policy.channels.contains(&changed));
            assert!(saved_policy.channels.contains(
                &policy(HostTemperatureSource::Cpu).channels[if second == "front" { 0 } else { 1 }]
            ));
            {
                state.lock().unwrap().now = Duration::from_secs(102);
            }
            engine.tick(Duration::from_secs(102)).unwrap();
            let fake = state.lock().unwrap();
            let number = if first == "front" { 3 } else { 4 };
            assert_eq!(
                fake.pwm[&number],
                duty_to_pwm(evaluate_curve(&changed.curve, 60_000))
            );
            assert_eq!(
                fake.pwm[&if number == 3 { 4 } else { 3 }],
                pwm[&if number == 3 { 4 } else { 3 }]
            );
        }
    }

    #[test]
    fn live_update_resets_old_curve_memory_but_keeps_two_degree_cooling_hysteresis() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::from_secs(101);
            fake.last_sample = sample(fake.now, Some(80_000), Some(45_000));
        }
        engine.tick(Duration::from_secs(101)).unwrap();
        assert_eq!(state.lock().unwrap().pwm, [(3, 255), (4, 255)].into());
        {
            let mut fake = state.lock().unwrap();
            fake.last_sample = sample(fake.now, Some(40_000), Some(45_000));
        }
        let changed = changed_policy("front", HostTemperatureSource::Cpu);
        engine.update(std::slice::from_ref(&changed)).unwrap();
        assert_eq!(state.lock().unwrap().pwm, [(3, 255), (4, 255)].into());
        for (time, temp, front) in [(102, 40_000, 192), (103, 39_000, 192), (104, 37_000, 186)] {
            let now = Duration::from_secs(time);
            {
                let mut fake = state.lock().unwrap();
                fake.now = now;
                fake.last_sample = sample(now, Some(temp), Some(45_000));
            }
            engine.tick(now).unwrap();
            assert_eq!(state.lock().unwrap().pwm[&3], front);
        }
        assert_eq!(
            engine.active[0].duty_millipercent,
            evaluate_curve(&changed.curve, 39_000)
        );
    }

    #[test]
    fn live_update_observations_advance_deadline_and_next_tick_applies_target() {
        let state = new_fake_state(None);
        {
            let mut fake = state.lock().unwrap();
            fake.now = Duration::ZERO;
            fake.last_sample = sample(Duration::ZERO, Some(40_000), Some(45_000));
        }
        let (mut store, _) = update_store(&state);
        store.persist_time = Some(Duration::from_millis(1_200));
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                .with_policy_store(Box::new(store));
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        engine.sysfs = Box::new(AdvancingReadSysfs {
            inner: FakeSysfs(state.clone()),
            state: state.clone(),
            trigger_at: Duration::from_millis(1_200),
            advance_to: Duration::from_millis(2_200),
        });
        state.lock().unwrap().now = Duration::from_millis(900);
        let baselines = engine.baselines;
        let record = state.lock().unwrap().record.clone();
        engine
            .update(&[changed_policy("front", HostTemperatureSource::Cpu)])
            .unwrap();
        assert_eq!(engine.snapshot().state, HostControlState::Running);
        assert_eq!(engine.last_tick, Some(Duration::from_millis(2_200)));
        assert_eq!(engine.last_time, engine.last_tick);
        assert_eq!(engine.baselines, baselines);
        {
            let fake = state.lock().unwrap();
            assert_eq!(fake.enable, [(3, 1), (4, 1)].into());
            assert_eq!(fake.pwm, [(3, 128), (4, 128)].into());
            assert_eq!(fake.record, record);
            assert!(
                !fake
                    .log
                    .iter()
                    .any(|op| op == "record_remove" || op == "write_pwm:3:192")
            );
        }
        state.lock().unwrap().now = Duration::from_millis(2_900);
        engine.tick(Duration::from_millis(2_900)).unwrap();
        assert_eq!(state.lock().unwrap().pwm, [(3, 192), (4, 128)].into());
        let writes = state
            .lock()
            .unwrap()
            .log
            .iter()
            .filter(|op| op.starts_with("write_pwm:"))
            .count();
        state.lock().unwrap().now = Duration::from_millis(2_950);
        engine.tick(Duration::from_millis(2_950)).unwrap();
        let fake = state.lock().unwrap();
        assert_eq!(fake.pwm, [(3, 192), (4, 128)].into());
        assert_eq!(
            fake.log
                .iter()
                .filter(|op| op.starts_with("write_pwm:"))
                .count(),
            writes
        );
    }

    #[test]
    fn live_update_rejects_stalled_persist_and_slow_guard_observations() {
        for (name, persist_time, read_at, advance_to) in [
            ("persist", 3_010, None, 0),
            ("pre-read", 1_200, Some(900), 2_100),
            ("post-read", 1_200, Some(1_200), 3_300),
        ] {
            let state = new_fake_state(None);
            {
                let mut fake = state.lock().unwrap();
                fake.now = Duration::ZERO;
                fake.last_sample = sample(Duration::ZERO, Some(40_000), Some(45_000));
            }
            let (mut store, saved) = update_store(&state);
            store.persist_time = Some(Duration::from_millis(persist_time));
            let mut engine =
                harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                    .with_policy_store(Box::new(store));
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            if let Some(read_at) = read_at {
                engine.sysfs = Box::new(AdvancingReadSysfs {
                    inner: FakeSysfs(state.clone()),
                    state: state.clone(),
                    trigger_at: Duration::from_millis(read_at),
                    advance_to: Duration::from_millis(advance_to),
                });
            }
            state.lock().unwrap().now = Duration::from_millis(900);
            assert!(
                engine
                    .update(&[changed_policy("front", HostTemperatureSource::Cpu)])
                    .is_err(),
                "{name}"
            );
            assert_ne!(engine.snapshot().state, HostControlState::Running, "{name}");
            assert!(
                !state
                    .lock()
                    .unwrap()
                    .log
                    .iter()
                    .any(|op| op == "write_pwm:3:192"),
                "{name}"
            );
            let persisted = saved.lock().unwrap();
            if name == "pre-read" {
                assert_eq!(persisted.len(), 2);
            } else {
                assert_eq!(persisted.len(), 3);
            }
            assert!(matches!(
                persisted.last(),
                Some(SavedPolicy::Disabled { .. })
            ));
        }
    }

    #[test]
    fn live_update_single_channel_merges_and_resumes_saved_intent() {
        for (id, number) in [("front", 3), ("rear", 4)] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("active-policy.json");
            let state = new_fake_state(None);
            let mut engine = persistent_engine(state.clone(), path.clone());
            engine
                .config
                .as_mut()
                .unwrap()
                .channels
                .retain(|c| c.pwm_channel == number);
            let selected = HostControlPolicy {
                channels: policy(HostTemperatureSource::Cpu)
                    .channels
                    .into_iter()
                    .filter(|c| c.channel_id == ChannelId::new(id))
                    .collect(),
            };
            engine.start(&selected).unwrap();
            let changed = changed_policy(id, HostTemperatureSource::Cpu);
            let before = state.lock().unwrap().record.clone();
            engine.update(std::slice::from_ref(&changed)).unwrap();
            assert_eq!(state.lock().unwrap().record, before);
            assert_eq!(
                engine.snapshot().active_policy.unwrap().channels,
                vec![changed.clone()]
            );
            let other = if number == 3 { 4 } else { 3 };
            assert_eq!(state.lock().unwrap().enable[&other], 2);
            assert_eq!(state.lock().unwrap().pwm[&other], 63);
            engine.shutdown().unwrap();
            let mut resumed = persistent_engine(state, path);
            resumed
                .config
                .as_mut()
                .unwrap()
                .channels
                .retain(|c| c.pwm_channel == number);
            resumed.initialize().unwrap();
            assert_eq!(
                resumed.snapshot().active_policy.unwrap().channels,
                vec![changed]
            );
            resumed.shutdown().unwrap();
        }
    }

    #[test]
    fn live_source_change_uses_new_temperature_without_touching_other_channel() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let baseline = engine.baselines;
        let other = engine.active[1].clone();
        let new = changed_policy("front", HostTemperatureSource::Gpu);
        engine.update(std::slice::from_ref(&new)).unwrap();
        assert_eq!(engine.active[0].policy, new);
        assert_eq!(
            engine.active[0].duty_millipercent,
            evaluate_curve(&new.curve, 45_000)
        );
        assert_eq!(engine.active[0].commanded_pwm, 128);
        assert_eq!(engine.active[1].policy, other.policy);
        assert_eq!(engine.active[1].duty_millipercent, other.duty_millipercent);
        assert_eq!(engine.baselines, baseline);
        let fake = state.lock().unwrap();
        assert_eq!(fake.pwm, [(3, 128), (4, 128)].into());
        assert!(fake.record.is_some());
    }

    #[test]
    fn invalid_and_unavailable_new_source_leave_running_policy_untouched() {
        let (mut engine, state) = harness();
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        let baseline = engine.snapshot().active_policy;
        let record = state.lock().unwrap().record.clone();
        let before = state.lock().unwrap().log.len();
        assert!(
            engine
                .update(&[changed_policy("unknown", HostTemperatureSource::Cpu)])
                .is_err()
        );
        let mut malformed = changed_policy("front", HostTemperatureSource::Cpu);
        malformed.curve.points[0].duty_percent = 0;
        assert!(engine.update(std::slice::from_ref(&malformed)).is_err());
        assert_eq!(state.lock().unwrap().log.len(), before);
        state.lock().unwrap().last_sample.gpu = None;
        assert!(
            engine
                .update(&[changed_policy("front", HostTemperatureSource::Gpu)])
                .is_err()
        );
        assert_eq!(engine.snapshot().state, HostControlState::Running);
        assert_eq!(engine.snapshot().active_policy, baseline);
        let fake = state.lock().unwrap();
        assert_eq!(fake.record, record);
        assert_eq!(fake.enable, [(3, 1), (4, 1)].into());
        assert!(!fake.log[before..].iter().any(|op| op.starts_with("write_")));
    }

    #[test]
    fn live_update_guard_and_persistence_faults_never_ack() {
        for fault in [
            "cpu",
            "rpm",
            "pwm",
            "deadline",
            "persist",
            "post_persist",
            "post_cpu",
            "post_gpu",
        ] {
            let state = new_fake_state(None);
            let (mut store, saved) = update_store(&state);
            if fault == "post_persist" {
                store.stall_clock = true;
            }
            store.post_cpu_fault = fault == "post_cpu";
            store.post_gpu_missing = fault == "post_gpu";
            let fail = store.fail.clone();
            let mut engine =
                harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                    .with_policy_store(Box::new(store));
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            {
                let mut fake = state.lock().unwrap();
                match fault {
                    "cpu" => fake.last_sample.cpu.as_mut().unwrap().millidegrees = 151_000,
                    "rpm" => {
                        fake.fan.insert(4, 100);
                    }
                    "pwm" => {
                        fake.pwm.insert(3, 142);
                    }
                    "deadline" => fake.now += Duration::from_secs(3),
                    "persist" => fail.store(true, Ordering::Release),
                    _ => {}
                }
            }
            let source = if fault == "post_gpu" {
                HostTemperatureSource::Gpu
            } else {
                HostTemperatureSource::Cpu
            };
            assert!(
                engine.update(&[changed_policy("front", source)]).is_err(),
                "{fault}"
            );
            assert_ne!(
                engine.snapshot().state,
                HostControlState::Running,
                "{fault}"
            );
            if fault == "persist" {
                assert_eq!(engine.snapshot().state, HostControlState::RestoreRequired);
                assert!(state.lock().unwrap().record.is_some());
            }
            if !matches!(fault, "persist" | "post_persist" | "post_cpu" | "post_gpu") {
                assert_eq!(
                    saved.lock().unwrap().len(),
                    2,
                    "{fault}: start + disarm only"
                );
            }
        }
    }

    #[test]
    fn every_live_policy_install_fault_keeps_recovery_evidence() {
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
            let state = new_fake_state(None);
            let mut engine = persistent_engine(state.clone(), path.clone());
            engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
            let record = state.lock().unwrap().record.clone();
            engine.policy_store = Box::new(FilePolicyStore::at(path).failing_after(point));
            assert_eq!(
                engine
                    .update(&[changed_policy("rear", HostTemperatureSource::Cpu)])
                    .unwrap_err()
                    .kind(),
                HardwareErrorKind::RestoreRequired,
                "{point}"
            );
            assert_eq!(
                engine.snapshot().state,
                HostControlState::RestoreRequired,
                "{point}"
            );
            assert_eq!(state.lock().unwrap().record, record, "{point}");
            assert!(engine.policy_uncertain, "{point}");
            assert_eq!(
                state.lock().unwrap().enable,
                [(3, 2), (4, 2)].into(),
                "{point}"
            );
        }
    }

    #[test]
    fn worker_update_error_persists_after_guard_stop_and_rejected_update() {
        let (engine, state) = harness();
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3600));
        worker.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        state
            .lock()
            .unwrap()
            .last_sample
            .cpu
            .as_mut()
            .unwrap()
            .millidegrees = 151_000;
        let fault = worker
            .update(&[changed_policy("front", HostTemperatureSource::Cpu)])
            .unwrap_err();
        assert!(fault.message().contains("CPU sensor value is out of range"));
        assert!(
            worker
                .update(&[changed_policy("unknown", HostTemperatureSource::Cpu)])
                .is_err()
        );
        assert!(
            worker
                .snapshot()
                .last_error
                .unwrap()
                .contains("CPU sensor value is out of range")
        );
        worker.shutdown().unwrap();
    }

    #[test]
    fn shutdown_before_update_commit_never_overwrites_old_intent() {
        let state = new_fake_state(None);
        let (store, saved) = update_store(&state);
        let gate = BlockingGate::new("read_pwm:3");
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                .with_policy_store(Box::new(store));
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        engine.sysfs = Box::new(BlockingSysfs {
            inner: FakeSysfs(state.clone()),
            gate: gate.clone(),
        });
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3600));
        let (response, rx) = mpsc::sync_channel(1);
        worker
            .sender
            .send(HostControlCommand::Update {
                channel_policies: vec![changed_policy("rear", HostTemperatureSource::Cpu)],
                response,
            })
            .unwrap();
        gate.wait_until_entered();
        worker.shutdown_handle().request_shutdown();
        gate.release();
        assert!(rx.recv_timeout(Duration::from_secs(2)).unwrap().is_err());
        assert_eq!(saved.lock().unwrap().len(), 1);
        assert!(matches!(
            saved.lock().unwrap().last(),
            Some(SavedPolicy::Enabled { .. })
        ));
        worker.shutdown().unwrap();
        assert_eq!(state.lock().unwrap().enable, [(3, 2), (4, 2)].into());
    }

    #[test]
    fn update_hardware_failure_wins_shutdown_during_read() {
        let state = new_fake_state(None);
        let (store, saved) = update_store(&state);
        let gate = BlockingGate::new("read_pwm:3");
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                .with_policy_store(Box::new(store));
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        engine.sysfs = Box::new(BlockingSysfs {
            inner: FakeSysfs(state.clone()),
            gate: gate.clone(),
        });
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3600));
        let (response, rx) = mpsc::sync_channel(1);
        worker
            .sender
            .send(HostControlCommand::Update {
                channel_policies: vec![changed_policy("rear", HostTemperatureSource::Cpu)],
                response,
            })
            .unwrap();
        gate.wait_until_entered();
        set_fail_next(&state, 1);
        worker.shutdown_handle().request_shutdown();
        gate.release();
        let error = rx
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap_err();
        assert!(error.message().contains("injected failure"), "{error}");
        assert!(matches!(
            saved.lock().unwrap().last(),
            Some(SavedPolicy::Disabled { .. })
        ));
        worker.shutdown().unwrap();
    }

    #[test]
    fn shutdown_during_update_persist_keeps_enabled_intent_and_drains_stop() {
        let state = new_fake_state(None);
        let (mut store, saved) = update_store(&state);
        let gate = BlockingGate::new("policy_persist");
        let mut engine =
            harness_with_recovery(state.clone(), Box::new(FakeRecovery(state.clone())))
                .with_policy_store(Box::new(store.clone()));
        // Gate only the update, not Start.
        engine.start(&policy(HostTemperatureSource::Cpu)).unwrap();
        store.gate = Some(gate.clone());
        engine.policy_store = Box::new(store);
        let mut worker =
            HostControlWorker::with_test_tick_interval(engine, Duration::from_secs(3600));
        let (response, rx) = mpsc::sync_channel(1);
        worker
            .sender
            .send(HostControlCommand::Update {
                channel_policies: vec![changed_policy("rear", HostTemperatureSource::Cpu)],
                response,
            })
            .unwrap();
        gate.wait_until_entered();
        worker.shutdown_handle().request_shutdown();
        gate.release();
        assert!(rx.recv_timeout(Duration::from_secs(2)).unwrap().is_err());
        assert!(matches!(
            saved.lock().unwrap().last(),
            Some(SavedPolicy::Enabled { .. })
        ));
        assert_eq!(state.lock().unwrap().enable, [(3, 2), (4, 2)].into());
        // A Stop accepted before client loss can still disarm after quiescence.
        worker.stop().unwrap();
        assert!(matches!(
            saved.lock().unwrap().last(),
            Some(SavedPolicy::Disabled { .. })
        ));
        worker.shutdown().unwrap();
    }
}
