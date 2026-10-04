//! Secure cross-process ownership for motherboard host control.
//!
//! The production path is fixed. The lock file is intentionally left in place:
//! `flock` ownership belongs to the open file description and closing the guard
//! releases it, so an unlocked stale file can be reused safely.

use std::{
    fs::{self, File, OpenOptions},
    io,
    os::{
        fd::AsRawFd,
        unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt},
    },
    path::Path,
};

use crate::hardware::{HardwareError, HardwareErrorKind};

pub const HOST_CONTROL_LOCK_PATH: &str = "/run/nzxt-cam/host-control.lock";

const LOCK_MODE: u32 = 0o600;
const OPEN_FLAGS: i32 = libc::O_NOFOLLOW | libc::O_CLOEXEC | libc::O_NONBLOCK;
const MAX_CREATE_RACE_ATTEMPTS: usize = 3;

#[derive(Clone, Copy)]
enum LockPurpose {
    Service,
    Restore,
}

impl LockPurpose {
    const fn busy_kind(self) -> HardwareErrorKind {
        match self {
            Self::Service => HardwareErrorKind::Unavailable,
            Self::Restore => HardwareErrorKind::RestoreRequired,
        }
    }

    const fn busy_message(self) -> &'static str {
        match self {
            Self::Service => {
                "host-control ownership lock is held by another service or restore process"
            }
            Self::Restore => {
                "host-control restore refused because the service or another restore process owns the lock"
            }
        }
    }
}

#[derive(Clone, Copy)]
struct LockMetadata {
    is_regular: bool,
    owner: u32,
    links: u64,
    mode: u32,
}

/// An exclusive process-lifetime host-control ownership guard.
///
/// The file is retained solely to keep its Linux advisory lock alive.
#[derive(Debug)]
pub(crate) struct HostControlLock {
    _file: File,
}

impl HostControlLock {
    pub(crate) fn acquire_service() -> Result<Self, HardwareError> {
        Self::acquire(
            Path::new(HOST_CONTROL_LOCK_PATH),
            true,
            LockPurpose::Service,
        )
    }

    pub(crate) fn acquire_restore() -> Result<Self, HardwareError> {
        Self::acquire(
            Path::new(HOST_CONTROL_LOCK_PATH),
            true,
            LockPurpose::Restore,
        )
    }

    fn acquire(
        path: &Path,
        enforce_root_owner: bool,
        purpose: LockPurpose,
    ) -> Result<Self, HardwareError> {
        let file = open_secure_lock_file(path, enforce_root_owner)?;

        // SAFETY: `file` owns a valid descriptor for the duration of the call.
        // `flock` does not retain the pointer (there is none), and the File kept
        // by this guard holds the successful lock until drop.
        let result = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
        if result != 0 {
            let error = io::Error::last_os_error();
            if error
                .raw_os_error()
                .is_some_and(|code| code == libc::EWOULDBLOCK || code == libc::EAGAIN)
            {
                return Err(HardwareError::with_kind(
                    purpose.busy_kind(),
                    purpose.busy_message(),
                ));
            }
            return Err(lock_io_error("acquire host-control ownership lock", error));
        }

        // Validate the locked descriptor once more. Namespace replacement does
        // not redirect this descriptor, and the root-owned runtime parent is
        // supplied by the ordered socket unit.
        validate_file_metadata(&file, enforce_root_owner)?;
        Ok(Self { _file: file })
    }

    #[cfg(test)]
    pub(crate) fn acquire_service_at(path: &Path) -> Result<Self, HardwareError> {
        Self::acquire(path, false, LockPurpose::Service)
    }

    #[cfg(test)]
    pub(crate) fn acquire_restore_at(path: &Path) -> Result<Self, HardwareError> {
        Self::acquire(path, false, LockPurpose::Restore)
    }
}

fn open_secure_lock_file(path: &Path, enforce_root_owner: bool) -> Result<File, HardwareError> {
    for _ in 0..MAX_CREATE_RACE_ATTEMPTS {
        match OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(OPEN_FLAGS)
            .open(path)
        {
            Ok(file) => {
                // Reject insecure pre-existing metadata rather than silently
                // repairing a file that may have been substituted.
                validate_file_metadata(&file, enforce_root_owner)?;
                set_and_validate_mode(&file, enforce_root_owner)?;
                return Ok(file);
            }
            Err(error) if error.kind() == io::ErrorKind::NotFound => {}
            Err(error) => return Err(lock_io_error("open host-control ownership lock", error)),
        }

        match OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .mode(LOCK_MODE)
            .custom_flags(OPEN_FLAGS)
            .open(path)
        {
            Ok(file) => {
                // The requested creation mode is filtered by umask, so apply
                // 0600 explicitly to the newly created descriptor.
                set_and_validate_mode(&file, enforce_root_owner)?;
                return Ok(file);
            }
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(lock_io_error("create host-control ownership lock", error)),
        }
    }

    Err(HardwareError::with_kind(
        HardwareErrorKind::Unavailable,
        "host-control ownership lock path changed repeatedly during creation",
    ))
}

fn set_and_validate_mode(file: &File, enforce_root_owner: bool) -> Result<(), HardwareError> {
    file.set_permissions(fs::Permissions::from_mode(LOCK_MODE))
        .map_err(|error| lock_io_error("set host-control ownership lock mode", error))?;
    validate_file_metadata(file, enforce_root_owner)
}

fn validate_file_metadata(file: &File, enforce_root_owner: bool) -> Result<(), HardwareError> {
    let metadata = file
        .metadata()
        .map_err(|error| lock_io_error("inspect host-control ownership lock", error))?;
    validate_lock_metadata(
        LockMetadata {
            is_regular: metadata.file_type().is_file(),
            owner: metadata.uid(),
            links: metadata.nlink(),
            mode: metadata.mode(),
        },
        enforce_root_owner,
    )
}

fn validate_lock_metadata(
    metadata: LockMetadata,
    enforce_root_owner: bool,
) -> Result<(), HardwareError> {
    if !metadata.is_regular {
        return Err(invalid_lock(
            "host-control ownership lock is not a regular file",
        ));
    }
    if enforce_root_owner && metadata.owner != 0 {
        return Err(invalid_lock(
            "host-control ownership lock is not owned by root",
        ));
    }
    if metadata.links != 1 {
        return Err(invalid_lock(
            "host-control ownership lock must have exactly one hard link",
        ));
    }
    if metadata.mode & 0o7777 != LOCK_MODE {
        return Err(invalid_lock(
            "host-control ownership lock must have mode 0600",
        ));
    }
    Ok(())
}

fn invalid_lock(message: &'static str) -> HardwareError {
    HardwareError::with_kind(HardwareErrorKind::InvalidData, message)
}

fn lock_io_error(operation: &str, error: io::Error) -> HardwareError {
    let kind = match error.kind() {
        io::ErrorKind::PermissionDenied => HardwareErrorKind::PermissionDenied,
        io::ErrorKind::NotFound => HardwareErrorKind::Unavailable,
        _ => HardwareErrorKind::Internal,
    };
    HardwareError::with_kind(kind, format!("{operation}: {error}"))
}

#[cfg(test)]
mod tests {
    use std::os::unix::fs::{PermissionsExt, symlink};

    use tempfile::TempDir;

    use super::*;

    #[test]
    fn independent_opens_contend_and_drop_releases_the_lock() {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("host-control.lock");
        let first = HostControlLock::acquire_service_at(&path).unwrap();

        let busy = HostControlLock::acquire_service_at(&path).unwrap_err();
        assert_eq!(busy.kind(), HardwareErrorKind::Unavailable);
        assert!(busy.message().contains("held"));
        assert_eq!(fs::metadata(&path).unwrap().mode() & 0o7777, 0o600);

        drop(first);
        let second = HostControlLock::acquire_restore_at(&path).unwrap();
        drop(second);
        assert!(path.exists(), "the reusable lock file should remain");
    }

    #[test]
    fn malformed_symlink_hardlink_and_wrong_mode_lock_files_are_rejected() {
        let temp = TempDir::new().unwrap();

        let directory = temp.path().join("directory-lock");
        fs::create_dir(&directory).unwrap();
        assert!(HostControlLock::acquire_service_at(&directory).is_err());

        let target = temp.path().join("target");
        fs::write(&target, b"").unwrap();
        fs::set_permissions(&target, fs::Permissions::from_mode(0o600)).unwrap();
        let link = temp.path().join("symlink-lock");
        symlink(&target, &link).unwrap();
        assert!(HostControlLock::acquire_service_at(&link).is_err());

        let hardlink = temp.path().join("hardlink-lock");
        fs::hard_link(&target, &hardlink).unwrap();
        let hardlink_error = HostControlLock::acquire_service_at(&target).unwrap_err();
        assert_eq!(hardlink_error.kind(), HardwareErrorKind::InvalidData);
        assert!(hardlink_error.message().contains("one hard link"));

        let wrong_mode = temp.path().join("wrong-mode-lock");
        fs::write(&wrong_mode, b"").unwrap();
        fs::set_permissions(&wrong_mode, fs::Permissions::from_mode(0o640)).unwrap();
        let mode_error = HostControlLock::acquire_service_at(&wrong_mode).unwrap_err();
        assert_eq!(mode_error.kind(), HardwareErrorKind::InvalidData);
        assert!(mode_error.message().contains("mode 0600"));
    }

    #[test]
    fn metadata_requires_root_owner_in_production() {
        let error = validate_lock_metadata(
            LockMetadata {
                is_regular: true,
                owner: 1000,
                links: 1,
                mode: 0o100600,
            },
            true,
        )
        .unwrap_err();
        assert_eq!(error.kind(), HardwareErrorKind::InvalidData);
        assert!(error.message().contains("owned by root"));
    }
}
