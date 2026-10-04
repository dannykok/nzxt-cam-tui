#[cfg(target_os = "linux")]
use std::{ffi::OsStr, fmt, future::Future, io, os::fd::AsFd, process};

#[cfg(target_os = "linux")]
use listenfd::ListenFd;
#[cfg(target_os = "linux")]
use nzxt_cam_hwd::{
    hardware::{HardwareError, HardwareManager, HardwareManagerError},
    host_control::{recover_service_production, restore_production},
    server::{ServiceError, serve_until},
};
#[cfg(target_os = "linux")]
use socket2::SockRef;
#[cfg(target_os = "linux")]
use tokio::{
    net::UnixListener,
    signal::unix::{SignalKind, signal},
};

#[cfg(target_os = "linux")]
#[tokio::main]
async fn main() {
    if let Err(error) = run_with(
        std::env::args_os().skip(1),
        restore_production,
        recover_service_production,
        run_service,
    )
    .await
    {
        eprintln!("nzxt-cam-hwd: {error}");
        process::exit(1);
    }
}

#[cfg(target_os = "linux")]
async fn run_with<I, A, R, C, S, ServiceFuture>(
    args: I,
    restore: R,
    recover_service: C,
    service: S,
) -> Result<(), RunError>
where
    I: IntoIterator<Item = A>,
    A: AsRef<OsStr>,
    R: FnOnce() -> Result<(), HardwareError>,
    C: FnOnce() -> Result<(), HardwareError>,
    S: FnOnce() -> ServiceFuture,
    ServiceFuture: Future<Output = Result<(), StartupError>>,
{
    match classify_args(args).map_err(RunError::Usage)? {
        Command::Service => service().await.map_err(RunError::Startup),
        Command::Restore => restore().map_err(RunError::Restore),
        Command::RecoverService => recover_service().map_err(RunError::RecoverService),
    }
}

#[cfg(target_os = "linux")]
fn classify_args<I, A>(args: I) -> Result<Command, UsageError>
where
    I: IntoIterator<Item = A>,
    A: AsRef<OsStr>,
{
    let mut args = args.into_iter();
    let Some(first) = args.next() else {
        return Ok(Command::Service);
    };
    let command = match first.as_ref() {
        value if value == OsStr::new("restore") => Command::Restore,
        value if value == OsStr::new("recover-service") => Command::RecoverService,
        _ => return Err(UsageError::UnknownCommand),
    };
    if args.next().is_some() {
        return Err(UsageError::UnexpectedArguments);
    }
    Ok(command)
}

#[cfg(target_os = "linux")]
async fn run_service() -> Result<(), StartupError> {
    validate_listen_pid(std::env::var_os("LISTEN_PID").as_deref(), process::id())
        .map_err(StartupError::InvalidListenPid)?;

    let mut inherited = ListenFd::from_env();
    validate_descriptor_count(inherited.len()).map_err(StartupError::InheritedDescriptors)?;

    let listener = inherited
        .take_unix_listener(0)
        .map_err(StartupError::InvalidInheritedListener)?
        .ok_or(StartupError::InheritedDescriptors(
            InheritedDescriptorError::Missing,
        ))?;
    require_listening_socket(&listener).map_err(StartupError::InvalidListenerState)?;
    listener
        .set_nonblocking(true)
        .map_err(StartupError::ConfigureInheritedListener)?;
    let listener =
        UnixListener::from_std(listener).map_err(StartupError::ConvertInheritedListener)?;

    let mut terminate =
        signal(SignalKind::terminate()).map_err(StartupError::InstallSignalHandler)?;
    let mut interrupt =
        signal(SignalKind::interrupt()).map_err(StartupError::InstallSignalHandler)?;
    let shutdown = async move {
        tokio::select! {
            _ = terminate.recv() => {}
            _ = interrupt.recv() => {}
        }
    };

    let hardware = HardwareManager::new().map_err(StartupError::Hardware)?;
    serve_until(listener, hardware, shutdown)
        .await
        .map_err(StartupError::Serve)?;
    Ok(())
}

#[cfg(target_os = "linux")]
fn validate_listen_pid(value: Option<&OsStr>, current_pid: u32) -> Result<(), ListenPidError> {
    let value = value.ok_or(ListenPidError::Missing)?;
    let parsed = value
        .to_str()
        .and_then(|value| value.parse::<u32>().ok())
        .filter(|pid| *pid != 0)
        .ok_or(ListenPidError::Malformed)?;
    if parsed != current_pid {
        return Err(ListenPidError::Mismatched);
    }
    Ok(())
}

#[cfg(target_os = "linux")]
fn validate_descriptor_count(actual: usize) -> Result<(), InheritedDescriptorError> {
    match actual {
        0 => Err(InheritedDescriptorError::Missing),
        1 => Ok(()),
        actual => Err(InheritedDescriptorError::Unexpected { actual }),
    }
}

#[cfg(target_os = "linux")]
fn require_listening_socket<S: AsFd>(socket: &S) -> Result<(), ListenerValidationError> {
    match SockRef::from(socket).is_listener() {
        Ok(true) => Ok(()),
        Ok(false) => Err(ListenerValidationError::NotListening),
        Err(error) => Err(ListenerValidationError::Inspect(error)),
    }
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Command {
    Service,
    Restore,
    RecoverService,
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum UsageError {
    UnknownCommand,
    UnexpectedArguments,
}

#[cfg(target_os = "linux")]
impl fmt::Display for UsageError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let reason = match self {
            Self::UnknownCommand => "unknown command",
            Self::UnexpectedArguments => "the selected command accepts no arguments",
        };
        write!(
            formatter,
            "{reason}; usage: nzxt-cam-hwd [restore|recover-service]"
        )
    }
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ListenPidError {
    Missing,
    Malformed,
    Mismatched,
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum InheritedDescriptorError {
    Missing,
    Unexpected { actual: usize },
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
enum ListenerValidationError {
    Inspect(io::Error),
    NotListening,
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
enum RunError {
    Usage(UsageError),
    Restore(HardwareError),
    RecoverService(HardwareError),
    Startup(StartupError),
}

#[cfg(target_os = "linux")]
impl fmt::Display for RunError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Usage(error) => write!(formatter, "{error}"),
            Self::Restore(error) => write!(formatter, "host-control restore failed: {error}"),
            Self::RecoverService(error) => {
                write!(formatter, "automatic service recovery failed: {error}")
            }
            Self::Startup(error) => write!(formatter, "{error}"),
        }
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
enum StartupError {
    InvalidListenPid(ListenPidError),
    InheritedDescriptors(InheritedDescriptorError),
    InvalidInheritedListener(io::Error),
    InvalidListenerState(ListenerValidationError),
    ConfigureInheritedListener(io::Error),
    ConvertInheritedListener(io::Error),
    InstallSignalHandler(io::Error),
    Hardware(HardwareManagerError),
    Serve(ServiceError),
}

#[cfg(target_os = "linux")]
impl fmt::Display for StartupError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidListenPid(ListenPidError::Missing) => {
                formatter.write_str("socket activation requires LISTEN_PID, but it was not present")
            }
            Self::InvalidListenPid(ListenPidError::Malformed) => formatter
                .write_str("socket activation LISTEN_PID must be a nonzero decimal process ID"),
            Self::InvalidListenPid(ListenPidError::Mismatched) => {
                formatter.write_str("socket activation LISTEN_PID does not match this process")
            }
            Self::InheritedDescriptors(InheritedDescriptorError::Missing) => formatter.write_str(
                "expected exactly one socket-activated Unix listener, but none was inherited",
            ),
            Self::InheritedDescriptors(InheritedDescriptorError::Unexpected { actual }) => write!(
                formatter,
                "expected exactly one socket-activated Unix listener, but inherited {actual}"
            ),
            Self::InvalidInheritedListener(error) => write!(
                formatter,
                "inherited descriptor is not a Unix stream socket: {error}"
            ),
            Self::InvalidListenerState(ListenerValidationError::Inspect(error)) => write!(
                formatter,
                "could not inspect the inherited Unix stream socket: {error}"
            ),
            Self::InvalidListenerState(ListenerValidationError::NotListening) => {
                formatter.write_str("inherited Unix stream socket is not in the listening state")
            }
            Self::ConfigureInheritedListener(error) => {
                write!(
                    formatter,
                    "could not make inherited listener nonblocking: {error}"
                )
            }
            Self::ConvertInheritedListener(error) => write!(
                formatter,
                "could not convert inherited listener for the Tokio runtime: {error}"
            ),
            Self::InstallSignalHandler(error) => {
                write!(
                    formatter,
                    "could not install shutdown signal handlers: {error}"
                )
            }
            Self::Hardware(error) => write!(formatter, "hardware manager startup failed: {error}"),
            Self::Serve(error) => write!(formatter, "socket service failed: {error}"),
        }
    }
}

#[cfg(target_os = "linux")]
impl std::error::Error for StartupError {}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("nzxt-cam-hwd: the socket-activated service binary is supported only on Linux");
    std::process::exit(1);
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use std::{
        cell::Cell,
        ffi::{OsStr, OsString},
        os::unix::{ffi::OsStringExt, net::UnixListener as StdUnixListener},
        sync::atomic::{AtomicU64, Ordering},
    };

    use super::*;

    static NEXT_SOCKET_ID: AtomicU64 = AtomicU64::new(0);

    #[test]
    fn arguments_select_only_service_restore_or_recover_service() {
        assert_eq!(
            classify_args(std::iter::empty::<&str>()),
            Ok(Command::Service)
        );
        assert_eq!(classify_args(["restore"]), Ok(Command::Restore));
        assert_eq!(
            classify_args(["recover-service"]),
            Ok(Command::RecoverService)
        );
        assert_eq!(
            classify_args(["recover-service", "extra"]),
            Err(UsageError::UnexpectedArguments)
        );
        assert_eq!(
            classify_args(["recover-service=force"]),
            Err(UsageError::UnknownCommand)
        );
        for invalid in [
            "probe-it8689-fan3",
            "probe-it8689-fan4",
            "probe-it8689-fan1",
            "probe-it8689-fan2",
            "probe-it8689-fan5",
            "probe-it8689-fan04",
            "probe-it8689-fan4=128",
            "handoff-it8689-fan04",
            "handoff-it8689-fan5",
            "restore-it8689-handoff=fan3",
            "trial-it8689-dual-auto=fan3",
            "trial-it8689-dual-auto-fan4",
            "restore-it8689-dual-trial=fan4",
        ] {
            assert_eq!(classify_args([invalid]), Err(UsageError::UnknownCommand));
        }
        assert_eq!(classify_args(["unknown"]), Err(UsageError::UnknownCommand));
        assert_eq!(
            classify_args(["restore", "extra"]),
            Err(UsageError::UnexpectedArguments)
        );
    }

    #[tokio::test]
    async fn retired_commands_are_unknown_without_service_or_recovery_writes() {
        for command in [
            "probe-it8689-fan3",
            "probe-it8689-fan4",
            "handoff-it8689-fan3",
            "handoff-it8689-fan4",
            "restore-it8689-handoff",
            "trial-it8689-dual-auto",
            "restore-it8689-dual-trial",
            "inspect-it8689-dual-log",
        ] {
            assert!(!UsageError::UnknownCommand.to_string().contains(command));
            for args in [vec![command], vec![command, "extra"]] {
                let result = run_with(
                    args,
                    || panic!("unknown command must not restore hardware"),
                    || panic!("unknown command must not recover hardware"),
                    || -> std::future::Ready<Result<(), StartupError>> {
                        panic!("unknown command must not start the service")
                    },
                )
                .await;
                assert!(matches!(
                    result,
                    Err(RunError::Usage(UsageError::UnknownCommand))
                ));
            }
        }
    }

    #[tokio::test]
    async fn no_arguments_run_the_socket_activated_service_path() {
        let restore_called = Cell::new(false);
        let service_called = Cell::new(false);

        let result = run_with(
            std::iter::empty::<&str>(),
            || {
                restore_called.set(true);
                Ok(())
            },
            || panic!("ordinary startup must not dispatch post-stop recovery"),
            || {
                service_called.set(true);
                std::future::ready(Ok(()))
            },
        )
        .await;

        assert!(result.is_ok());
        assert!(!restore_called.get());
        assert!(service_called.get());
    }

    #[tokio::test]
    async fn restore_bypasses_socket_listener_validation() {
        let restore_called = Cell::new(false);
        let listener_validation_called = Cell::new(false);

        let result = run_with(
            ["restore"],
            || {
                restore_called.set(true);
                Ok(())
            },
            || panic!("explicit restore must not dispatch automatic recovery"),
            || {
                listener_validation_called.set(true);
                std::future::ready(
                    validate_listen_pid(None, process::id())
                        .map_err(StartupError::InvalidListenPid),
                )
            },
        )
        .await;

        assert!(result.is_ok());
        assert!(restore_called.get());
        assert!(!listener_validation_called.get());
    }

    #[tokio::test]
    async fn restore_failure_is_bounded_and_propagated() {
        let result = run_with(
            ["restore"],
            || Err(HardwareError::new("x".repeat(10_000))),
            || panic!("wrong recovery helper"),
            || std::future::ready(Ok(())),
        )
        .await;

        let Err(RunError::Restore(error)) = result else {
            panic!("expected restore failure");
        };
        assert!(error.message().len() <= 512);
    }

    #[tokio::test]
    async fn automatic_recovery_is_distinct_from_explicit_restore_and_bypasses_socket_validation() {
        let automatic_called = Cell::new(false);
        run_with(
            ["recover-service"],
            || panic!("post-stop recovery must not disarm through explicit restore"),
            || {
                automatic_called.set(true);
                Ok(())
            },
            || -> std::future::Ready<Result<(), StartupError>> {
                panic!("post-stop recovery must not start the service")
            },
        )
        .await
        .unwrap();
        assert!(automatic_called.get());
    }

    #[tokio::test]
    async fn automatic_recovery_failure_is_propagated() {
        let result = run_with(
            ["recover-service"],
            || panic!("automatic recovery must not dispatch explicit restore"),
            || Err(HardwareError::new("recovery retained")),
            || -> std::future::Ready<Result<(), StartupError>> {
                panic!("automatic recovery must not start the service")
            },
        )
        .await;
        assert!(matches!(result, Err(RunError::RecoverService(_))));
    }

    #[test]
    fn listen_pid_validation_classifies_missing_malformed_and_mismatched_values() {
        assert_eq!(validate_listen_pid(None, 42), Err(ListenPidError::Missing));

        for value in [OsStr::new(""), OsStr::new("0"), OsStr::new("not-a-pid")] {
            assert_eq!(
                validate_listen_pid(Some(value), 42),
                Err(ListenPidError::Malformed)
            );
        }
        let non_utf8 = OsString::from_vec(vec![0xff]);
        assert_eq!(
            validate_listen_pid(Some(&non_utf8), 42),
            Err(ListenPidError::Malformed)
        );
        assert_eq!(
            validate_listen_pid(Some(OsStr::new("43")), 42),
            Err(ListenPidError::Mismatched)
        );
        assert_eq!(validate_listen_pid(Some(OsStr::new("42")), 42), Ok(()));
    }

    #[test]
    fn inherited_descriptor_count_requires_exactly_one() {
        assert_eq!(
            validate_descriptor_count(0),
            Err(InheritedDescriptorError::Missing)
        );
        assert_eq!(validate_descriptor_count(1), Ok(()));
        assert_eq!(
            validate_descriptor_count(2),
            Err(InheritedDescriptorError::Unexpected { actual: 2 })
        );
    }

    #[test]
    fn listener_validation_rejects_a_connected_unix_stream() {
        let sequence = NEXT_SOCKET_ID.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "nzxt-cam-hwd-listener-test-{}-{sequence}.sock",
            process::id()
        ));
        let _ = std::fs::remove_file(&path);
        let listener = StdUnixListener::bind(&path).unwrap();
        let (connected, _peer) = std::os::unix::net::UnixStream::pair().unwrap();

        assert!(require_listening_socket(&listener).is_ok());
        assert!(matches!(
            require_listening_socket(&connected),
            Err(ListenerValidationError::NotListening)
        ));

        drop(listener);
        std::fs::remove_file(path).unwrap();
    }
}
