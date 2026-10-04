pub mod app;
pub mod backend;
pub mod config;
pub mod ipc;
pub mod model;
pub mod profile;
pub mod theme;
pub mod ui;

use std::{
    collections::BTreeMap,
    io,
    sync::mpsc::{self, Receiver, Sender, TryRecvError},
    thread,
    time::{Duration, Instant},
};

use app::{App, AppCommand, CurveKey};
use backend::{BackendCancellation, BackendError, BackendErrorKind, HardwareBackend};
use config::{AppConfig, ConfigStore};
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use model::{
    CurvePoint, DeviceId, HardwareSnapshot, HostChannelPolicy, HostControlPolicy, KrakenDisplayMode,
};
use nzxt_cam_protocol::{
    MonitoringActivationOutcome, MonitoringDisplaySelection, MonitoringFirmwareCurve,
};
use profile::{
    ProfileBinding, ProfileCurve, ProfileIntent, ProfileLibrary, ProfileRef, ProfileStore,
    ProfileTarget,
};
use ratatui::DefaultTerminal;

const INPUT_POLL_INTERVAL: Duration = Duration::from_millis(100);
const TELEMETRY_REFRESH_INTERVAL: Duration = Duration::from_millis(800);

pub fn run<B: HardwareBackend>(terminal: &mut DefaultTerminal, backend: B) -> io::Result<()> {
    let backend_name = backend.name();
    let worker = BackendWorker::spawn(backend)?;
    let (config, config_warning) = load_config_store();
    let (profile_store, library, profile_warning) =
        load_profile_store(backend_name, &config, config_warning.is_none());
    let mut app = App::with_profile_library(
        HardwareSnapshot {
            devices: Vec::new(),
            sequence: 0,
            host_control: Default::default(),
            kraken_display: Default::default(),
            monitoring: Default::default(),
        },
        backend_name,
        library,
        config,
    );
    let warnings = [profile_warning, config_warning]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    if !warnings.is_empty() {
        app.apply_failed(warnings.join(" • "));
    }
    let mut refresh = RefreshState {
        in_flight: worker.request_refresh(),
        has_snapshot: false,
        last_refresh: Instant::now(),
    };
    let mut pending_applies = BTreeMap::new();
    let mut host_operations = HostOperationState::default();
    let mut exit_when_idle = false;

    while !app.exit {
        process_backend_events(
            &mut app,
            &worker,
            &mut refresh,
            &mut pending_applies,
            &mut host_operations,
            profile_store.as_ref(),
        );
        if exit_when_idle
            && pending_applies.is_empty()
            && host_operations.pending.is_none()
            && !host_operations.queued_stop
            && !app.activation_busy
            && !app.auto_resume_busy
            && !app.display_operation_busy
        {
            app.exit = true;
        }
        let terminal_size = terminal.size()?;
        let terminal_too_small =
            terminal_size.width < ui::MIN_WIDTH || terminal_size.height < ui::MIN_HEIGHT;
        terminal.draw(|frame| ui::draw(frame, &app))?;

        if event::poll(INPUT_POLL_INTERVAL)?
            && let Event::Key(key_event) = event::read()?
            && should_handle_key_event(&key_event)
        {
            if exit_when_idle || terminal_too_small {
                if is_quit_key(&key_event) {
                    app.exit = true;
                }
            } else if let Some(command) = app.handle_key_event(key_event) {
                send_app_command(
                    &mut app,
                    &worker,
                    profile_store.as_ref(),
                    &mut pending_applies,
                    &mut host_operations,
                    command,
                );
            }
            begin_graceful_exit(
                &mut app,
                pending_write_count(&pending_applies),
                &host_operations,
                &mut exit_when_idle,
            );
        }

        if !refresh.in_flight && refresh.last_refresh.elapsed() >= TELEMETRY_REFRESH_INTERVAL {
            refresh.in_flight = worker.request_refresh();
            refresh.last_refresh = Instant::now();
            if !refresh.in_flight {
                app.apply_failed(format!("{} is unavailable", control_connection(&app)));
            }
        }
    }

    Ok(())
}

fn process_backend_events(
    app: &mut App,
    worker: &BackendWorker,
    refresh: &mut RefreshState,
    pending_applies: &mut BTreeMap<CurveKey, PendingApply>,
    host_operations: &mut HostOperationState,
    profile_store: Option<&ProfileStore>,
) {
    loop {
        match worker.try_event() {
            Ok(WorkerEvent::Refreshed(result)) => {
                refresh.in_flight = false;
                refresh.last_refresh = Instant::now();
                match result {
                    Ok(snapshot) => {
                        if !refresh.has_snapshot {
                            app.backend_connected();
                            refresh.has_snapshot = true;
                        }
                        app.update_telemetry(snapshot);
                    }
                    Err(error) => {
                        if error.invalidates_live_curve_claims() {
                            app.invalidate_live_curve_claims();
                            refresh.has_snapshot = false;
                        }
                        if error.kind() == BackendErrorKind::RestoreRequired {
                            app.mark_host_restore_required();
                        } else {
                            app.mark_host_status_unknown();
                        }
                        app.apply_failed(format!("Telemetry refresh failed: {error}"));
                    }
                }
            }
            Ok(WorkerEvent::MonitoringActivated(result)) => {
                app.activation_result(result);
                if !refresh.in_flight {
                    refresh.in_flight = worker.request_refresh();
                }
            }
            Ok(WorkerEvent::MonitoringAutoResumeSet(result)) => {
                app.auto_resume_busy = false;
                match result {
                    Ok(()) => {
                        app.status = app::StatusMessage {
                            kind: app::StatusKind::Info,
                            text: "Future-restart setting saved; awaiting service snapshot".into(),
                        }
                    }
                    Err(error) => {
                        app.apply_failed(format!("Future-restart setting not confirmed: {error}"))
                    }
                }
                if !refresh.in_flight {
                    refresh.in_flight = worker.request_refresh();
                }
            }
            Ok(WorkerEvent::DisplaySet(result)) => {
                match result {
                    Ok(()) => app.display_change_succeeded(),
                    Err(error) => app.display_change_failed(&error.to_string()),
                }
                if !refresh.in_flight {
                    refresh.in_flight = worker.request_refresh();
                }
            }
            Ok(WorkerEvent::Applied {
                key,
                request,
                result,
            }) => {
                let apply_succeeded = result.is_ok();
                if result
                    .as_ref()
                    .is_err_and(BackendError::invalidates_live_curve_claims)
                {
                    app.invalidate_live_curve_claims();
                    app.mark_host_status_unknown();
                    refresh.has_snapshot = false;
                }
                match result {
                    Ok(()) => {
                        app.apply_succeeded(key.clone(), request.points);
                        remember_success(app, profile_store, &[request.binding]);
                    }
                    Err(error) if error.write_outcome_unknown() => {
                        app.apply_outcome_unknown(&key, format!("Curve apply failed: {error}"))
                    }
                    Err(error) => app.apply_failed(format!("Curve apply failed: {error}")),
                }

                let queued = if apply_succeeded {
                    pending_applies
                        .get_mut(&key)
                        .and_then(|pending| pending.queued.take())
                } else {
                    pending_applies.remove(&key);
                    None
                };
                if let Some(request) = queued {
                    if dispatch_curve(app, worker, profile_store, key.clone(), request) {
                        app.queued_curve_write_started();
                    } else {
                        pending_applies.remove(&key);
                    }
                } else {
                    pending_applies.remove(&key);
                }
            }
            Ok(WorkerEvent::HostStarted {
                policy,
                bindings,
                result,
            }) => {
                host_operations.pending = None;
                if result
                    .as_ref()
                    .is_err_and(BackendError::invalidates_live_curve_claims)
                {
                    app.invalidate_live_curve_claims();
                    app.mark_host_status_unknown();
                    refresh.has_snapshot = false;
                }
                match result {
                    Ok(()) => {
                        app.host_start_succeeded(policy);
                        remember_success(app, profile_store, &bindings);
                    }
                    Err(error) => app.host_operation_failed(&error),
                }
                finish_host_mutation(app, worker, host_operations);
            }
            Ok(WorkerEvent::HostUpdated {
                channel_policies,
                bindings,
                result,
            }) => {
                host_operations.pending = None;
                if result
                    .as_ref()
                    .is_err_and(BackendError::invalidates_live_curve_claims)
                {
                    app.invalidate_live_curve_claims();
                    app.mark_host_status_unknown();
                    refresh.has_snapshot = false;
                }
                match result {
                    Ok(()) => {
                        app.host_update_succeeded(channel_policies);
                        remember_success(app, profile_store, &bindings);
                    }
                    Err(error) => app.host_operation_failed(&error),
                }
                finish_host_mutation(app, worker, host_operations);
            }
            Ok(WorkerEvent::HostStopped(result)) => {
                host_operations.pending = None;
                if result
                    .as_ref()
                    .is_err_and(BackendError::invalidates_live_curve_claims)
                {
                    app.invalidate_live_curve_claims();
                    app.mark_host_status_unknown();
                    refresh.has_snapshot = false;
                }
                match result {
                    Ok(()) => app.host_stop_succeeded(),
                    Err(error) => app.host_operation_failed(&error),
                }
                app.host_operation_busy = false;
            }
            Err(TryRecvError::Empty) => break,
            Err(TryRecvError::Disconnected) => {
                refresh.in_flight = false;
                refresh.has_snapshot = false;
                app.invalidate_live_curve_claims();
                app.mark_host_status_unknown();
                pending_applies.clear();
                host_operations.pending = None;
                host_operations.queued_stop = false;
                app.host_operation_busy = false;
                app.display_operation_busy = false;
                app.activation_busy = false;
                app.auto_resume_busy = false;
                app.apply_failed(format!("{} stopped unexpectedly", control_connection(app)));
                break;
            }
        }
    }
}

fn control_connection(app: &App) -> &'static str {
    if app.backend_name == "DEMO" {
        "Simulator"
    } else {
        "Monitoring service connection"
    }
}

fn finish_host_mutation(app: &mut App, worker: &BackendWorker, state: &mut HostOperationState) {
    if state.queued_stop {
        state.queued_stop = false;
        if worker.stop_host_control() {
            state.pending = Some(PendingHostOperation::Stop);
            app.status = app::StatusMessage {
                kind: app::StatusKind::Info,
                text: "Stopping fan control…".into(),
            };
            return;
        }
        app.apply_failed(format!(
            "{} is unavailable; stop outcome unknown",
            control_connection(app)
        ));
        app.mark_host_status_unknown();
    }
    app.host_operation_busy = false;
}

fn prepare_profiles(
    app: &mut App,
    store: Option<&ProfileStore>,
    intents: &[ProfileIntent],
) -> Option<Vec<ProfileBinding>> {
    let Some(store) = store else {
        app.apply_failed("Apply not sent: profile storage is unavailable");
        return None;
    };
    match store.prepare(intents) {
        Ok((library, bindings)) => {
            app.set_profile_library(library);
            Some(bindings)
        }
        Err(error) => {
            app.apply_failed(format!("Apply not sent: could not save profiles: {error}"));
            None
        }
    }
}

fn remember_success(app: &mut App, store: Option<&ProfileStore>, bindings: &[ProfileBinding]) {
    if bindings.is_empty() {
        return;
    }
    let result = store
        .ok_or_else(|| "profile storage is unavailable".to_owned())
        .and_then(|store| {
            store
                .bind_success(bindings)
                .map_err(|error| error.to_string())
        });
    match result {
        Ok(library) => {
            app.set_profile_library(library);
            app.profile_bindings_applied(bindings);
        }
        Err(error) => {
            // Hardware acceptance is real even if remembering its selection
            // failed. The exact proposed curves were already durably saved.
            app.profile_bindings_applied(bindings);
            app.apply_failed(format!(
                "Applied, but profile selection could not be saved: {error}"
            ));
        }
    }
}

fn dispatch_curve(
    app: &mut App,
    worker: &BackendWorker,
    store: Option<&ProfileStore>,
    key: CurveKey,
    request: CurveRequest,
) -> bool {
    let intent = ProfileIntent {
        target: ProfileTarget::Firmware {
            device_id: key.0.clone(),
            channel_id: key.1.clone(),
        },
        curve: ProfileCurve::Firmware(request.points.clone()),
        preferred: request.profile,
    };
    let Some(mut bindings) = prepare_profiles(app, store, &[intent]) else {
        return false;
    };
    let prepared = PreparedCurveRequest {
        points: request.points,
        binding: bindings.remove(0),
    };
    if worker.apply_curve(key, prepared) {
        true
    } else {
        app.apply_failed(format!(
            "{} is unavailable; profile saved, apply not sent",
            control_connection(app)
        ));
        false
    }
}

fn send_app_command(
    app: &mut App,
    worker: &BackendWorker,
    profile_store: Option<&ProfileStore>,
    pending_applies: &mut BTreeMap<CurveKey, PendingApply>,
    host_operations: &mut HostOperationState,
    command: AppCommand,
) {
    match command {
        AppCommand::ActivateMonitoring {
            firmware_curves,
            host_policy,
            display,
        } => {
            if worker.activate_monitoring(firmware_curves, host_policy, display) {
                app.activation_busy = true;
            } else {
                // Nothing was queued or sent. Require a fresh snapshot before
                // accepting another explicit activation attempt.
                app.activation_attempted = false;
                app.snapshot_received = false;
                app.apply_failed(
                    "Service unavailable; activation not confirmed — check service status",
                );
            }
        }
        AppCommand::SetMonitoringAutoResume { enabled } => {
            if worker.set_monitoring_auto_resume(enabled) {
                app.auto_resume_busy = true;
                app.status = app::StatusMessage {
                    kind: app::StatusKind::Info,
                    text: "Saving future-restart setting…".into(),
                };
            } else {
                app.apply_failed("Service unavailable; future-restart setting not confirmed");
            }
        }
        AppCommand::SaveConfirmApply { enabled } => {
            match ConfigStore::from_environment()
                .and_then(|store| store.save_confirm_apply(enabled))
            {
                Ok(()) => {
                    app.confirm_apply = enabled;
                    app.status = app::StatusMessage {
                        kind: app::StatusKind::Success,
                        text: "App confirmation preference saved".into(),
                    };
                }
                Err(error) => app.apply_failed(format!("App preference not saved: {error}")),
            }
        }
        AppCommand::ApplyCurve {
            key,
            points,
            profile,
        } => {
            let request = CurveRequest { points, profile };
            if let Some(pending) = pending_applies.get_mut(&key) {
                pending.queued = Some(request);
                app.curve_write_queued();
                return;
            }
            let pending_key = key.clone();
            if dispatch_curve(app, worker, profile_store, key, request) {
                pending_applies.insert(pending_key, PendingApply::default());
            }
        }
        AppCommand::RenameProfile { id, name } => {
            let result = profile_store
                .ok_or_else(|| "profile storage is unavailable".to_owned())
                .and_then(|store| store.rename(id, &name).map_err(|error| error.to_string()));
            match result {
                Ok(library) => {
                    app.set_profile_library(library);
                    app.rename_succeeded(id);
                }
                Err(error) => app.apply_failed(format!("Rename failed: {error}")),
            }
        }
        AppCommand::StartHostControl { policy } => {
            if host_operations.pending.is_some() {
                app.apply_failed("A host-control operation is already in progress");
                return;
            }
            let intents = app.host_profile_intents(&policy.channels);
            let Some(bindings) = prepare_profiles(app, profile_store, &intents) else {
                return;
            };
            if worker.start_host_control(policy, bindings) {
                host_operations.pending = Some(PendingHostOperation::Start);
                app.host_operation_busy = true;
            } else {
                app.apply_failed(format!(
                    "{} is unavailable; profiles saved, Start not sent",
                    control_connection(app)
                ));
            }
        }
        AppCommand::UpdateHostControl { channel_policies } => {
            if host_operations.pending.is_some() {
                app.apply_failed("A host-control operation is already in progress");
                return;
            }
            let intents = app.host_profile_intents(&channel_policies);
            let Some(bindings) = prepare_profiles(app, profile_store, &intents) else {
                return;
            };
            if worker.update_host_control(channel_policies, bindings) {
                host_operations.pending = Some(PendingHostOperation::Update);
                app.host_operation_busy = true;
            } else {
                app.apply_failed(format!(
                    "{} is unavailable; profiles saved, update not sent",
                    control_connection(app)
                ));
            }
        }
        AppCommand::RememberHostProfiles { channel_policies } => {
            let known_applied = app.host_status_trusted
                && app.host_control_state == model::HostControlState::Running
                && !app.host_operation_busy
                && host_operations.pending.is_none()
                && !channel_policies.is_empty()
                && app
                    .snapshot
                    .host_control
                    .active_policy
                    .as_ref()
                    .is_some_and(|policy| {
                        channel_policies
                            .iter()
                            .all(|accepted| policy.channels.contains(accepted))
                    });
            if !known_applied {
                app.apply_failed("Fan status changed; choose Apply again");
                return;
            }
            let intents = app.host_profile_intents(&channel_policies);
            if let Some(bindings) = prepare_profiles(app, profile_store, &intents) {
                app.status = app::StatusMessage {
                    kind: app::StatusKind::Success,
                    text: "Already applied; profiles saved".into(),
                };
                remember_success(app, profile_store, &bindings);
            }
        }
        AppCommand::SetKrakenDisplay { device_id, mode } => {
            if app.display_operation_busy {
                app.apply_failed("A display change is already in progress");
            } else if !app.host_status_trusted
                || app.snapshot.kraken_display.device_id.as_ref() != Some(&device_id)
            {
                app.apply_failed("Kraken display changed or is unavailable; refresh and try again");
            } else if worker.set_kraken_display(device_id, mode) {
                app.display_operation_busy = true;
                app.status = app::StatusMessage {
                    kind: app::StatusKind::Info,
                    text: format!("Changing Kraken display to {}…", mode.title()),
                };
            } else {
                app.apply_failed(format!("{} is unavailable", control_connection(app)));
            }
        }
        AppCommand::StopHostControl => {
            if host_operations.pending.is_some() {
                if !matches!(host_operations.pending, Some(PendingHostOperation::Stop)) {
                    host_operations.queued_stop = true;
                    app.status = app::StatusMessage {
                        kind: app::StatusKind::Info,
                        text: "Stop queued after fan operation…".into(),
                    };
                }
            } else if worker.stop_host_control() {
                host_operations.pending = Some(PendingHostOperation::Stop);
                app.host_operation_busy = true;
            } else {
                app.apply_failed(format!("{} is unavailable", control_connection(app)));
            }
        }
    }
}

fn load_profile_store(
    backend_name: &str,
    config: &AppConfig,
    config_readable: bool,
) -> (Option<ProfileStore>, ProfileLibrary, Option<String>) {
    let store = if backend_name == "DEMO" {
        ProfileStore::for_demo()
    } else {
        ProfileStore::from_environment().map(|store| {
            if config_readable {
                store.with_legacy_defaults(config.default_profiles.clone())
            } else {
                // Only migration depends on old config selections. A v2 library
                // is self-contained and remains usable with safe confirmation.
                store.with_unavailable_legacy_defaults()
            }
        })
    };
    match store {
        Ok(store) => match store.load() {
            Ok(library) => (Some(store), library, None),
            Err(error) => (
                Some(store),
                ProfileLibrary::default(),
                Some(format!("Profiles could not be loaded: {error}")),
            ),
        },
        Err(error) => (
            None,
            ProfileLibrary::default(),
            Some(format!("Profile storage is unavailable: {error}")),
        ),
    }
}

fn load_config_store() -> (AppConfig, Option<String>) {
    match ConfigStore::from_environment().and_then(|store| store.load()) {
        Ok(config) => (config, None),
        Err(error) => (
            AppConfig::default(),
            Some(format!("Configuration could not be loaded: {error}")),
        ),
    }
}

fn is_quit_key(key_event: &KeyEvent) -> bool {
    matches!(key_event.code, KeyCode::Esc | KeyCode::Char('q' | 'Q'))
        || (key_event.modifiers.contains(KeyModifiers::CONTROL)
            && matches!(key_event.code, KeyCode::Char('c' | 'C')))
}

fn should_handle_key_event(key_event: &KeyEvent) -> bool {
    match key_event.kind {
        KeyEventKind::Press => true,
        KeyEventKind::Repeat => matches!(
            key_event.code,
            KeyCode::Left | KeyCode::Right | KeyCode::Up | KeyCode::Down | KeyCode::Backspace
        ),
        KeyEventKind::Release => false,
    }
}

fn begin_graceful_exit(
    app: &mut App,
    pending_applies: usize,
    pending_host_operation: &HostOperationState,
    exit_when_idle: &mut bool,
) {
    if !app.exit || *exit_when_idle {
        return;
    }
    let pending = pending_applies
        + usize::from(pending_host_operation.pending.is_some())
        + usize::from(pending_host_operation.queued_stop)
        + usize::from(app.display_operation_busy)
        + usize::from(app.activation_busy)
        + usize::from(app.auto_resume_busy);
    if pending == 0 {
        return;
    }

    app.exit = false;
    *exit_when_idle = true;
    app.waiting_for_pending_operations(pending);
}

#[derive(Clone)]
struct CurveRequest {
    points: Vec<CurvePoint>,
    profile: Option<ProfileRef>,
}

#[derive(Clone)]
struct PreparedCurveRequest {
    points: Vec<CurvePoint>,
    binding: ProfileBinding,
}

#[derive(Default)]
struct PendingApply {
    queued: Option<CurveRequest>,
}

#[derive(Clone)]
enum PendingHostOperation {
    Start,
    Update,
    Stop,
}

#[derive(Default)]
struct HostOperationState {
    pending: Option<PendingHostOperation>,
    queued_stop: bool,
}

struct RefreshState {
    in_flight: bool,
    has_snapshot: bool,
    last_refresh: Instant,
}

fn pending_write_count(pending: &BTreeMap<CurveKey, PendingApply>) -> usize {
    pending
        .values()
        .map(|apply| 1 + usize::from(apply.queued.is_some()))
        .sum()
}

enum WorkerCommand {
    ActivateMonitoring {
        curves: Vec<MonitoringFirmwareCurve>,
        host: Option<HostControlPolicy>,
        display: Option<MonitoringDisplaySelection>,
    },
    SetMonitoringAutoResume(bool),
    Refresh,
    SetKrakenDisplay {
        device_id: DeviceId,
        mode: KrakenDisplayMode,
    },
    ApplyCurve {
        key: CurveKey,
        request: PreparedCurveRequest,
    },
    StartHostControl {
        policy: HostControlPolicy,
        bindings: Vec<ProfileBinding>,
    },
    UpdateHostControl {
        channel_policies: Vec<HostChannelPolicy>,
        bindings: Vec<ProfileBinding>,
    },
    StopHostControl,
    Shutdown,
}

enum WorkerEvent {
    MonitoringActivated(Result<Vec<MonitoringActivationOutcome>, BackendError>),
    MonitoringAutoResumeSet(Result<(), BackendError>),
    Refreshed(Result<HardwareSnapshot, BackendError>),
    DisplaySet(Result<(), BackendError>),
    Applied {
        key: CurveKey,
        request: PreparedCurveRequest,
        result: Result<(), BackendError>,
    },
    HostStarted {
        policy: HostControlPolicy,
        bindings: Vec<ProfileBinding>,
        result: Result<(), BackendError>,
    },
    HostUpdated {
        channel_policies: Vec<HostChannelPolicy>,
        bindings: Vec<ProfileBinding>,
        result: Result<(), BackendError>,
    },
    HostStopped(Result<(), BackendError>),
}

struct BackendWorker {
    commands: Sender<WorkerCommand>,
    events: Receiver<WorkerEvent>,
    cancellation: Option<BackendCancellation>,
    worker_thread: Option<thread::JoinHandle<()>>,
}

impl BackendWorker {
    fn spawn<B: HardwareBackend>(mut backend: B) -> io::Result<Self> {
        let (command_sender, command_receiver) = mpsc::channel();
        let (event_sender, event_receiver) = mpsc::channel();
        let cancellation = backend.cancellation_handle();

        let worker_thread = thread::Builder::new()
            .name("nzxt-hardware-backend".into())
            .spawn(move || {
                while let Ok(command) = command_receiver.recv() {
                    let event = match command {
                        WorkerCommand::Refresh => WorkerEvent::Refreshed(backend.refresh()),
                        WorkerCommand::ActivateMonitoring {
                            curves,
                            host,
                            display,
                        } => WorkerEvent::MonitoringActivated(backend.activate_monitoring(
                            &curves,
                            host.as_ref(),
                            display.as_ref(),
                        )),
                        WorkerCommand::SetMonitoringAutoResume(enabled) => {
                            WorkerEvent::MonitoringAutoResumeSet(
                                backend.set_monitoring_auto_resume(enabled),
                            )
                        }
                        WorkerCommand::SetKrakenDisplay { device_id, mode } => {
                            WorkerEvent::DisplaySet(backend.set_kraken_display(&device_id, mode))
                        }
                        WorkerCommand::ApplyCurve { key, request } => {
                            let result = backend.apply_curve(&key.0, &key.1, &request.points);
                            WorkerEvent::Applied {
                                key,
                                request,
                                result,
                            }
                        }
                        WorkerCommand::StartHostControl { policy, bindings } => {
                            let result = backend.start_host_control(&policy);
                            WorkerEvent::HostStarted {
                                policy,
                                bindings,
                                result,
                            }
                        }
                        WorkerCommand::UpdateHostControl {
                            channel_policies,
                            bindings,
                        } => {
                            let result = backend.update_host_control(&channel_policies);
                            WorkerEvent::HostUpdated {
                                channel_policies,
                                bindings,
                                result,
                            }
                        }
                        WorkerCommand::StopHostControl => {
                            WorkerEvent::HostStopped(backend.stop_host_control())
                        }
                        WorkerCommand::Shutdown => break,
                    };
                    if event_sender.send(event).is_err() {
                        break;
                    }
                }
            })?;

        Ok(Self {
            commands: command_sender,
            events: event_receiver,
            cancellation,
            worker_thread: Some(worker_thread),
        })
    }

    fn activate_monitoring(
        &self,
        curves: Vec<MonitoringFirmwareCurve>,
        host: Option<HostControlPolicy>,
        display: Option<MonitoringDisplaySelection>,
    ) -> bool {
        self.commands
            .send(WorkerCommand::ActivateMonitoring {
                curves,
                host,
                display,
            })
            .is_ok()
    }

    fn set_monitoring_auto_resume(&self, enabled: bool) -> bool {
        self.commands
            .send(WorkerCommand::SetMonitoringAutoResume(enabled))
            .is_ok()
    }

    fn request_refresh(&self) -> bool {
        self.commands.send(WorkerCommand::Refresh).is_ok()
    }

    fn set_kraken_display(&self, device_id: DeviceId, mode: KrakenDisplayMode) -> bool {
        self.commands
            .send(WorkerCommand::SetKrakenDisplay { device_id, mode })
            .is_ok()
    }

    fn apply_curve(&self, key: CurveKey, request: PreparedCurveRequest) -> bool {
        self.commands
            .send(WorkerCommand::ApplyCurve { key, request })
            .is_ok()
    }

    fn start_host_control(&self, policy: HostControlPolicy, bindings: Vec<ProfileBinding>) -> bool {
        self.commands
            .send(WorkerCommand::StartHostControl { policy, bindings })
            .is_ok()
    }

    fn update_host_control(
        &self,
        channel_policies: Vec<HostChannelPolicy>,
        bindings: Vec<ProfileBinding>,
    ) -> bool {
        self.commands
            .send(WorkerCommand::UpdateHostControl {
                channel_policies,
                bindings,
            })
            .is_ok()
    }

    fn stop_host_control(&self) -> bool {
        self.commands.send(WorkerCommand::StopHostControl).is_ok()
    }

    fn try_event(&self) -> Result<WorkerEvent, TryRecvError> {
        self.events.try_recv()
    }
}

impl Drop for BackendWorker {
    fn drop(&mut self) {
        if let Some(cancellation) = &self.cancellation {
            cancellation.cancel();
        }
        let _ = self.commands.send(WorkerCommand::Shutdown);
        if let Some(worker_thread) = self.worker_thread.take() {
            let _ = worker_thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use model::HostControlState;

    struct TempProfiles {
        directory: std::path::PathBuf,
        store: ProfileStore,
    }

    impl TempProfiles {
        fn new() -> Self {
            static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let stamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let directory = std::env::temp_dir().join(format!(
                "nzxt-profile-runtime-{}-{stamp}-{}",
                std::process::id(),
                NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
            ));
            std::fs::create_dir(&directory).unwrap();
            Self {
                store: ProfileStore::at(directory.join("profiles.json")),
                directory,
            }
        }
    }

    impl Drop for TempProfiles {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.directory).unwrap();
        }
    }

    fn fake_prepared(key: &CurveKey, points: Vec<CurvePoint>) -> PreparedCurveRequest {
        PreparedCurveRequest {
            binding: ProfileBinding {
                target: ProfileTarget::Firmware {
                    device_id: key.0.clone(),
                    channel_id: key.1.clone(),
                },
                profile: ProfileRef::Custom(1),
                curve: ProfileCurve::Firmware(points.clone()),
            },
            points,
        }
    }

    fn recording_worker() -> (BackendWorker, Receiver<WorkerCommand>, Sender<WorkerEvent>) {
        let (commands, received) = mpsc::channel();
        let (events, incoming) = mpsc::channel();
        (
            BackendWorker {
                commands,
                events: incoming,
                cancellation: None,
                worker_thread: None,
            },
            received,
            events,
        )
    }

    fn drain_events(
        app: &mut App,
        worker: &BackendWorker,
        store: &ProfileStore,
        pending: &mut BTreeMap<CurveKey, PendingApply>,
        state: &mut HostOperationState,
    ) {
        let mut refresh = RefreshState {
            in_flight: false,
            has_snapshot: true,
            last_refresh: Instant::now(),
        };
        process_backend_events(app, worker, &mut refresh, pending, state, Some(store));
    }

    fn empty_app() -> App {
        App::new(
            HardwareSnapshot {
                devices: Vec::new(),
                sequence: 0,
                host_control: Default::default(),
                kraken_display: Default::default(),
                monitoring: model::MonitoringSnapshot {
                    opted_in: true,
                    ..Default::default()
                },
            },
            "TEST",
        )
    }

    fn event_worker() -> (BackendWorker, Sender<WorkerEvent>) {
        let (command_sender, _command_receiver) = mpsc::channel();
        let (event_sender, event_receiver) = mpsc::channel();
        (
            BackendWorker {
                commands: command_sender,
                events: event_receiver,
                cancellation: None,
                worker_thread: None,
            },
            event_sender,
        )
    }

    fn process_test_events(
        app: &mut App,
        worker: &BackendWorker,
        has_snapshot: &mut bool,
        pending_applies: &mut BTreeMap<CurveKey, PendingApply>,
    ) {
        let mut refresh = RefreshState {
            in_flight: true,
            has_snapshot: *has_snapshot,
            last_refresh: Instant::now(),
        };
        let mut host_operations = HostOperationState::default();
        process_backend_events(
            app,
            worker,
            &mut refresh,
            pending_applies,
            &mut host_operations,
            None,
        );
        *has_snapshot = refresh.has_snapshot;
    }

    fn app_with_applied_dirty_firmware_curve() -> App {
        let mut backend = crate::backend::DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.monitoring.opted_in = true;
        let mut app = App::new(snapshot, "TEST");
        let key = app.active_curve_key().unwrap();
        let applied = app.active_channel().unwrap().points.clone();
        app.apply_succeeded(key, applied);
        app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::NONE));
        assert!(app.active_curve_is_verified());
        assert!(app.is_active_curve_dirty());
        app
    }

    fn firmware_editor_points(app: &App) -> Vec<Vec<CurvePoint>> {
        app.snapshot
            .devices
            .iter()
            .flat_map(|device| &device.cooling_channels)
            .map(|channel| channel.points.clone())
            .collect()
    }

    fn process_worker_disconnect(
        app: &mut App,
        pending_applies: &mut BTreeMap<CurveKey, PendingApply>,
        host_operations: &mut HostOperationState,
    ) -> RefreshState {
        let (worker, events) = event_worker();
        drop(events);
        let mut refresh = RefreshState {
            in_flight: true,
            has_snapshot: true,
            last_refresh: Instant::now(),
        };
        process_backend_events(
            app,
            &worker,
            &mut refresh,
            pending_applies,
            host_operations,
            None,
        );
        refresh
    }

    struct CancelAwareBackend {
        cancellation: BackendCancellation,
    }

    impl HardwareBackend for CancelAwareBackend {
        fn name(&self) -> &'static str {
            "CANCEL-TEST"
        }

        fn cancellation_handle(&self) -> Option<BackendCancellation> {
            Some(self.cancellation.clone())
        }

        fn refresh(&mut self) -> Result<HardwareSnapshot, BackendError> {
            while !self.cancellation.is_cancelled() {
                thread::sleep(Duration::from_millis(5));
            }
            Err(BackendError::new("cancelled"))
        }

        fn apply_curve(
            &mut self,
            _device_id: &crate::model::DeviceId,
            _channel_id: &crate::model::ChannelId,
            _points: &[CurvePoint],
        ) -> Result<(), BackendError> {
            Ok(())
        }
    }

    #[test]
    fn first_snapshot_and_reconnect_keep_the_actual_controller_fault_visible() {
        let mut backend = crate::backend::DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot.clone(), "LIVE-TEST");
        snapshot.host_control.last_error = Some("fan3 RPM below safety threshold".into());
        let (worker, events) = event_worker();
        let mut pending = BTreeMap::new();
        for _ in 0..2 {
            let mut has_snapshot = false;
            app.mark_host_status_unknown();
            app.apply_failed("old transport failure");
            events
                .send(WorkerEvent::Refreshed(Ok(snapshot.clone())))
                .unwrap();
            process_test_events(&mut app, &worker, &mut has_snapshot, &mut pending);
            assert!(has_snapshot);
            assert!(app.host_status_trusted);
            assert_eq!(app.status.kind, crate::app::StatusKind::Error);
            assert_eq!(
                app.status.text,
                "Fan control: fan3 RPM below safety threshold"
            );
        }
    }

    #[test]
    fn display_command_waits_for_ack_and_uses_authoritative_snapshot() {
        let store = TempProfiles::new();
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "TEST");
        let (worker, received, events) = recording_worker();
        let device_id = app.snapshot.kraken_display.device_id.clone().unwrap();
        let mut pending = BTreeMap::new();
        let mut host_operations = HostOperationState::default();
        send_app_command(
            &mut app,
            &worker,
            Some(&store.store),
            &mut pending,
            &mut host_operations,
            AppCommand::SetKrakenDisplay {
                device_id: device_id.clone(),
                mode: KrakenDisplayMode::CpuGpu,
            },
        );
        assert!(
            matches!(received.try_recv().unwrap(), WorkerCommand::SetKrakenDisplay {
            device_id: id, mode: KrakenDisplayMode::CpuGpu,
        } if id == device_id)
        );
        assert!(app.display_operation_busy);
        assert_eq!(
            app.snapshot.kraken_display.mode,
            KrakenDisplayMode::BuiltinLiquid
        );
        events.send(WorkerEvent::DisplaySet(Ok(()))).unwrap();
        drain_events(
            &mut app,
            &worker,
            &store.store,
            &mut pending,
            &mut host_operations,
        );
        assert!(!app.display_operation_busy);
        assert_eq!(
            app.snapshot.kraken_display.mode,
            KrakenDisplayMode::BuiltinLiquid
        );
        assert!(matches!(
            received.try_recv().unwrap(),
            WorkerCommand::Refresh
        ));
        let mut fresh = app.snapshot.clone();
        fresh.kraken_display.mode = KrakenDisplayMode::CpuGpu;
        events.send(WorkerEvent::Refreshed(Ok(fresh))).unwrap();
        drain_events(
            &mut app,
            &worker,
            &store.store,
            &mut pending,
            &mut host_operations,
        );
        assert_eq!(app.snapshot.kraken_display.mode, KrakenDisplayMode::CpuGpu);
    }

    #[test]
    fn dropping_worker_cancels_and_joins_active_backend_work() {
        let cancellation = BackendCancellation::default();
        let worker = BackendWorker::spawn(CancelAwareBackend {
            cancellation: cancellation.clone(),
        })
        .unwrap();
        assert!(worker.request_refresh());
        let started = Instant::now();

        drop(worker);

        assert!(cancellation.is_cancelled());
        assert!(started.elapsed() < Duration::from_secs(1));
    }

    #[test]
    fn demo_worker_failure_names_the_simulator_not_a_service() {
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        let mut pending = BTreeMap::new();
        let mut host_operations = HostOperationState::default();
        process_worker_disconnect(&mut app, &mut pending, &mut host_operations);
        assert_eq!(app.status.text, "Simulator stopped unexpectedly");
    }

    #[test]
    fn worker_disconnect_keeps_running_state_untrusted_until_reconnect() {
        let mut app = app_with_applied_dirty_firmware_curve();
        let points = firmware_editor_points(&app);
        let dirty_count = app.dirty_curve_count();
        app.host_start_succeeded(app.host_editor.policy());
        let mut pending_applies = BTreeMap::new();
        let mut host_operations = HostOperationState::default();

        let refresh =
            process_worker_disconnect(&mut app, &mut pending_applies, &mut host_operations);

        assert!(!refresh.in_flight);
        assert!(!refresh.has_snapshot);
        assert_eq!(app.host_control_state, HostControlState::Running);
        assert!(!app.host_status_trusted);
        assert_eq!(firmware_editor_points(&app), points);
        assert_eq!(app.dirty_curve_count(), dirty_count);
        assert!(
            app.status
                .text
                .contains("Monitoring service connection stopped unexpectedly")
        );
    }

    #[test]
    fn worker_disconnect_marks_pending_host_start_unknown_from_available() {
        let mut app = app_with_applied_dirty_firmware_curve();
        app.handle_key_event(KeyEvent::new(KeyCode::BackTab, KeyModifiers::NONE));
        app.handle_key_event(KeyEvent::new(KeyCode::Char('s'), KeyModifiers::NONE));
        let firmware_points = firmware_editor_points(&app);
        let host_policy = app.host_editor.policy();
        let dirty_count = app.dirty_curve_count();
        assert_eq!(app.host_control_state, HostControlState::Available);
        assert!(app.host_policy_is_dirty());
        let mut pending_applies = BTreeMap::new();
        let mut host_operations = HostOperationState {
            pending: Some(PendingHostOperation::Start),
            queued_stop: false,
        };

        process_worker_disconnect(&mut app, &mut pending_applies, &mut host_operations);

        assert_eq!(app.host_control_state, HostControlState::Available);
        assert!(!app.host_status_trusted);
        assert!(host_operations.pending.is_none());
        assert_eq!(app.host_editor.policy(), host_policy);
        assert!(app.host_policy_is_dirty());
        assert_eq!(firmware_editor_points(&app), firmware_points);
        assert_eq!(app.dirty_curve_count(), dirty_count);
    }

    #[test]
    fn worker_disconnect_marks_pending_host_stop_unknown_from_available() {
        let mut app = app_with_applied_dirty_firmware_curve();
        let points = firmware_editor_points(&app);
        let dirty_count = app.dirty_curve_count();
        assert_eq!(app.host_control_state, HostControlState::Available);
        let mut pending_applies = BTreeMap::new();
        let mut host_operations = HostOperationState {
            pending: Some(PendingHostOperation::Stop),
            queued_stop: false,
        };

        process_worker_disconnect(&mut app, &mut pending_applies, &mut host_operations);

        assert_eq!(app.host_control_state, HostControlState::Available);
        assert!(!app.host_status_trusted);
        assert!(host_operations.pending.is_none());
        assert_eq!(firmware_editor_points(&app), points);
        assert_eq!(app.dirty_curve_count(), dirty_count);
    }

    #[test]
    fn worker_disconnect_invalidates_applied_firmware_claims_and_clears_pending_writes() {
        let mut app = app_with_applied_dirty_firmware_curve();
        let key = app.active_curve_key().unwrap();
        let points = firmware_editor_points(&app);
        let dirty_count = app.dirty_curve_count();
        let mut pending_applies = BTreeMap::from([(key, PendingApply::default())]);
        let mut host_operations = HostOperationState::default();

        let refresh =
            process_worker_disconnect(&mut app, &mut pending_applies, &mut host_operations);

        assert!(!refresh.in_flight);
        assert!(!refresh.has_snapshot);
        assert!(pending_applies.is_empty());
        assert!(
            app.snapshot
                .devices
                .iter()
                .flat_map(|device| &device.cooling_channels)
                .all(|channel| channel.curve_state == crate::model::CurveState::Unverified)
        );
        assert_eq!(app.host_control_state, HostControlState::Available);
        assert!(!app.host_status_trusted);
        assert_eq!(firmware_editor_points(&app), points);
        assert_eq!(app.dirty_curve_count(), dirty_count);
    }

    #[test]
    fn duplicate_apply_for_same_channel_is_coalesced() {
        let mut backend = crate::backend::DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot, backend.name());
        let worker = BackendWorker::spawn(backend).unwrap();
        let saved = TempProfiles::new();
        let mut pending = BTreeMap::new();
        let first = app
            .handle_key_event(KeyEvent::new(KeyCode::Enter, KeyModifiers::NONE))
            .unwrap();
        let mut host_operations = HostOperationState::default();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut host_operations,
            first,
        );
        app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::NONE));
        let second = app
            .handle_key_event(KeyEvent::new(KeyCode::Enter, KeyModifiers::NONE))
            .unwrap();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut host_operations,
            second,
        );

        assert_eq!(pending.len(), 1);
        let queued = pending.values().next().unwrap().queued.as_ref().unwrap();
        assert_eq!(
            queued.points[0].duty,
            app.selected_curve_point().unwrap().duty
        );
        assert!(app.status.text.contains("queued the latest curve"));
    }

    #[test]
    fn refresh_session_loss_immediately_invalidates_all_claims_and_preserves_edits() {
        let mut backend = crate::backend::DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot, "TEST");
        app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::NONE));
        let points = app
            .snapshot
            .devices
            .iter()
            .flat_map(|device| &device.cooling_channels)
            .map(|channel| channel.points.clone())
            .collect::<Vec<_>>();
        let dirty_count = app.dirty_curve_count();
        let (worker, events) = event_worker();
        events
            .send(WorkerEvent::Refreshed(Err(BackendError::new(
                "service reconnect failed",
            )
            .invalidating_live_curve_claims())))
            .unwrap();
        let mut has_snapshot = true;
        let mut pending = BTreeMap::new();

        process_test_events(&mut app, &worker, &mut has_snapshot, &mut pending);

        assert!(!has_snapshot);
        assert!(
            app.snapshot
                .devices
                .iter()
                .flat_map(|device| &device.cooling_channels)
                .all(|channel| channel.curve_state == crate::model::CurveState::Unverified)
        );
        assert_eq!(
            app.snapshot
                .devices
                .iter()
                .flat_map(|device| &device.cooling_channels)
                .map(|channel| channel.points.clone())
                .collect::<Vec<_>>(),
            points
        );
        assert_eq!(app.dirty_curve_count(), dirty_count);
        assert!(app.status.text.contains("service reconnect failed"));

        events
            .send(WorkerEvent::Refreshed(Ok(app.snapshot.clone())))
            .unwrap();
        process_test_events(&mut app, &worker, &mut has_snapshot, &mut pending);
        assert!(has_snapshot);
        assert_eq!(app.status.kind, app::StatusKind::Success);
        assert!(app.status.text.contains("Monitoring service connected"));
    }

    #[test]
    fn apply_connection_loss_invalidates_every_claim_and_marks_target_unknown() {
        let mut backend = crate::backend::DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot, "TEST");
        let key = app.active_curve_key().unwrap();
        let request = fake_prepared(&key, app.active_channel().unwrap().points.clone());
        let (worker, events) = event_worker();
        events
            .send(WorkerEvent::Applied {
                key: key.clone(),
                request,
                result: Err(BackendError::with_kind(
                    backend::BackendErrorKind::UnknownOutcome,
                    "connection lost after apply",
                )
                .invalidating_live_curve_claims()),
            })
            .unwrap();
        let mut has_snapshot = true;
        let mut pending = BTreeMap::new();
        pending.insert(key.clone(), PendingApply::default());

        process_test_events(&mut app, &worker, &mut has_snapshot, &mut pending);

        assert!(!has_snapshot);
        assert!(
            app.snapshot
                .devices
                .iter()
                .flat_map(|device| &device.cooling_channels)
                .all(|channel| channel.curve_state == crate::model::CurveState::Unverified)
        );
        assert!(!app.active_curve_is_verified());
        assert!(app.status.text.contains("connection lost after apply"));
    }

    #[test]
    fn failed_apply_drops_queued_write_until_user_confirms_again() {
        let mut backend = crate::backend::DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot, "TEST");
        let key = app.active_curve_key().unwrap();
        let points = app.active_channel().unwrap().points.clone();
        let (command_sender, command_receiver) = mpsc::channel();
        let (event_sender, event_receiver) = mpsc::channel();
        let worker = BackendWorker {
            commands: command_sender,
            events: event_receiver,
            cancellation: None,
            worker_thread: None,
        };
        event_sender
            .send(WorkerEvent::Applied {
                key: key.clone(),
                request: fake_prepared(&key, points.clone()),
                result: Err(BackendError::with_kind(
                    backend::BackendErrorKind::UnknownOutcome,
                    "write outcome is uncertain",
                )),
            })
            .unwrap();
        let mut pending = BTreeMap::from([(
            key,
            PendingApply {
                queued: Some(CurveRequest {
                    points,
                    profile: None,
                }),
            },
        )]);
        let mut has_snapshot = true;

        process_test_events(&mut app, &worker, &mut has_snapshot, &mut pending);

        assert!(pending.is_empty());
        assert!(matches!(
            command_receiver.try_recv(),
            Err(TryRecvError::Empty)
        ));
        assert!(app.status.text.contains("write outcome is uncertain"));
    }

    #[test]
    fn profile_save_failure_blocks_firmware_start_and_batch_dispatch() {
        for operation in 0..3 {
            let saved = TempProfiles::new();
            let blocker = saved.directory.join("not-a-directory");
            std::fs::write(&blocker, b"preserve").unwrap();
            let broken = ProfileStore::at(blocker.join("profiles.json"));
            let mut backend = crate::backend::DemoBackend::new();
            let mut app = App::new(backend.refresh().unwrap(), "DEMO");
            let policy = app.host_editor.policy();
            let command = match operation {
                0 => AppCommand::ApplyCurve {
                    key: app.active_curve_key().unwrap(),
                    points: app.active_channel().unwrap().points.clone(),
                    profile: None,
                },
                1 => AppCommand::StartHostControl { policy },
                _ => AppCommand::UpdateHostControl {
                    channel_policies: policy.channels,
                },
            };
            let (worker, received, _events) = recording_worker();
            let mut pending = BTreeMap::new();
            let mut state = HostOperationState::default();
            send_app_command(
                &mut app,
                &worker,
                Some(&broken),
                &mut pending,
                &mut state,
                command,
            );
            assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
            assert!(pending.is_empty() && state.pending.is_none());
            assert!(!app.host_operation_busy);
            assert!(app.status.text.contains("Apply not sent"));
            assert_eq!(std::fs::read(&blocker).unwrap(), b"preserve");
        }
    }

    #[test]
    fn firmware_curve_is_saved_before_dispatch_and_named_only_after_success() {
        let saved = TempProfiles::new();
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.handle_key_event(KeyCode::Up.into());
        let command = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        let (worker, received, events) = recording_worker();
        let mut pending = BTreeMap::new();
        let mut state = HostOperationState::default();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            command,
        );
        let WorkerCommand::ApplyCurve { key, request } = received.try_recv().unwrap() else {
            panic!("expected firmware apply")
        };
        let library = saved.store.load().unwrap();
        assert_eq!(library.profiles.len(), 1);
        assert!(library.bindings.is_empty());
        assert_eq!(
            library.profiles[0].curve,
            ProfileCurve::Firmware(request.points.clone())
        );
        assert_eq!(app.active_profile_name(), "Custom");
        events
            .send(WorkerEvent::Applied {
                key,
                request,
                result: Ok(()),
            })
            .unwrap();
        drain_events(&mut app, &worker, &saved.store, &mut pending, &mut state);
        assert_eq!(app.active_profile_name(), "my-custom-profile-1");
        assert_eq!(saved.store.load().unwrap().bindings.len(), 1);
        let again = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            again,
        );
        assert!(matches!(
            received.try_recv(),
            Ok(WorkerCommand::ApplyCurve { .. })
        ));
        assert_eq!(saved.store.load().unwrap().profiles.len(), 1);
    }

    #[test]
    fn queued_curves_save_exact_dispatched_values_and_preserve_newer_draft() {
        let saved = TempProfiles::new();
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        let (worker, received, events) = recording_worker();
        let mut pending = BTreeMap::new();
        let mut state = HostOperationState::default();
        app.handle_key_event(KeyCode::Up.into());
        let first = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            first,
        );
        let WorkerCommand::ApplyCurve { key, request } = received.try_recv().unwrap() else {
            panic!("expected first apply")
        };
        let first_points = request.points.clone();
        app.handle_key_event(KeyCode::Up.into());
        let second_points = app.active_channel().unwrap().points.clone();
        let second = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            second,
        );
        assert_eq!(saved.store.load().unwrap().profiles.len(), 1);
        events
            .send(WorkerEvent::Applied {
                key,
                request,
                result: Ok(()),
            })
            .unwrap();
        drain_events(&mut app, &worker, &saved.store, &mut pending, &mut state);
        assert_eq!(app.active_channel().unwrap().points, second_points);
        assert_eq!(app.active_profile_name(), "Custom");
        let WorkerCommand::ApplyCurve { key, request } = received.try_recv().unwrap() else {
            panic!("expected queued apply")
        };
        assert_eq!(request.points, second_points);
        let library = saved.store.load().unwrap();
        assert_eq!(library.profiles.len(), 2);
        assert_eq!(
            library.profiles[0].curve,
            ProfileCurve::Firmware(first_points)
        );
        assert_eq!(
            library.profiles[1].curve,
            ProfileCurve::Firmware(second_points)
        );
        // A rename while the write is in flight is resolved by ID, never stale name.
        app.set_profile_library(saved.store.rename(2, "Quiet AIO").unwrap());
        events
            .send(WorkerEvent::Applied {
                key,
                request,
                result: Ok(()),
            })
            .unwrap();
        drain_events(&mut app, &worker, &saved.store, &mut pending, &mut state);
        assert_eq!(app.active_profile_name(), "Quiet AIO");
        assert!(pending.is_empty());
    }

    #[test]
    fn host_profiles_survive_stop_and_empty_snapshot_restart_without_starting() {
        let saved = TempProfiles::new();
        let mut backend = crate::backend::DemoBackend::new();
        let available = backend.refresh().unwrap();
        let mut app = App::new(available.clone(), "DEMO");
        for channel in &mut app.host_editor.channels {
            channel.curve.source = model::HostTemperatureSource::Gpu;
        }
        let policy = app.host_editor.policy();
        let (worker, received, events) = recording_worker();
        let mut pending = BTreeMap::new();
        let mut state = HostOperationState::default();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            AppCommand::StartHostControl {
                policy: policy.clone(),
            },
        );
        let WorkerCommand::StartHostControl {
            policy: accepted,
            bindings,
        } = received.try_recv().unwrap()
        else {
            panic!("expected start")
        };
        assert_eq!(accepted, policy);
        assert_eq!(saved.store.load().unwrap().profiles.len(), 1);
        assert!(saved.store.load().unwrap().bindings.is_empty());
        assert_eq!(bindings.len(), policy.channels.len());
        assert!(bindings.iter().all(|b| b.profile == bindings[0].profile));
        events
            .send(WorkerEvent::HostStarted {
                policy: accepted,
                bindings,
                result: Ok(()),
            })
            .unwrap();
        drain_events(&mut app, &worker, &saved.store, &mut pending, &mut state);
        assert_eq!(
            app.active_host_profile_name().as_deref(),
            Some("my-custom-profile-1")
        );
        app.host_stop_succeeded();
        let library = saved.store.load().unwrap();
        assert_eq!(library.bindings.len(), policy.channels.len());
        let mut reopened =
            App::with_profile_library(empty_app().snapshot, "LIVE", library, AppConfig::default());
        reopened.update_telemetry(available);
        assert_eq!(reopened.host_control_state, HostControlState::Available);
        assert_eq!(reopened.host_editor.policy(), policy);
        assert_eq!(
            reopened.active_host_profile_name().as_deref(),
            Some("my-custom-profile-1")
        );
        assert!(reopened.snapshot.host_control.active_policy.is_none());
        assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
    }

    #[test]
    fn confirmed_unchanged_running_curve_saves_without_hardware_update() {
        let saved = TempProfiles::new();
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.host_start_succeeded(app.host_editor.policy());
        app.handle_key_event(KeyCode::BackTab.into());
        assert!(app.handle_key_event(KeyCode::Enter.into()).is_none());
        let command = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        assert!(matches!(command, AppCommand::RememberHostProfiles { .. }));
        let (worker, received, _events) = recording_worker();
        let mut pending = BTreeMap::new();
        let mut state = HostOperationState::default();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            command,
        );
        assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
        assert_eq!(saved.store.load().unwrap().bindings.len(), 1);
        assert_eq!(
            app.active_host_profile_name().as_deref(),
            Some("my-custom-profile-1")
        );
        assert!(!app.host_operation_busy);
    }

    #[test]
    fn rejected_or_uncertain_firmware_apply_keeps_saved_copy_without_binding() {
        for kind in [
            BackendErrorKind::Unavailable,
            BackendErrorKind::UnknownOutcome,
        ] {
            let saved = TempProfiles::new();
            let mut backend = crate::backend::DemoBackend::new();
            let mut app = App::new(backend.refresh().unwrap(), "DEMO");
            app.handle_key_event(KeyCode::Up.into());
            let command = app.handle_key_event(KeyCode::Enter.into()).unwrap();
            let (worker, received, events) = recording_worker();
            let mut pending = BTreeMap::new();
            let mut state = HostOperationState::default();
            send_app_command(
                &mut app,
                &worker,
                Some(&saved.store),
                &mut pending,
                &mut state,
                command,
            );
            let WorkerCommand::ApplyCurve { key, request } = received.try_recv().unwrap() else {
                panic!("expected firmware apply")
            };
            events
                .send(WorkerEvent::Applied {
                    key,
                    request,
                    result: Err(BackendError::with_kind(kind, "not confirmed")),
                })
                .unwrap();
            drain_events(&mut app, &worker, &saved.store, &mut pending, &mut state);
            assert_eq!(app.active_profile_name(), "Custom");
            let persisted = saved.store.load().unwrap();
            assert_eq!(persisted.profiles.len(), 1);
            assert!(persisted.bindings.is_empty());
            assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
        }
    }

    #[test]
    fn both_picker_rename_flows_persist_names_without_any_hardware_command() {
        for host in [false, true] {
            let saved = TempProfiles::new();
            let mut backend = crate::backend::DemoBackend::new();
            let snapshot = backend.refresh().unwrap();
            let initial = App::new(snapshot.clone(), "DEMO");
            let key = initial.active_curve_key().unwrap();
            let host_policy = initial.host_editor.policy().channels[0].clone();
            let (_, bindings) = saved
                .store
                .prepare(&[
                    ProfileIntent {
                        target: ProfileTarget::Firmware {
                            device_id: key.0,
                            channel_id: key.1,
                        },
                        curve: ProfileCurve::Firmware(
                            initial.active_channel().unwrap().points.clone(),
                        ),
                        preferred: None,
                    },
                    ProfileIntent {
                        target: ProfileTarget::Host {
                            channel_id: host_policy.channel_id,
                        },
                        curve: ProfileCurve::Host(host_policy.curve),
                        preferred: None,
                    },
                ])
                .unwrap();
            let library = saved.store.bind_success(&bindings).unwrap();
            let mut app =
                App::with_profile_library(snapshot, "DEMO", library, AppConfig::default());
            if host {
                app.handle_key_event(KeyCode::BackTab.into());
            }
            let before_fw = firmware_editor_points(&app);
            let before_host = app.host_editor.policy();
            app.handle_key_event(KeyCode::Char('p').into());
            app.handle_key_event(KeyCode::Char('r').into());
            assert_eq!(app.modal, Some(app::Modal::RenameProfile));
            for _ in 0..32 {
                app.handle_key_event(KeyCode::Backspace.into());
            }
            for character in "My favourite".chars() {
                app.handle_key_event(KeyCode::Char(character).into());
            }
            let command = app.handle_key_event(KeyCode::Enter.into()).unwrap();
            let (worker, received, _events) = recording_worker();
            let mut pending = BTreeMap::new();
            let mut state = HostOperationState::default();
            send_app_command(
                &mut app,
                &worker,
                Some(&saved.store),
                &mut pending,
                &mut state,
                command,
            );
            assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
            let reference = bindings[usize::from(host)].profile;
            assert_eq!(
                saved.store.load().unwrap().name(reference),
                Some("My favourite")
            );
            assert_eq!(saved.store.load().unwrap().bindings, bindings);
            assert_eq!(firmware_editor_points(&app), before_fw);
            assert_eq!(app.host_editor.policy(), before_host);
            assert_eq!(app.host_control_state, HostControlState::Available);
            if host {
                assert_eq!(
                    app.active_host_profile_name().as_deref(),
                    Some("My favourite")
                );
            } else {
                assert_eq!(app.active_profile_name(), "My favourite");
            }
        }
    }

    #[test]
    fn binding_failure_reports_applied_but_does_not_lose_prepared_curve() {
        let saved = TempProfiles::new();
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.handle_key_event(KeyCode::Up.into());
        let command = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        let (worker, received, events) = recording_worker();
        let mut pending = BTreeMap::new();
        let mut state = HostOperationState::default();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            command,
        );
        let WorkerCommand::ApplyCurve { key, request } = received.try_recv().unwrap() else {
            panic!("expected firmware apply")
        };
        let backup = saved.directory.join("saved-copy.json");
        let path = saved.directory.join("profiles.json");
        std::fs::rename(&path, &backup).unwrap();
        std::fs::create_dir(&path).unwrap(); // deterministic fake filesystem failure
        events
            .send(WorkerEvent::Applied {
                key,
                request,
                result: Ok(()),
            })
            .unwrap();
        drain_events(&mut app, &worker, &saved.store, &mut pending, &mut state);
        assert!(app.active_curve_is_verified());
        assert_eq!(app.active_profile_name(), "my-custom-profile-1");
        assert!(
            app.status
                .text
                .contains("Applied, but profile selection could not be saved")
        );
        let preserved = ProfileStore::at(backup).load().unwrap();
        assert_eq!(preserved.profiles.len(), 1);
        assert!(preserved.bindings.is_empty());
    }

    #[test]
    fn repeated_enter_is_ignored_but_repeated_edit_keys_are_allowed() {
        let repeated_enter =
            KeyEvent::new_with_kind(KeyCode::Enter, KeyModifiers::NONE, KeyEventKind::Repeat);
        let repeated_up =
            KeyEvent::new_with_kind(KeyCode::Up, KeyModifiers::NONE, KeyEventKind::Repeat);

        assert!(!should_handle_key_event(&repeated_enter));
        assert!(should_handle_key_event(&repeated_up));
    }

    #[test]
    fn explicit_stop_queues_once_behind_update_and_first_quit_waits_for_it() {
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.host_start_succeeded(app.host_editor.policy());
        app.handle_key_event(KeyCode::BackTab.into());
        app.handle_key_event(KeyCode::Char('s').into());
        assert!(app.handle_key_event(KeyCode::Enter.into()).is_none());
        let update = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        let (commands, received) = mpsc::channel();
        let (events, incoming) = mpsc::channel();
        let worker = BackendWorker {
            commands,
            events: incoming,
            cancellation: None,
            worker_thread: None,
        };
        let mut pending = BTreeMap::new();
        let mut state = HostOperationState::default();
        let saved = TempProfiles::new();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            update,
        );
        let WorkerCommand::UpdateHostControl {
            channel_policies: accepted,
            bindings,
        } = received.try_recv().unwrap()
        else {
            panic!("expected update")
        };
        assert!(app.host_operation_busy);
        assert!(app.handle_key_event(KeyCode::Char('s').into()).is_none());
        assert_eq!(app.handle_key_event(KeyCode::Enter.into()), None);
        app.handle_key_event(KeyCode::Char('o').into());
        assert_eq!(
            app.handle_key_event(KeyCode::Char('t').into()),
            Some(AppCommand::StopHostControl)
        );
        send_app_command(
            &mut app,
            &worker,
            None,
            &mut pending,
            &mut state,
            AppCommand::StopHostControl,
        );
        send_app_command(
            &mut app,
            &worker,
            None,
            &mut pending,
            &mut state,
            AppCommand::StopHostControl,
        );
        assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
        app.exit = true;
        let mut exiting = false;
        begin_graceful_exit(&mut app, 0, &state, &mut exiting);
        assert!(exiting && !app.exit);
        events
            .send(WorkerEvent::HostUpdated {
                channel_policies: accepted,
                bindings,
                result: Ok(()),
            })
            .unwrap();
        let mut refresh = RefreshState {
            in_flight: false,
            has_snapshot: true,
            last_refresh: Instant::now(),
        };
        process_backend_events(
            &mut app,
            &worker,
            &mut refresh,
            &mut pending,
            &mut state,
            None,
        );
        assert!(matches!(
            received.try_recv().unwrap(),
            WorkerCommand::StopHostControl
        ));
        assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
        assert!(app.host_operation_busy);
        events.send(WorkerEvent::HostStopped(Ok(()))).unwrap();
        process_backend_events(
            &mut app,
            &worker,
            &mut refresh,
            &mut pending,
            &mut state,
            None,
        );
        assert!(!app.host_operation_busy);
        assert!(state.pending.is_none() && !state.queued_stop);
    }

    #[test]
    fn batch_update_failure_keeps_drafts_and_does_not_retry_or_claim_applied() {
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.host_start_succeeded(app.host_editor.policy());
        app.handle_key_event(KeyCode::BackTab.into());
        app.handle_key_event(KeyCode::Char('s').into());
        app.handle_key_event(KeyCode::Enter.into());
        app.handle_key_event(KeyCode::End.into());
        let command = app.handle_key_event(KeyCode::Enter.into()).unwrap();
        let accepted = match &command {
            AppCommand::UpdateHostControl { channel_policies } => channel_policies.clone(),
            _ => panic!("expected batch"),
        };
        let drafts = app.host_editor.policy();
        let applied = app.snapshot.host_control.active_policy.clone();
        let (commands, received) = mpsc::channel();
        let (events, incoming) = mpsc::channel();
        let worker = BackendWorker {
            commands,
            events: incoming,
            cancellation: None,
            worker_thread: None,
        };
        let mut pending = BTreeMap::new();
        let mut state = HostOperationState::default();
        let saved = TempProfiles::new();
        send_app_command(
            &mut app,
            &worker,
            Some(&saved.store),
            &mut pending,
            &mut state,
            command,
        );
        let WorkerCommand::UpdateHostControl {
            channel_policies,
            bindings,
        } = received.try_recv().unwrap()
        else {
            panic!("expected update")
        };
        assert_eq!(channel_policies, accepted);
        assert!(saved.store.load().unwrap().bindings.is_empty());
        events
            .send(WorkerEvent::HostUpdated {
                channel_policies: accepted,
                bindings,
                result: Err(BackendError::with_kind(
                    BackendErrorKind::UnknownOutcome,
                    "reply lost",
                )),
            })
            .unwrap();
        let mut refresh = RefreshState {
            in_flight: false,
            has_snapshot: true,
            last_refresh: Instant::now(),
        };
        process_backend_events(
            &mut app,
            &worker,
            &mut refresh,
            &mut pending,
            &mut state,
            None,
        );
        assert!(matches!(received.try_recv(), Err(TryRecvError::Empty)));
        assert_eq!(app.host_editor.policy(), drafts);
        assert_eq!(app.snapshot.host_control.active_policy, applied);
        assert!(!app.host_status_trusted);
        assert!(app.status.text.contains("reply lost"));
    }

    #[test]
    fn host_worker_start_and_stop_are_single_sequential_operations() {
        let mut backend = crate::backend::DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let policy = App::new(snapshot, backend.name()).host_editor.policy();
        let worker = BackendWorker::spawn(backend).unwrap();

        assert!(worker.start_host_control(policy.clone(), Vec::new()));
        let WorkerEvent::HostStarted {
            policy: returned,
            result,
            ..
        } = worker.events.recv_timeout(Duration::from_secs(1)).unwrap()
        else {
            panic!("expected host start event");
        };
        assert_eq!(returned, policy);
        result.unwrap();
        assert!(worker.stop_host_control());
        let WorkerEvent::HostStopped(result) =
            worker.events.recv_timeout(Duration::from_secs(1)).unwrap()
        else {
            panic!("expected host stop event");
        };
        result.unwrap();
    }

    #[test]
    fn quit_while_running_detaches_without_sending_stop() {
        let mut backend = crate::backend::DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), backend.name());
        app.host_start_succeeded(app.host_editor.policy());
        app.exit = true;
        let (command_sender, command_receiver) = mpsc::channel();
        let (_event_sender, event_receiver) = mpsc::channel();
        let _worker = BackendWorker {
            commands: command_sender,
            events: event_receiver,
            cancellation: None,
            worker_thread: None,
        };
        let mut exit_when_idle = false;
        begin_graceful_exit(
            &mut app,
            0,
            &HostOperationState::default(),
            &mut exit_when_idle,
        );
        assert!(app.exit);
        assert!(!exit_when_idle);
        assert!(matches!(
            command_receiver.try_recv(),
            Err(TryRecvError::Empty)
        ));
    }

    #[test]
    fn quit_during_start_waits_then_detaches_without_stop() {
        let mut app = empty_app();
        app.exit = true;
        let mut exit_when_idle = false;
        begin_graceful_exit(
            &mut app,
            0,
            &HostOperationState {
                pending: Some(PendingHostOperation::Start),
                queued_stop: false,
            },
            &mut exit_when_idle,
        );
        assert!(!app.exit);
        assert!(exit_when_idle);
        app.host_start_succeeded(app.host_editor.policy());
        assert_eq!(app.host_control_state, HostControlState::Running);
    }

    #[test]
    fn first_quit_waits_for_pending_writes_and_second_forces_exit() {
        let mut app = empty_app();
        let mut exit_when_idle = false;
        app.exit = true;

        let (_worker, _events) = event_worker();
        let pending_host = HostOperationState::default();
        begin_graceful_exit(&mut app, 1, &pending_host, &mut exit_when_idle);

        assert!(!app.exit);
        assert!(exit_when_idle);
        assert!(app.status.text.contains("pending hardware operation"));

        app.exit = true;
        begin_graceful_exit(&mut app, 1, &pending_host, &mut exit_when_idle);
        assert!(app.exit);
    }

    #[test]
    fn quit_waits_for_explicit_activation_or_future_restart_setting() {
        for activation in [false, true] {
            let mut app = empty_app();
            app.activation_busy = activation;
            app.auto_resume_busy = !activation;
            app.exit = true;
            let mut exit_when_idle = false;
            begin_graceful_exit(
                &mut app,
                0,
                &HostOperationState::default(),
                &mut exit_when_idle,
            );
            assert!(!app.exit);
            assert!(exit_when_idle);
            assert!(app.status.text.contains("pending hardware operation"));
        }
    }

    #[test]
    fn quit_is_not_deferred_without_pending_writes() {
        let mut app = empty_app();
        let mut exit_when_idle = false;
        app.exit = true;

        let (_worker, _events) = event_worker();
        let pending_host = HostOperationState::default();
        begin_graceful_exit(&mut app, 0, &pending_host, &mut exit_when_idle);

        assert!(app.exit);
        assert!(!exit_when_idle);
    }
}
