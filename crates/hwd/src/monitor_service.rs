//! Service request orchestration: preflight, durable selection, then independent writes.
use std::collections::HashSet;

use nzxt_cam_core::{
    ChannelId, CurvePoint, DeviceId, HostControlPolicy, HostControlState, KrakenDisplayMode,
    MonitoringActualState, MonitoringError,
};
use nzxt_cam_protocol::{
    ErrorMessage, MAX_MONITORING_FIRMWARE_CURVES, MonitoringActivationOutcome,
    MonitoringActivationStatus as Status, MonitoringActivationTarget as Target,
    MonitoringDisplaySelection, MonitoringFirmwareCurve,
};

use crate::{
    hardware::{HardwareError, HardwareErrorKind},
    monitor_intent::{
        FileMonitorIntentStore, MonitorTarget, SavedMonitorIntent, SavedTarget, WriteState,
    },
};

pub(crate) trait MonitoringStore {
    fn load(&self) -> Result<Option<SavedMonitorIntent>, HardwareError>;
    fn persist(&mut self, intent: &SavedMonitorIntent) -> Result<(), HardwareError>;
    fn mark_in_flight(&mut self, target: &MonitorTarget) -> Result<(), HardwareError>;
    fn mark_ready(&mut self, target: &MonitorTarget) -> Result<(), HardwareError>;
}
impl MonitoringStore for FileMonitorIntentStore {
    fn load(&self) -> Result<Option<SavedMonitorIntent>, HardwareError> {
        self.load()
    }
    fn persist(&mut self, intent: &SavedMonitorIntent) -> Result<(), HardwareError> {
        self.persist(intent)
    }
    fn mark_in_flight(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
        self.mark_in_flight(target)
    }
    fn mark_ready(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
        self.mark_ready(target)
    }
}

pub(crate) trait MonitoringIo {
    fn preflight_curve(
        &mut self,
        id: &DeviceId,
        channel: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<bool, HardwareError>;
    fn write_curve(
        &mut self,
        id: &DeviceId,
        channel: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError>;
    fn preflight_display(&mut self, id: &DeviceId) -> Result<(), HardwareError>;
    fn select_display(&mut self, id: DeviceId, mode: KrakenDisplayMode);
    fn clear_display(&mut self);
    fn host_state(&self) -> HostControlState;
    fn start_host(&mut self, policy: &HostControlPolicy) -> Result<(), HardwareError>;
}

pub(crate) fn same_key(a: &MonitorTarget, b: &MonitorTarget) -> bool {
    match (a, b) {
        (
            MonitorTarget::AioCurve {
                device_id: a,
                channel_id: ac,
                ..
            },
            MonitorTarget::AioCurve {
                device_id: b,
                channel_id: bc,
                ..
            },
        ) => a == b && ac == bc,
        (
            MonitorTarget::Display { device_id: a, .. },
            MonitorTarget::Display { device_id: b, .. },
        ) => a == b,
        _ => false,
    }
}

/// Updates only an already opted-in selection, preserving all blocked evidence.
/// Never call an editor write unless this persistence succeeds.
pub(crate) fn update_selected<S: MonitoringStore>(
    store: &mut S,
    target: MonitorTarget,
) -> Result<(), HardwareError> {
    let mut intent = store.load()?.ok_or_else(|| {
        HardwareError::with_kind(HardwareErrorKind::RestoreRequired, "monitor intent absent")
    })?;
    if !intent.opted_in {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::RestoreRequired,
            "monitor intent not opted in",
        ));
    }
    if intent
        .targets
        .iter()
        .any(|saved| same_key(&saved.target, &target) && saved.state == WriteState::InFlightUnknown)
    {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::RestoreRequired,
            "target write outcome requires review",
        ));
    }
    intent.targets.retain(|saved| {
        if matches!(target, MonitorTarget::Display { .. }) {
            // There is one selected LCD face, not one selection per cooler.
            // Preserve any uncertain prior write as evidence for review.
            !matches!(saved.target, MonitorTarget::Display { .. })
                || saved.state == WriteState::InFlightUnknown
        } else {
            !same_key(&saved.target, &target)
        }
    });
    intent.targets.push(SavedTarget {
        target,
        state: WriteState::Ready,
    });
    persist_selection(store, &intent)
}

/// Future-startup authorization only; does not touch host policy or hardware.
pub(crate) fn set_auto_resume<S: MonitoringStore>(
    store: &mut S,
    enabled: bool,
) -> Result<(), HardwareError> {
    let mut intent = store.load()?.ok_or_else(|| {
        HardwareError::with_kind(HardwareErrorKind::RestoreRequired, "monitor intent absent")
    })?;
    if !intent.opted_in {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::RestoreRequired,
            "monitor intent not opted in",
        ));
    }
    intent.auto_resume = enabled;
    if enabled {
        // A post-rename sync failure can install `true` despite returning an
        // error. Use the same fail-closed disarm as activation/selection.
        persist_selection(store, &intent)
    } else {
        store.persist(&intent)
    }
}

/// A post-rename error can leave a newly authorized selection on disk.
/// Do not permit that ambiguous request to silently run on a later boot.
fn persist_selection<S: MonitoringStore>(
    store: &mut S,
    intent: &SavedMonitorIntent,
) -> Result<(), HardwareError> {
    if let Err(error) = store.persist(intent) {
        if store.load().ok().flatten().as_ref() == Some(intent) {
            let mut disarmed = intent.clone();
            disarmed.auto_resume = false;
            if store.persist(&disarmed).is_err() {
                // The second save may have left an orphan temp file (which blocks
                // replay). Otherwise block each newly selected Ready target.
                for saved in &intent.targets {
                    if saved.state == WriteState::Ready {
                        let _ = store.mark_in_flight(&saved.target);
                    }
                }
            }
        }
        return Err(error);
    }
    Ok(())
}

/// Once marked, ALL errors block future replay, including a definite failed command.
pub(crate) fn write_curve<S: MonitoringStore>(
    store: &mut S,
    target: &MonitorTarget,
    write: impl FnOnce(&DeviceId, &ChannelId, &[CurvePoint]) -> Result<(), HardwareError>,
) -> Result<(), HardwareError> {
    let MonitorTarget::AioCurve {
        device_id,
        channel_id,
        points,
    } = target
    else {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::InvalidData,
            "not a firmware curve",
        ));
    };
    store.mark_in_flight(target)?;
    write(device_id, channel_id, points)?;
    match store.mark_ready(target) {
        Ok(()) => Ok(()),
        Err(error) => {
            // A post-rename sync error may leave Ready on disk despite the
            // failed acknowledgement. Re-block if possible; an orphan temp
            // otherwise prevents any replay on restart.
            let _ = store.mark_in_flight(target);
            Err(error)
        }
    }
}

pub(crate) fn record_write(
    actual: &mut Vec<(
        MonitorTarget,
        MonitoringActualState,
        Option<MonitoringError>,
    )>,
    target: &MonitorTarget,
    result: &Result<(), HardwareError>,
) {
    actual.retain(|(saved, _, _)| !same_key(saved, target));
    actual.push((
        target.clone(),
        if result.is_ok() {
            MonitoringActualState::Applied
        } else {
            MonitoringActualState::ReviewRequired
        },
        result
            .as_ref()
            .err()
            .map(|e| MonitoringError::new(e.message())),
    ));
}

fn outcome(target: Target, status: Status, error: Option<&str>) -> MonitoringActivationOutcome {
    MonitoringActivationOutcome {
        target,
        status,
        error: error.map(ErrorMessage::new),
    }
}
fn failure(target: Target, error: &HardwareError) -> MonitoringActivationOutcome {
    outcome(target, Status::Failed, Some(error.message()))
}

pub(crate) fn activate<S: MonitoringStore, I: MonitoringIo>(
    store: &mut S,
    io: &mut I,
    actual: &mut Vec<(
        MonitorTarget,
        MonitoringActualState,
        Option<MonitoringError>,
    )>,
    curves: &[MonitoringFirmwareCurve],
    host_policy: Option<&HostControlPolicy>,
    display: Option<&MonitoringDisplaySelection>,
) -> Result<Vec<MonitoringActivationOutcome>, HardwareError> {
    if curves.len() > MAX_MONITORING_FIRMWARE_CURVES {
        return Err(HardwareError::with_kind(
            HardwareErrorKind::InvalidData,
            "too many monitoring curves",
        ));
    }
    let previous = store.load()?;
    let mut selected = previous.as_ref().map_or_else(Vec::new, |old| {
        old.targets
            .iter()
            .filter(|t| t.state == WriteState::InFlightUnknown)
            .cloned()
            .collect()
    });
    let mut outcomes = Vec::new();
    let mut eligible = Vec::new();
    let mut keys = HashSet::new();
    for curve in curves {
        let label = Target::FirmwareCurve {
            device_id: curve.device_id.clone(),
            channel_id: curve.channel_id.clone(),
        };
        let target = MonitorTarget::AioCurve {
            device_id: curve.device_id.clone(),
            channel_id: curve.channel_id.clone(),
            points: curve.points.clone(),
        };
        if !keys.insert((curve.device_id.clone(), curve.channel_id.clone())) {
            outcomes.push(outcome(
                label,
                Status::Skipped,
                Some("duplicate firmware target"),
            ));
            continue;
        }
        if previous.as_ref().is_some_and(|old| {
            old.targets.iter().any(|saved| {
                same_key(&saved.target, &target) && saved.state == WriteState::InFlightUnknown
            })
        }) {
            outcomes.push(outcome(
                label,
                Status::Unknown,
                Some("target requires review before another write"),
            ));
            continue;
        }
        match io.preflight_curve(&curve.device_id, &curve.channel_id, &curve.points) {
            Ok(true) => {
                selected.push(SavedTarget {
                    target: target.clone(),
                    state: WriteState::Ready,
                });
                eligible.push((outcomes.len(), target));
                outcomes.push(outcome(label, Status::Pending, None));
            }
            Ok(false) => outcomes.push(outcome(label, Status::Skipped, Some("cooler offline"))),
            Err(error) => outcomes.push(failure(label, &error)),
        }
    }
    let mut display_ready = false;
    if let Some(display) = display {
        let label = Target::Display {
            device_id: display.device_id.clone(),
        };
        let target = MonitorTarget::Display {
            device_id: display.device_id.clone(),
            mode: display.mode,
        };
        if previous.as_ref().is_some_and(|old| {
            old.targets.iter().any(|saved| {
                same_key(&saved.target, &target) && saved.state == WriteState::InFlightUnknown
            })
        }) {
            outcomes.push(outcome(
                label,
                Status::Unknown,
                Some("display target requires review"),
            ));
        } else {
            match io.preflight_display(&display.device_id) {
                Ok(()) => {
                    selected.push(SavedTarget {
                        target,
                        state: WriteState::Ready,
                    });
                    display_ready = true;
                    outcomes.push(outcome(label, Status::Pending, None));
                }
                Err(error) => outcomes.push(failure(label, &error)),
            }
        }
    }
    // Selection is durable before ANY write or worker dispatch. A failed save
    // cannot authorize even one of the otherwise eligible components.
    let intent = SavedMonitorIntent {
        version: 1,
        opted_in: true,
        auto_resume: true,
        targets: selected,
    };
    persist_selection(store, &intent)?;
    for (index, target) in eligible {
        let result = write_curve(store, &target, |id, channel, points| {
            io.write_curve(id, channel, points)
        });
        record_write(actual, &target, &result);
        outcomes[index] = match result {
            Ok(()) => outcome(outcomes[index].target.clone(), Status::Applied, None),
            Err(error) => outcome(
                outcomes[index].target.clone(),
                Status::Unknown,
                Some(error.message()),
            ),
        };
    }
    if !display_ready
        && previous.as_ref().is_some_and(|old| {
            old.targets
                .iter()
                .any(|saved| matches!(saved.target, MonitorTarget::Display { .. }))
        })
    {
        io.clear_display();
    }
    if display_ready {
        let selection = display.expect("display_ready requires selection");
        actual.retain(|(saved, _, _)| !matches!(saved, MonitorTarget::Display { .. }));
        io.select_display(selection.device_id.clone(), selection.mode);
    }
    if let Some(policy) = host_policy {
        let label = Target::HostControl {};
        outcomes.push(match io.host_state() {
            HostControlState::Running => {
                outcome(label, Status::Skipped, Some("host control already running"))
            }
            HostControlState::Available => match io.start_host(policy) {
                Ok(()) => outcome(label, Status::Applied, None),
                Err(error) => failure(label, &error),
            },
            _ => outcome(label, Status::Skipped, Some("host control unavailable")),
        });
    }
    Ok(outcomes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{cell::RefCell, rc::Rc};

    #[derive(Clone, Default)]
    struct Store {
        saved: Option<SavedMonitorIntent>,
        events: Rc<RefCell<Vec<String>>>,
        fail_save: bool,
        fail_after_save: bool,
        fail_ready: bool,
    }
    impl MonitoringStore for Store {
        fn load(&self) -> Result<Option<SavedMonitorIntent>, HardwareError> {
            Ok(self.saved.clone())
        }
        fn persist(&mut self, intent: &SavedMonitorIntent) -> Result<(), HardwareError> {
            self.events.borrow_mut().push("persist".into());
            if self.fail_save {
                return Err(HardwareError::new("disk failed"));
            }
            intent.validate()?;
            if let Some(old) = &self.saved {
                for blocked in old
                    .targets
                    .iter()
                    .filter(|t| t.state == WriteState::InFlightUnknown)
                {
                    if !intent.targets.contains(blocked) {
                        return Err(HardwareError::new("blocked target changed"));
                    }
                }
            }
            self.saved = Some(intent.clone());
            if std::mem::take(&mut self.fail_after_save) {
                return Err(HardwareError::new("post-rename sync failed"));
            }
            Ok(())
        }
        fn mark_in_flight(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
            self.events.borrow_mut().push("in_flight".into());
            let saved = self
                .saved
                .as_mut()
                .unwrap()
                .targets
                .iter_mut()
                .find(|t| &t.target == target && t.state == WriteState::Ready)
                .ok_or_else(|| HardwareError::new("target blocked"))?;
            saved.state = WriteState::InFlightUnknown;
            Ok(())
        }
        fn mark_ready(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
            self.events.borrow_mut().push("ready".into());
            if self.fail_ready {
                return Err(HardwareError::new("sync failed"));
            }
            let saved = self
                .saved
                .as_mut()
                .unwrap()
                .targets
                .iter_mut()
                .find(|t| &t.target == target && t.state == WriteState::InFlightUnknown)
                .unwrap();
            saved.state = WriteState::Ready;
            Ok(())
        }
    }
    struct Io {
        events: Rc<RefCell<Vec<String>>>,
        curve: Result<bool, HardwareError>,
        display: Result<(), HardwareError>,
        write: Result<(), HardwareError>,
        host: Result<(), HardwareError>,
        state: HostControlState,
    }
    impl Io {
        fn new(events: Rc<RefCell<Vec<String>>>) -> Self {
            Self {
                events,
                curve: Ok(true),
                display: Ok(()),
                write: Ok(()),
                host: Ok(()),
                state: HostControlState::Available,
            }
        }
    }
    impl MonitoringIo for Io {
        fn preflight_curve(
            &mut self,
            _: &DeviceId,
            _: &ChannelId,
            _: &[CurvePoint],
        ) -> Result<bool, HardwareError> {
            self.curve.clone()
        }
        fn write_curve(
            &mut self,
            _: &DeviceId,
            _: &ChannelId,
            _: &[CurvePoint],
        ) -> Result<(), HardwareError> {
            self.events.borrow_mut().push("write".into());
            self.write.clone()
        }
        fn preflight_display(&mut self, _: &DeviceId) -> Result<(), HardwareError> {
            self.display.clone()
        }
        fn select_display(&mut self, _: DeviceId, _: KrakenDisplayMode) {
            self.events.borrow_mut().push("select".into());
        }
        fn clear_display(&mut self) {
            self.events.borrow_mut().push("clear_display".into());
        }
        fn host_state(&self) -> HostControlState {
            self.state
        }
        fn start_host(&mut self, _: &HostControlPolicy) -> Result<(), HardwareError> {
            self.events.borrow_mut().push("host_start".into());
            self.host.clone()
        }
    }
    fn curve(id: &str) -> MonitoringFirmwareCurve {
        MonitoringFirmwareCurve {
            device_id: DeviceId::new(id),
            channel_id: ChannelId::new("pump"),
            points: (20..60)
                .map(|temperature| CurvePoint {
                    temperature,
                    duty: if temperature == 59 { 100 } else { 45 },
                })
                .collect(),
        }
    }
    fn target(curve: &MonitoringFirmwareCurve) -> MonitorTarget {
        MonitorTarget::AioCurve {
            device_id: curve.device_id.clone(),
            channel_id: curve.channel_id.clone(),
            points: curve.points.clone(),
        }
    }
    fn display(id: &str) -> MonitoringDisplaySelection {
        MonitoringDisplaySelection {
            device_id: DeviceId::new(id),
            mode: KrakenDisplayMode::Cpu,
        }
    }
    fn setup() -> (Store, Io) {
        let store = Store::default();
        let io = Io::new(store.events.clone());
        (store, io)
    }
    #[test]
    fn persist_precedes_all_writes_and_display_selection_and_failed_store_does_nothing() {
        let (mut store, mut io) = setup();
        let mut actual = Vec::new();
        let selected = display("lcd");
        let outcomes = activate(
            &mut store,
            &mut io,
            &mut actual,
            &[curve("aio")],
            None,
            Some(&selected),
        )
        .unwrap();
        assert_eq!(
            outcomes.iter().map(|o| o.status).collect::<Vec<_>>(),
            vec![Status::Applied, Status::Pending]
        );
        assert_eq!(
            &*store.events.borrow(),
            &["persist", "in_flight", "write", "ready", "select"]
        );
        assert_eq!(store.saved.as_ref().unwrap().targets.len(), 2);
        let (mut store, mut io) = setup();
        store.fail_save = true;
        assert!(
            activate(
                &mut store,
                &mut io,
                &mut Vec::new(),
                &[curve("aio")],
                None,
                Some(&selected)
            )
            .is_err()
        );
        assert_eq!(&*store.events.borrow(), &["persist"]);
    }
    #[test]
    fn failed_post_rename_selection_disarms_future_replay_without_starting_any_worker() {
        let (mut store, mut io) = setup();
        store.fail_after_save = true;
        assert!(
            activate(
                &mut store,
                &mut io,
                &mut Vec::new(),
                &[curve("aio")],
                None,
                Some(&display("lcd"))
            )
            .is_err()
        );
        let saved = store.saved.unwrap();
        assert!(saved.opted_in);
        assert!(!saved.auto_resume);
        assert_eq!(saved.targets.len(), 2);
        assert_eq!(&*io.events.borrow(), &["persist", "persist"]);
    }

    #[test]
    fn partial_activation_blocks_unknown_and_never_retries_even_after_restart() {
        let (mut store, mut io) = setup();
        io.write = Err(HardwareError::with_kind(
            HardwareErrorKind::Timeout,
            "timed out",
        ));
        io.display = Err(HardwareError::with_kind(
            HardwareErrorKind::Unsupported,
            "wrong LCD",
        ));
        io.host = Err(HardwareError::new("host guard failed"));
        let curves = [curve("aio"), curve("aio")];
        let policy = HostControlPolicy {
            channels: Vec::new(),
        };
        let outcomes = activate(
            &mut store,
            &mut io,
            &mut Vec::new(),
            &curves,
            Some(&policy),
            Some(&display("wrong")),
        )
        .unwrap();
        assert_eq!(
            outcomes.iter().map(|o| o.status).collect::<Vec<_>>(),
            vec![
                Status::Unknown,
                Status::Skipped,
                Status::Failed,
                Status::Failed
            ]
        );
        assert_eq!(
            store.saved.as_ref().unwrap().targets[0].state,
            WriteState::InFlightUnknown
        );
        io.write = Ok(());
        let before = store
            .events
            .borrow()
            .iter()
            .filter(|e| *e == "write")
            .count();
        let outcomes = activate(
            &mut store,
            &mut io,
            &mut Vec::new(),
            &[curve("aio")],
            None,
            None,
        )
        .unwrap();
        assert_eq!(outcomes[0].status, Status::Unknown);
        assert_eq!(
            store
                .events
                .borrow()
                .iter()
                .filter(|e| *e == "write")
                .count(),
            before
        );
        assert!(update_selected(&mut store, target(&curve("aio"))).is_err());
        let mut changed = curve("aio");
        changed.points[0].duty = 46;
        assert!(update_selected(&mut store, target(&changed)).is_err());
    }
    #[test]
    fn editor_post_rename_failure_does_not_enable_unacknowledged_replay() {
        let (mut store, _) = setup();
        store.saved = Some(SavedMonitorIntent {
            version: 1,
            opted_in: true,
            auto_resume: true,
            targets: Vec::new(),
        });
        store.fail_after_save = true;
        assert!(
            update_selected(
                &mut store,
                MonitorTarget::Display {
                    device_id: DeviceId::new("lcd"),
                    mode: KrakenDisplayMode::Cpu
                }
            )
            .is_err()
        );
        assert!(!store.saved.unwrap().auto_resume);
        assert_eq!(&*store.events.borrow(), &["persist", "persist"]);
    }

    #[test]
    fn editor_updates_exact_target_then_blocks_on_failed_ready_and_future_resume_is_only_a_store_change()
     {
        let (mut store, mut io) = setup();
        activate(
            &mut store,
            &mut io,
            &mut Vec::new(),
            &[curve("aio")],
            None,
            None,
        )
        .unwrap();
        let mut changed = curve("aio");
        changed.points[0].duty = 46;
        update_selected(&mut store, target(&changed)).unwrap();
        store.fail_ready = true;
        assert!(write_curve(&mut store, &target(&changed), |_, _, _| Ok(())).is_err());
        assert_eq!(
            store.saved.as_ref().unwrap().targets[0].state,
            WriteState::InFlightUnknown
        );
        let event_count = store.events.borrow().len();
        set_auto_resume(&mut store, false).unwrap();
        assert!(!store.saved.as_ref().unwrap().auto_resume);
        assert_eq!(
            store.saved.as_ref().unwrap().targets[0].state,
            WriteState::InFlightUnknown
        );
        assert_eq!(store.events.borrow().len(), event_count + 1);
        let (mut absent, _) = setup();
        assert!(set_auto_resume(&mut absent, false).is_err());
        assert!(update_selected(&mut absent, target(&curve("aio"))).is_err());
    }
    #[test]
    fn switching_lcd_replaces_the_prior_ready_device_selection() {
        let (mut store, _) = setup();
        store.saved = Some(SavedMonitorIntent {
            version: 1,
            opted_in: true,
            auto_resume: true,
            targets: vec![SavedTarget {
                target: MonitorTarget::Display {
                    device_id: DeviceId::new("lcd-old"),
                    mode: KrakenDisplayMode::Cpu,
                },
                state: WriteState::Ready,
            }],
        });
        update_selected(
            &mut store,
            MonitorTarget::Display {
                device_id: DeviceId::new("lcd-new"),
                mode: KrakenDisplayMode::Gpu,
            },
        )
        .unwrap();
        assert_eq!(store.saved.unwrap().targets.len(), 1);
    }

    #[test]
    fn failed_post_rename_auto_resume_enable_disarms_before_next_boot() {
        let (mut store, _) = setup();
        store.saved = Some(SavedMonitorIntent {
            version: 1,
            opted_in: true,
            auto_resume: false,
            targets: vec![SavedTarget {
                target: target(&curve("aio")),
                state: WriteState::Ready,
            }],
        });
        store.fail_after_save = true;
        assert!(set_auto_resume(&mut store, true).is_err());
        assert!(!store.saved.as_ref().unwrap().auto_resume);
        assert_eq!(&*store.events.borrow(), &["persist", "persist"]);
    }

    #[test]
    fn all_32_firmware_curves_and_one_display_fit_the_durable_selection() {
        let (mut store, mut io) = setup();
        let curves: Vec<_> = (0..MAX_MONITORING_FIRMWARE_CURVES)
            .map(|n| curve(&format!("aio-{n}")))
            .collect();
        let outcomes = activate(
            &mut store,
            &mut io,
            &mut Vec::new(),
            &curves,
            None,
            Some(&display("lcd")),
        )
        .unwrap();
        assert_eq!(
            store.saved.as_ref().unwrap().targets.len(),
            MAX_MONITORING_FIRMWARE_CURVES + 1
        );
        assert_eq!(outcomes.len(), MAX_MONITORING_FIRMWARE_CURVES + 1);
        assert_eq!(
            outcomes[MAX_MONITORING_FIRMWARE_CURVES].status,
            Status::Pending
        );
    }

    #[test]
    fn replacing_a_display_with_no_selection_stops_old_uploads_without_resetting_hardware() {
        let (mut store, mut io) = setup();
        activate(
            &mut store,
            &mut io,
            &mut Vec::new(),
            &[],
            None,
            Some(&display("lcd")),
        )
        .unwrap();
        io.events.borrow_mut().clear();
        activate(&mut store, &mut io, &mut Vec::new(), &[], None, None).unwrap();
        assert!(store.saved.unwrap().targets.is_empty());
        assert_eq!(&*io.events.borrow(), &["persist", "clear_display"]);
    }

    #[test]
    fn offline_and_wrong_device_do_not_get_selected_or_written_and_running_host_is_not_restarted() {
        let (mut store, mut io) = setup();
        io.curve = Ok(false);
        io.display = Err(HardwareError::with_kind(
            HardwareErrorKind::Unsupported,
            "duplicate serial",
        ));
        io.state = HostControlState::Running;
        let policy = HostControlPolicy {
            channels: Vec::new(),
        };
        let outcomes = activate(
            &mut store,
            &mut io,
            &mut Vec::new(),
            &[curve("offline")],
            Some(&policy),
            Some(&display("duplicate")),
        )
        .unwrap();
        assert_eq!(
            outcomes.iter().map(|o| o.status).collect::<Vec<_>>(),
            vec![Status::Skipped, Status::Failed, Status::Skipped]
        );
        assert!(store.saved.unwrap().targets.is_empty());
        assert_eq!(&*io.events.borrow(), &["persist"]);
    }
}
