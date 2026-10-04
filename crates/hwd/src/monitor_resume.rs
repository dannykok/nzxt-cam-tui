//! Startup-only replay. All decisions are made after the intent is loaded and
//! guarded host recovery has completed; LCD uploads remain worker-owned.
use std::{
    thread,
    time::{Duration, Instant},
};

use nzxt_cam_core::{ChannelId, CurvePoint, DeviceId, KrakenDisplayMode};

use crate::{
    hardware::{HardwareCancellation, HardwareError},
    liquidctl::LiquidctlHardware,
    monitor_intent::{FileMonitorIntentStore, MonitorTarget, SavedMonitorIntent, WriteState},
};

pub(crate) trait IntentStore {
    fn load(&self) -> Result<Option<SavedMonitorIntent>, HardwareError>;
    fn mark_in_flight(&mut self, target: &MonitorTarget) -> Result<(), HardwareError>;
    fn mark_ready(&mut self, target: &MonitorTarget) -> Result<(), HardwareError>;
}

impl IntentStore for FileMonitorIntentStore {
    fn load(&self) -> Result<Option<SavedMonitorIntent>, HardwareError> {
        self.load()
    }
    fn mark_in_flight(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
        self.mark_in_flight(target)
    }
    fn mark_ready(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
        self.mark_ready(target)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum Preflight {
    Ready,
    Offline,
}

pub(crate) trait ResumeHardware {
    fn preflight(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<Preflight, HardwareError>;
    fn write_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError>;
}

impl ResumeHardware for LiquidctlHardware {
    fn preflight(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<Preflight, HardwareError> {
        match self.preflight_resume_curve(device_id, channel_id, points)? {
            true => Ok(Preflight::Ready),
            false => Ok(Preflight::Offline),
        }
    }
    fn write_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), HardwareError> {
        use crate::hardware::HardwareOperations;
        self.apply_firmware_curve(device_id, channel_id, points)
    }
}

pub(crate) struct BootSelection {
    pub(crate) restored_display: Option<(DeviceId, KrakenDisplayMode)>,
    /// Definite successful writes during this boot, not a device readback.
    pub(crate) applied_curves: Vec<MonitorTarget>,
}

/// A bad store cannot authorize host resume, but must not prevent guarded
/// recovery. No USB command or LCD worker selection precedes host recovery.
pub(crate) fn boot<S: IntentStore, H: ResumeHardware>(
    store: &mut S,
    hardware: &mut H,
    mut initialize_host: impl FnMut(bool) -> Result<(), HardwareError>,
    mut pause: impl FnMut(),
    mut check: impl FnMut() -> Result<(), HardwareError>,
) -> Result<BootSelection, HardwareError> {
    let loaded = store.load();
    let resume_host = loaded
        .as_ref()
        .ok()
        .and_then(Option::as_ref)
        .is_none_or(|intent| intent.opted_in && intent.auto_resume);
    initialize_host(resume_host && loaded.is_ok())?;
    check()?;
    let intent = loaded?;
    let Some(intent) = intent else {
        return Ok(BootSelection {
            restored_display: None,
            applied_curves: Vec::new(),
        });
    };
    let mut selection = BootSelection {
        restored_display: None,
        applied_curves: Vec::new(),
    };
    if !intent.opted_in || !intent.auto_resume {
        return Ok(selection);
    }
    for saved in intent.targets {
        check()?;
        if saved.state != WriteState::Ready {
            continue;
        }
        match &saved.target {
            MonitorTarget::Display { device_id, mode } => {
                // The worker will validate this exact identity before each upload.
                selection.restored_display = Some((device_id.clone(), *mode));
            }
            MonitorTarget::AioCurve {
                device_id,
                channel_id,
                points,
            } => {
                let mut ready = false;
                for attempt in 0..3 {
                    check()?;
                    match hardware.preflight(device_id, channel_id, points) {
                        Ok(Preflight::Ready) => {
                            ready = true;
                            break;
                        }
                        Ok(Preflight::Offline) if attempt < 2 => {
                            check()?;
                            pause();
                        }
                        Ok(Preflight::Offline) => break,
                        Err(error) => {
                            eprintln!("saved cooler preflight skipped: {error}");
                            break;
                        }
                    }
                }
                if !ready {
                    continue;
                }
                check()?;
                store.mark_in_flight(&saved.target)?;
                check()?;
                match hardware.write_curve(device_id, channel_id, points) {
                    Ok(()) => {
                        // A definite success must be persisted even if shutdown
                        // arrived during the write. Never retry an uncertain write.
                        if let Err(error) = store.mark_ready(&saved.target) {
                            let _ = store.mark_in_flight(&saved.target);
                            return Err(error);
                        }
                        selection.applied_curves.push(saved.target.clone());
                    }
                    Err(error) => {
                        // Including a definite error: never retry without operator review.
                        eprintln!("saved cooler write blocked pending review: {error}");
                    }
                }
            }
        }
    }
    check()?;
    Ok(selection)
}

pub(crate) fn startup_pause(cancellation: &HardwareCancellation) {
    let deadline = Instant::now() + Duration::from_millis(250);
    while !cancellation.is_cancelled() {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        thread::sleep(remaining.min(Duration::from_millis(10)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hardware::{HardwareError, HardwareErrorKind};
    use crate::monitor_intent::SavedTarget;
    use std::{cell::RefCell, rc::Rc};

    fn curve(id: &str) -> MonitorTarget {
        MonitorTarget::AioCurve {
            device_id: DeviceId::new(id),
            channel_id: ChannelId::new("pump"),
            points: (20..60)
                .map(|temperature| CurvePoint {
                    temperature,
                    duty: if temperature == 59 { 100 } else { 50 },
                })
                .collect(),
        }
    }
    fn saved(targets: Vec<(MonitorTarget, WriteState)>, auto_resume: bool) -> SavedMonitorIntent {
        SavedMonitorIntent {
            version: 1,
            opted_in: true,
            auto_resume,
            targets: targets
                .into_iter()
                .map(|(target, state)| SavedTarget { target, state })
                .collect(),
        }
    }
    #[derive(Default)]
    struct FakeStore {
        intent: Option<SavedMonitorIntent>,
        fail_load: bool,
        fail_mark: bool,
        cancel_after_mark: Option<HardwareCancellation>,
        log: Rc<RefCell<Vec<String>>>,
    }
    impl IntentStore for FakeStore {
        fn load(&self) -> Result<Option<SavedMonitorIntent>, HardwareError> {
            self.log.borrow_mut().push("load".into());
            if self.fail_load {
                return Err(HardwareError::new("bad store"));
            }
            Ok(self.intent.clone())
        }
        fn mark_in_flight(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
            self.log.borrow_mut().push("block".into());
            if self.fail_mark {
                return Err(HardwareError::new("store unavailable"));
            }
            let saved = self
                .intent
                .as_mut()
                .unwrap()
                .targets
                .iter_mut()
                .find(|s| &s.target == target)
                .unwrap();
            assert_eq!(saved.state, WriteState::Ready);
            saved.state = WriteState::InFlightUnknown;
            if let Some(cancellation) = &self.cancel_after_mark {
                cancellation.cancel();
            }
            Ok(())
        }
        fn mark_ready(&mut self, target: &MonitorTarget) -> Result<(), HardwareError> {
            self.log.borrow_mut().push("ready".into());
            let saved = self
                .intent
                .as_mut()
                .unwrap()
                .targets
                .iter_mut()
                .find(|s| &s.target == target)
                .unwrap();
            assert_eq!(saved.state, WriteState::InFlightUnknown);
            saved.state = WriteState::Ready;
            Ok(())
        }
    }
    #[derive(Default)]
    struct FakeHardware {
        online: Vec<String>,
        fail: Vec<String>,
        cancel_preflight: Option<HardwareCancellation>,
        cancel_write: Option<HardwareCancellation>,
        log: Rc<RefCell<Vec<String>>>,
    }
    impl ResumeHardware for FakeHardware {
        fn preflight(
            &mut self,
            id: &DeviceId,
            _: &ChannelId,
            points: &[CurvePoint],
        ) -> Result<Preflight, HardwareError> {
            self.log.borrow_mut().push(format!("preflight:{id}"));
            assert_eq!(points.len(), 40);
            if let Some(cancellation) = &self.cancel_preflight {
                cancellation.cancel();
            }
            Ok(if self.online.contains(&id.0) {
                Preflight::Ready
            } else {
                Preflight::Offline
            })
        }
        fn write_curve(
            &mut self,
            id: &DeviceId,
            _: &ChannelId,
            _: &[CurvePoint],
        ) -> Result<(), HardwareError> {
            self.log.borrow_mut().push(format!("write:{id}"));
            if let Some(cancellation) = &self.cancel_write {
                cancellation.cancel();
            }
            if self.fail.contains(&id.0) {
                Err(HardwareError::with_kind(
                    HardwareErrorKind::UnknownOutcome,
                    "unknown",
                ))
            } else {
                Ok(())
            }
        }
    }
    fn run(
        store: &mut FakeStore,
        hardware: &mut FakeHardware,
    ) -> Result<(BootSelection, Vec<bool>), HardwareError> {
        let mut host = Vec::new();
        let log = hardware.log.clone();
        let selection = boot(
            store,
            hardware,
            |resume| {
                store_log_host(&log, resume);
                host.push(resume);
                Ok(())
            },
            || {},
            || Ok(()),
        )?;
        Ok((selection, host))
    }
    fn store_log_host(log: &Rc<RefCell<Vec<String>>>, resume: bool) {
        log.borrow_mut().push(format!("host:{resume}"));
    }

    #[test]
    fn absence_preserves_host_resume_without_selecting_display_but_opt_out_and_bad_store_fail_closed()
     {
        let mut store = FakeStore::default();
        let mut hw = FakeHardware::default();
        let (selection, host) = run(&mut store, &mut hw).unwrap();
        assert_eq!(host, [true]);
        assert!(selection.restored_display.is_none());
        assert!(hw.log.borrow().iter().all(|e| !e.starts_with("write:")));
        store.intent = Some(saved(vec![(curve("aio"), WriteState::Ready)], false));
        let (selection, host) = run(&mut store, &mut hw).unwrap();
        assert_eq!(host, [false]);
        assert!(selection.restored_display.is_none());
        assert!(hw.log.borrow().iter().all(|e| !e.starts_with("write:")));
        store.fail_load = true;
        assert!(run(&mut store, &mut hw).is_err());
        assert_eq!(hw.log.borrow().last().unwrap(), "host:false");
    }

    #[test]
    fn no_opt_in_does_not_restore_display_or_write_aio_even_with_auto_resume() {
        let mut intent = saved(
            vec![
                (
                    MonitorTarget::Display {
                        device_id: DeviceId::new("lcd-a"),
                        mode: KrakenDisplayMode::Cpu,
                    },
                    WriteState::Ready,
                ),
                (curve("aio"), WriteState::Ready),
            ],
            true,
        );
        intent.opted_in = false;
        let mut store = FakeStore {
            intent: Some(intent),
            ..Default::default()
        };
        let mut hw = FakeHardware {
            online: vec!["aio".into()],
            ..Default::default()
        };
        let (selection, host) = run(&mut store, &mut hw).unwrap();
        assert_eq!(host, [false]);
        assert!(selection.restored_display.is_none());
        assert!(
            hw.log
                .borrow()
                .iter()
                .all(|event| !event.starts_with("write:") && !event.starts_with("preflight:"))
        );
    }

    #[test]
    fn exact_ready_replay_partial_failure_and_blocked_restart() {
        let log = Rc::new(RefCell::new(Vec::new()));
        let mut store = FakeStore {
            intent: Some(saved(
                vec![
                    (curve("good"), WriteState::Ready),
                    (curve("failed"), WriteState::Ready),
                    (curve("blocked"), WriteState::InFlightUnknown),
                    (curve("replacement"), WriteState::Ready),
                ],
                true,
            )),
            log: log.clone(),
            ..Default::default()
        };
        let mut hw = FakeHardware {
            online: vec![
                "good".into(),
                "failed".into(),
                "blocked".into(),
                "other".into(),
            ],
            fail: vec!["failed".into()],
            log: log.clone(),
            ..Default::default()
        };
        let (selection, _) = run(&mut store, &mut hw).unwrap();
        assert_eq!(selection.applied_curves, vec![curve("good")]);
        let events = log.borrow().clone();
        assert_eq!(
            &events[..5],
            ["load", "host:true", "preflight:good", "block", "write:good"]
        );
        assert!(events.contains(&"ready".into()));
        assert!(!events.contains(&"write:blocked".into()));
        assert!(!events.contains(&"write:replacement".into()));
        assert!(!events.contains(&"write:other".into()));
        assert_eq!(
            store.intent.as_ref().unwrap().targets[1].state,
            WriteState::InFlightUnknown
        );
        log.borrow_mut().clear();
        hw.online.clear(); // after a restart no blocked target may be retried
        hw.online.push("failed".into());
        run(&mut store, &mut hw).unwrap();
        assert!(!log.borrow().iter().any(|s| s == "write:failed"));
    }

    #[test]
    fn display_restores_exact_selection_without_curve_transition_or_replacement() {
        let mut store = FakeStore {
            intent: Some(saved(
                vec![(
                    MonitorTarget::Display {
                        device_id: DeviceId::new("lcd-a"),
                        mode: KrakenDisplayMode::Gpu,
                    },
                    WriteState::Ready,
                )],
                true,
            )),
            ..Default::default()
        };
        let mut hw = FakeHardware {
            online: vec!["lcd-b".into()],
            ..Default::default()
        };
        let (selection, _) = run(&mut store, &mut hw).unwrap();
        assert_eq!(
            selection.restored_display,
            Some((DeviceId::new("lcd-a"), KrakenDisplayMode::Gpu))
        );
        assert!(store.log.borrow().iter().all(|s| s != "block"));
        assert!(hw.log.borrow().iter().all(|s| !s.starts_with("write:")));
    }

    #[test]
    fn failed_host_recovery_prevents_any_usb_preflight_or_write() {
        let mut store = FakeStore {
            intent: Some(saved(vec![(curve("aio"), WriteState::Ready)], true)),
            ..Default::default()
        };
        let mut hw = FakeHardware {
            online: vec!["aio".into()],
            ..Default::default()
        };
        assert!(
            boot(
                &mut store,
                &mut hw,
                |_| Err(HardwareError::new("recovery failed")),
                || {},
                || Ok(())
            )
            .is_err()
        );
        assert!(hw.log.borrow().is_empty());
        assert_eq!(store.intent.unwrap().targets[0].state, WriteState::Ready);
    }

    fn check(cancellation: &HardwareCancellation) -> Result<(), HardwareError> {
        if cancellation.is_cancelled() {
            Err(HardwareError::service_shutdown())
        } else {
            Ok(())
        }
    }

    #[test]
    fn shutdown_after_guarded_host_recovery_prevents_usb_and_display_selection() {
        let cancellation = HardwareCancellation::default();
        let mut store = FakeStore {
            intent: Some(saved(vec![(curve("aio"), WriteState::Ready)], true)),
            ..Default::default()
        };
        let mut hw = FakeHardware {
            online: vec!["aio".into()],
            ..Default::default()
        };
        let mut recovery_called = false;
        let error = boot(
            &mut store,
            &mut hw,
            |_| {
                recovery_called = true;
                cancellation.cancel();
                Ok(())
            },
            || panic!("pause after shutdown"),
            || check(&cancellation),
        )
        .err()
        .unwrap();
        assert!(error.is_service_shutdown());
        assert!(recovery_called);
        assert!(hw.log.borrow().is_empty());
        assert_eq!(store.intent.unwrap().targets[0].state, WriteState::Ready);
    }

    #[test]
    fn shutdown_during_preflight_prevents_pause_persistence_and_write() {
        for online in [false, true] {
            let cancellation = HardwareCancellation::default();
            let mut store = FakeStore {
                intent: Some(saved(vec![(curve("aio"), WriteState::Ready)], true)),
                ..Default::default()
            };
            let mut hw = FakeHardware {
                online: if online {
                    vec!["aio".into()]
                } else {
                    Vec::new()
                },
                cancel_preflight: Some(cancellation.clone()),
                ..Default::default()
            };
            let error = boot(
                &mut store,
                &mut hw,
                |_| Ok(()),
                || panic!("pause after cancelled preflight"),
                || check(&cancellation),
            )
            .err()
            .unwrap();
            assert!(error.is_service_shutdown());
            assert_eq!(*hw.log.borrow(), ["preflight:aio"]);
            assert_eq!(*store.log.borrow(), ["load"]);
            assert_eq!(store.intent.unwrap().targets[0].state, WriteState::Ready);
        }
    }

    #[test]
    fn shutdown_during_pause_prevents_retry() {
        let cancellation = HardwareCancellation::default();
        let mut store = FakeStore {
            intent: Some(saved(vec![(curve("offline"), WriteState::Ready)], true)),
            ..Default::default()
        };
        let mut hw = FakeHardware::default();
        let mut pauses = 0;
        let error = boot(
            &mut store,
            &mut hw,
            |_| Ok(()),
            || {
                pauses += 1;
                cancellation.cancel();
            },
            || check(&cancellation),
        )
        .err()
        .unwrap();
        assert!(error.is_service_shutdown());
        assert_eq!(pauses, 1);
        assert_eq!(*hw.log.borrow(), ["preflight:offline"]);
        assert_eq!(*store.log.borrow(), ["load"]);
    }

    #[test]
    fn shutdown_after_mark_in_flight_preserves_evidence_without_writing() {
        let cancellation = HardwareCancellation::default();
        let mut store = FakeStore {
            intent: Some(saved(vec![(curve("aio"), WriteState::Ready)], true)),
            cancel_after_mark: Some(cancellation.clone()),
            ..Default::default()
        };
        let mut hw = FakeHardware {
            online: vec!["aio".into()],
            ..Default::default()
        };
        let error = boot(
            &mut store,
            &mut hw,
            |_| Ok(()),
            || {},
            || check(&cancellation),
        )
        .err()
        .unwrap();
        assert!(error.is_service_shutdown());
        assert_eq!(*hw.log.borrow(), ["preflight:aio"]);
        assert_eq!(
            store.intent.unwrap().targets[0].state,
            WriteState::InFlightUnknown
        );
    }

    #[test]
    fn shutdown_mid_write_preserves_definite_success_or_unknown_and_stops_later_targets() {
        for failed in [false, true] {
            let cancellation = HardwareCancellation::default();
            let mut store = FakeStore {
                intent: Some(saved(
                    vec![
                        (
                            MonitorTarget::Display {
                                device_id: DeviceId::new("lcd"),
                                mode: KrakenDisplayMode::Cpu,
                            },
                            WriteState::Ready,
                        ),
                        (curve("first"), WriteState::Ready),
                        (curve("later"), WriteState::Ready),
                    ],
                    true,
                )),
                ..Default::default()
            };
            let mut hw = FakeHardware {
                online: vec!["first".into(), "later".into()],
                fail: if failed {
                    vec!["first".into()]
                } else {
                    Vec::new()
                },
                cancel_write: Some(cancellation.clone()),
                ..Default::default()
            };
            let error = boot(
                &mut store,
                &mut hw,
                |_| Ok(()),
                || {},
                || check(&cancellation),
            )
            .err()
            .unwrap();
            assert!(error.is_service_shutdown()); // No LCD selection is returned.
            assert_eq!(*hw.log.borrow(), ["preflight:first", "write:first"]);
            let targets = &store.intent.as_ref().unwrap().targets;
            assert_eq!(
                targets[1].state,
                if failed {
                    WriteState::InFlightUnknown
                } else {
                    WriteState::Ready
                }
            );
            assert_eq!(targets[2].state, WriteState::Ready);
            assert_eq!(
                store
                    .log
                    .borrow()
                    .iter()
                    .filter(|event| *event == "ready")
                    .count(),
                usize::from(!failed)
            );
        }
    }

    #[test]
    fn shutdown_at_final_check_does_not_publish_saved_display() {
        let cancellation = HardwareCancellation::default();
        let mut store = FakeStore {
            intent: Some(saved(
                vec![(
                    MonitorTarget::Display {
                        device_id: DeviceId::new("lcd"),
                        mode: KrakenDisplayMode::Cpu,
                    },
                    WriteState::Ready,
                )],
                true,
            )),
            ..Default::default()
        };
        let mut hw = FakeHardware::default();
        let mut checks = 0;
        let error = boot(
            &mut store,
            &mut hw,
            |_| Ok(()),
            || {},
            || {
                checks += 1;
                if checks == 3 {
                    cancellation.cancel();
                }
                check(&cancellation)
            },
        )
        .err()
        .unwrap();
        assert!(error.is_service_shutdown());
        assert_eq!(checks, 3);
        assert!(hw.log.borrow().is_empty());
    }

    #[test]
    fn cancelled_startup_pause_returns_without_sleeping() {
        let cancellation = HardwareCancellation::default();
        cancellation.cancel();
        let before = Instant::now();
        startup_pause(&cancellation);
        assert!(before.elapsed() < Duration::from_millis(100));
    }

    #[test]
    fn persistence_failure_prevents_firmware_command() {
        let log = Rc::new(RefCell::new(Vec::new()));
        let mut store = FakeStore {
            intent: Some(saved(vec![(curve("aio"), WriteState::Ready)], true)),
            fail_mark: true,
            log: log.clone(),
            ..Default::default()
        };
        let mut hw = FakeHardware {
            online: vec!["aio".into()],
            log: log.clone(),
            ..Default::default()
        };
        assert!(run(&mut store, &mut hw).is_err());
        assert_eq!(
            *log.borrow(),
            ["load", "host:true", "preflight:aio", "block"]
        );
    }
}
