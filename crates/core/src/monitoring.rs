//! Service-owned startup monitoring intent and the outcome of the last attempt.
//!
//! These records are not hardware readback. `intent` describes the selected
//! target; `actual_state` describes what the service knows about applying it.
//! Host control's actual state remains in `HardwareSnapshot::host_control`.

use serde::{Deserialize, Deserializer, Serialize, de::Error as _};

use crate::{ChannelId, CurvePoint, DeviceId, KrakenDisplayMode};

/// Maximum diagnostic size on the monitoring snapshot wire, in UTF-8 bytes.
pub const MAX_MONITORING_ERROR_BYTES: usize = 512;

/// A bounded diagnostic for a selected target. Locally generated diagnostics
/// truncate at a UTF-8 boundary; peer-provided oversized diagnostics fail.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(transparent)]
pub struct MonitoringError(String);

impl MonitoringError {
    #[must_use]
    pub fn new(message: impl Into<String>) -> Self {
        let mut message = message.into();
        if message.len() > MAX_MONITORING_ERROR_BYTES {
            let mut end = MAX_MONITORING_ERROR_BYTES;
            while !message.is_char_boundary(end) {
                end -= 1;
            }
            message.truncate(end);
        }
        Self(message)
    }

    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl<'de> Deserialize<'de> for MonitoringError {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let message = String::deserialize(deserializer)?;
        if message.len() > MAX_MONITORING_ERROR_BYTES {
            return Err(D::Error::custom("monitoring last_error exceeds 512 bytes"));
        }
        Ok(Self(message))
    }
}

/// Firmware or LCD selection owned by the service; not evidence that a write
/// occurred. Host policy intent remains in its existing policy format.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MonitoringTargetIntent {
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

/// Service knowledge about the selected target, not a physical readback.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MonitoringActualState {
    /// Selected, but no confirmed application yet.
    Pending,
    /// A write completed successfully; this is not a device readback.
    Applied,
    /// Target could not be reached; no confirmed application.
    Unavailable,
    /// Outcome is uncertain; operator review is needed before another write.
    ReviewRequired,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MonitoringTargetSnapshot {
    pub intent: MonitoringTargetIntent,
    pub actual_state: MonitoringActualState,
    #[serde(deserialize_with = "required_last_error")]
    pub last_error: Option<MonitoringError>,
}

fn required_last_error<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<MonitoringError>, D::Error> {
    Option::<MonitoringError>::deserialize(deserializer)
}

/// Service-owned opt-in, future-startup replay setting, and per-target records.
/// Absence of a record does not imply any particular device state.
#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MonitoringSnapshot {
    pub opted_in: bool,
    pub auto_resume: bool,
    pub targets: Vec<MonitoringTargetSnapshot>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_not_opted_in_and_target_records_round_trip() {
        let empty = MonitoringSnapshot::default();
        assert_eq!(
            serde_json::to_string(&empty).unwrap(),
            r#"{"opted_in":false,"auto_resume":false,"targets":[]}"#
        );
        let snapshot = MonitoringSnapshot {
            opted_in: true,
            auto_resume: false,
            targets: vec![
                MonitoringTargetSnapshot {
                    intent: MonitoringTargetIntent::AioCurve {
                        device_id: DeviceId::new("aio"),
                        channel_id: ChannelId::new("pump"),
                        points: vec![CurvePoint {
                            temperature: 20,
                            duty: 40,
                        }],
                    },
                    actual_state: MonitoringActualState::ReviewRequired,
                    last_error: Some(MonitoringError::new("uncertain write")),
                },
                MonitoringTargetSnapshot {
                    intent: MonitoringTargetIntent::Display {
                        device_id: DeviceId::new("lcd"),
                        mode: KrakenDisplayMode::CpuGpu,
                    },
                    actual_state: MonitoringActualState::Unavailable,
                    last_error: None,
                },
            ],
        };
        let wire = serde_json::to_string(&snapshot).unwrap();
        assert_eq!(
            wire,
            r#"{"opted_in":true,"auto_resume":false,"targets":[{"intent":{"kind":"aio_curve","device_id":"aio","channel_id":"pump","points":[{"temperature":20,"duty":40}]},"actual_state":"review_required","last_error":"uncertain write"},{"intent":{"kind":"display","device_id":"lcd","mode":"cpu_gpu"},"actual_state":"unavailable","last_error":null}]}"#
        );
        assert_eq!(
            serde_json::from_str::<MonitoringSnapshot>(&wire).unwrap(),
            snapshot
        );
        for state in [
            MonitoringActualState::Pending,
            MonitoringActualState::Applied,
        ] {
            let wire = serde_json::to_string(&state).unwrap();
            assert_eq!(
                serde_json::from_str::<MonitoringActualState>(&wire).unwrap(),
                state
            );
        }
    }

    #[test]
    fn strict_nested_fields_and_bounded_errors() {
        for bad in [
            r#"{"opted_in":false,"auto_resume":false,"targets":[],"extra":1}"#,
            r#"{"opted_in":false,"targets":[]}"#,
            r#"{"opted_in":true,"auto_resume":false,"targets":[{"intent":{"kind":"display","device_id":"lcd","mode":"cpu","extra":1},"actual_state":"pending","last_error":null}]}"#,
            r#"{"opted_in":true,"auto_resume":false,"targets":[{"intent":{"kind":"aio_curve","device_id":"aio","channel_id":"pump","points":[{"temperature":20,"duty":40,"extra":1}]},"actual_state":"pending","last_error":null}]}"#,
            r#"{"opted_in":true,"auto_resume":false,"targets":[{"intent":{"kind":"display","device_id":"lcd","mode":"cpu"},"actual_state":"applied","last_error":null,"extra":1}]}"#,
            r#"{"opted_in":true,"auto_resume":false,"targets":[{"intent":{"kind":"display","device_id":"lcd","mode":"cpu"},"actual_state":"future","last_error":null}]}"#,
            r#"{"opted_in":true,"auto_resume":false,"targets":[{"intent":{"kind":"display","device_id":"lcd","mode":"cpu"},"actual_state":"pending"}]}"#,
        ] {
            assert!(
                serde_json::from_str::<MonitoringSnapshot>(bad).is_err(),
                "{bad}"
            );
        }
        let at_limit = "é".repeat(256);
        assert_eq!(
            serde_json::from_str::<MonitoringError>(&serde_json::to_string(&at_limit).unwrap())
                .unwrap()
                .as_str(),
            at_limit
        );
        assert!(
            serde_json::from_str::<MonitoringError>(
                &serde_json::to_string(&"é".repeat(257)).unwrap()
            )
            .is_err()
        );
        assert_eq!(
            MonitoringError::new(format!("{}é", "a".repeat(511)))
                .as_str()
                .len(),
            511
        );
        let mut snapshot = MonitoringSnapshot::default();
        snapshot.targets.push(MonitoringTargetSnapshot {
            intent: MonitoringTargetIntent::Display {
                device_id: DeviceId::new("lcd"),
                mode: KrakenDisplayMode::Cpu,
            },
            actual_state: MonitoringActualState::ReviewRequired,
            last_error: Some(MonitoringError::new("é".repeat(257))),
        });
        assert_eq!(
            serde_json::to_value(&snapshot).unwrap()["targets"][0]["last_error"]
                .as_str()
                .unwrap()
                .len(),
            512
        );
        let mut wire = serde_json::to_value(&snapshot).unwrap();
        wire["targets"][0]["last_error"] = serde_json::json!("é".repeat(257));
        assert!(serde_json::from_value::<MonitoringSnapshot>(wire).is_err());
    }
}
