//! Hardware-operation data-transfer objects.
//!
//! These messages define the protocol-v6 operation surface shared by clients
//! and the hardware service.

use std::fmt;

use nzxt_cam_core::{
    ChannelId, CurvePoint, DeviceId, HardwareSnapshot, HostChannelPolicy, HostControlPolicy,
    KrakenDisplayMode,
};
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _};

/// Maximum UTF-8 byte length of an error message sent over the protocol.
///
/// Locally constructed messages are truncated to this limit at a UTF-8
/// character boundary. Peer-provided messages over this limit are rejected.
pub const MAX_ERROR_MESSAGE_BYTES: usize = 512;

/// A bounded diagnostic suitable for sending to a protocol peer.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(transparent)]
pub struct ErrorMessage(String);

impl ErrorMessage {
    /// Builds a message from an internal diagnostic, truncating safely when it
    /// exceeds [`MAX_ERROR_MESSAGE_BYTES`].
    #[must_use]
    pub fn new(message: impl Into<String>) -> Self {
        let mut message = message.into();
        if message.len() > MAX_ERROR_MESSAGE_BYTES {
            let mut boundary = MAX_ERROR_MESSAGE_BYTES;
            while !message.is_char_boundary(boundary) {
                boundary -= 1;
            }
            message.truncate(boundary);
        }
        Self(message)
    }

    /// Returns the bounded diagnostic text.
    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl<'de> Deserialize<'de> for ErrorMessage {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let message = String::deserialize(deserializer)?;
        if message.len() > MAX_ERROR_MESSAGE_BYTES {
            return Err(D::Error::custom(format_args!(
                "error message exceeds the {MAX_ERROR_MESSAGE_BYTES}-byte limit"
            )));
        }
        Ok(Self(message))
    }
}

impl fmt::Display for ErrorMessage {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

/// Maximum number of firmware curves selected in one activation.
pub const MAX_MONITORING_FIRMWARE_CURVES: usize = 32;
/// Exact number of firmware curve points in an activation selection.
pub const MONITORING_FIRMWARE_CURVE_POINTS: usize = 40;
/// At most 32 firmware curves plus one host policy and one display selection.
pub const MAX_MONITORING_ACTIVATION_OUTCOMES: usize = MAX_MONITORING_FIRMWARE_CURVES + 2;

/// One firmware target selected for activation. Individual editor operations
/// remain available independently of this bundled startup selection.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MonitoringFirmwareCurve {
    pub device_id: DeviceId,
    pub channel_id: ChannelId,
    #[serde(
        deserialize_with = "deserialize_monitoring_points",
        serialize_with = "serialize_monitoring_points"
    )]
    pub points: Vec<CurvePoint>,
}

fn deserialize_monitoring_points<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Vec<CurvePoint>, D::Error> {
    let points = Vec::<CurvePoint>::deserialize(deserializer)?;
    if points.len() != MONITORING_FIRMWARE_CURVE_POINTS {
        return Err(D::Error::custom(
            "monitoring firmware curve requires 40 points",
        ));
    }
    Ok(points)
}

fn serialize_monitoring_points<S: Serializer>(
    points: &Vec<CurvePoint>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    if points.len() != MONITORING_FIRMWARE_CURVE_POINTS {
        return Err(serde::ser::Error::custom(
            "monitoring firmware curve requires 40 points",
        ));
    }
    points.serialize(serializer)
}

fn deserialize_monitoring_curves<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Vec<MonitoringFirmwareCurve>, D::Error> {
    let curves = Vec::<MonitoringFirmwareCurve>::deserialize(deserializer)?;
    if curves.len() > MAX_MONITORING_FIRMWARE_CURVES {
        return Err(D::Error::custom("too many monitoring firmware curves"));
    }
    Ok(curves)
}

fn serialize_monitoring_curves<S: Serializer>(
    curves: &Vec<MonitoringFirmwareCurve>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    if curves.len() > MAX_MONITORING_FIRMWARE_CURVES {
        return Err(serde::ser::Error::custom(
            "too many monitoring firmware curves",
        ));
    }
    curves.serialize(serializer)
}

fn deserialize_monitoring_outcomes<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Vec<MonitoringActivationOutcome>, D::Error> {
    let outcomes = Vec::<MonitoringActivationOutcome>::deserialize(deserializer)?;
    if outcomes.len() > MAX_MONITORING_ACTIVATION_OUTCOMES {
        return Err(D::Error::custom("too many monitoring activation outcomes"));
    }
    Ok(outcomes)
}

fn serialize_monitoring_outcomes<S: Serializer>(
    outcomes: &Vec<MonitoringActivationOutcome>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    if outcomes.len() > MAX_MONITORING_ACTIVATION_OUTCOMES {
        return Err(serde::ser::Error::custom(
            "too many monitoring activation outcomes",
        ));
    }
    outcomes.serialize(serializer)
}

fn required_option<'de, D: Deserializer<'de>, T: Deserialize<'de>>(
    deserializer: D,
) -> Result<Option<T>, D::Error> {
    Option::<T>::deserialize(deserializer)
}

/// Display target selected as part of activation.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MonitoringDisplaySelection {
    pub device_id: DeviceId,
    pub mode: KrakenDisplayMode,
}

/// Identity of one selected activation component; host control has its own
/// actual-state snapshot in `HardwareSnapshot::host_control`.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MonitoringActivationTarget {
    FirmwareCurve {
        device_id: DeviceId,
        channel_id: ChannelId,
    },
    HostControl {},
    Display {
        device_id: DeviceId,
    },
}

/// Outcome of this activation attempt, not a physical hardware readback.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MonitoringActivationStatus {
    Applied,
    Pending,
    Skipped,
    Failed,
    Unknown,
}

/// One outcome for each component selected by `ActivateMonitoring`.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MonitoringActivationOutcome {
    pub target: MonitoringActivationTarget,
    pub status: MonitoringActivationStatus,
    #[serde(deserialize_with = "required_option")]
    pub error: Option<ErrorMessage>,
}

/// A hardware operation requested by the TUI.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Request {
    GetSnapshot,
    ActivateMonitoring {
        #[serde(
            deserialize_with = "deserialize_monitoring_curves",
            serialize_with = "serialize_monitoring_curves"
        )]
        firmware_curves: Vec<MonitoringFirmwareCurve>,
        host_policy: Option<HostControlPolicy>,
        display: Option<MonitoringDisplaySelection>,
    },
    SetMonitoringAutoResume {
        enabled: bool,
    },
    SetKrakenDisplay {
        device_id: DeviceId,
        mode: KrakenDisplayMode,
    },
    ApplyFirmwareCurve {
        device_id: DeviceId,
        channel_id: ChannelId,
        points: Vec<CurvePoint>,
    },
    StartHostControl {
        complete_policy: HostControlPolicy,
    },
    UpdateHostControl {
        channel_policies: Vec<HostChannelPolicy>,
    },
    StopHostControl,
}

// Empty struct wire variants make `deny_unknown_fields` effective for the
// public unit variant as well as for variants carrying fields.
impl<'de> Deserialize<'de> for Request {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
        enum WireRequest {
            GetSnapshot {},
            ActivateMonitoring {
                #[serde(deserialize_with = "deserialize_monitoring_curves")]
                firmware_curves: Vec<MonitoringFirmwareCurve>,
                #[serde(deserialize_with = "required_option")]
                host_policy: Option<HostControlPolicy>,
                #[serde(deserialize_with = "required_option")]
                display: Option<MonitoringDisplaySelection>,
            },
            SetMonitoringAutoResume {
                enabled: bool,
            },
            SetKrakenDisplay {
                device_id: DeviceId,
                mode: KrakenDisplayMode,
            },
            ApplyFirmwareCurve {
                device_id: DeviceId,
                channel_id: ChannelId,
                points: Vec<CurvePoint>,
            },
            StartHostControl {
                complete_policy: HostControlPolicy,
            },
            UpdateHostControl {
                channel_policies: Vec<HostChannelPolicy>,
            },
            StopHostControl {},
        }

        Ok(match WireRequest::deserialize(deserializer)? {
            WireRequest::GetSnapshot {} => Self::GetSnapshot,
            WireRequest::ActivateMonitoring {
                firmware_curves,
                host_policy,
                display,
            } => Self::ActivateMonitoring {
                firmware_curves,
                host_policy,
                display,
            },
            WireRequest::SetMonitoringAutoResume { enabled } => {
                Self::SetMonitoringAutoResume { enabled }
            }
            WireRequest::SetKrakenDisplay { device_id, mode } => {
                Self::SetKrakenDisplay { device_id, mode }
            }
            WireRequest::ApplyFirmwareCurve {
                device_id,
                channel_id,
                points,
            } => Self::ApplyFirmwareCurve {
                device_id,
                channel_id,
                points,
            },
            WireRequest::StartHostControl { complete_policy } => {
                Self::StartHostControl { complete_policy }
            }
            WireRequest::UpdateHostControl { channel_policies } => {
                Self::UpdateHostControl { channel_policies }
            }
            WireRequest::StopHostControl {} => Self::StopHostControl,
        })
    }
}

/// The service's result for a hardware operation.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Response {
    Snapshot {
        snapshot: HardwareSnapshot,
    },
    FirmwareCurveApplied,
    MonitoringActivated {
        #[serde(serialize_with = "serialize_monitoring_outcomes")]
        outcomes: Vec<MonitoringActivationOutcome>,
    },
    MonitoringAutoResumeSet,
    KrakenDisplaySet,
    HostControlStarted,
    HostControlUpdated,
    HostControlStopped,
    Error {
        code: ErrorCode,
        message: ErrorMessage,
    },
}

// As above, use an empty struct wire variant so field rejection is uniform.
impl<'de> Deserialize<'de> for Response {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
        enum WireResponse {
            Snapshot {
                snapshot: HardwareSnapshot,
            },
            FirmwareCurveApplied {},
            MonitoringActivated {
                #[serde(deserialize_with = "deserialize_monitoring_outcomes")]
                outcomes: Vec<MonitoringActivationOutcome>,
            },
            MonitoringAutoResumeSet {},
            KrakenDisplaySet {},
            HostControlStarted {},
            HostControlUpdated {},
            HostControlStopped {},
            Error {
                code: ErrorCode,
                message: ErrorMessage,
            },
        }

        Ok(match WireResponse::deserialize(deserializer)? {
            WireResponse::Snapshot { snapshot } => Self::Snapshot { snapshot },
            WireResponse::FirmwareCurveApplied {} => Self::FirmwareCurveApplied,
            WireResponse::MonitoringActivated { outcomes } => {
                Self::MonitoringActivated { outcomes }
            }
            WireResponse::MonitoringAutoResumeSet {} => Self::MonitoringAutoResumeSet,
            WireResponse::KrakenDisplaySet {} => Self::KrakenDisplaySet,
            WireResponse::HostControlStarted {} => Self::HostControlStarted,
            WireResponse::HostControlUpdated {} => Self::HostControlUpdated,
            WireResponse::HostControlStopped {} => Self::HostControlStopped,
            WireResponse::Error { code, message } => Self::Error { code, message },
        })
    }
}

/// Finite machine-readable operation failure categories.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorCode {
    Unavailable,
    PermissionDenied,
    Unsupported,
    InvalidData,
    Timeout,
    UnknownOutcome,
    RestoreRequired,
    Internal,
}

#[cfg(test)]
mod tests {
    use nzxt_cam_core::{
        CoolingChannel, CurveState, Device, DeviceKind, HostChannelCapability, HostChannelPolicy,
        HostControlSnapshot, HostControlState, HostCurve, HostCurvePoint, HostTemperatureSource,
        Reading, ReadingKind, TemperatureSource,
    };
    use serde::{Serialize, de::DeserializeOwned};

    use super::*;

    fn assert_json_round_trip<T>(value: &T, expected: &str)
    where
        T: DeserializeOwned + fmt::Debug + PartialEq + Serialize,
    {
        let encoded = serde_json::to_string(value).unwrap();
        assert_eq!(encoded, expected);
        assert_eq!(serde_json::from_str::<T>(&encoded).unwrap(), *value);
    }

    fn representative_host_policy() -> HostControlPolicy {
        HostControlPolicy {
            channels: vec![HostChannelPolicy {
                channel_id: ChannelId::new("case-fan"),
                curve: HostCurve {
                    source: HostTemperatureSource::CpuGpuMax,
                    points: vec![
                        HostCurvePoint {
                            temperature_millidegrees: 30_000,
                            duty_percent: 25,
                        },
                        HostCurvePoint {
                            temperature_millidegrees: 80_000,
                            duty_percent: 100,
                        },
                    ],
                },
            }],
        }
    }

    fn representative_snapshot() -> HardwareSnapshot {
        HardwareSnapshot {
            devices: vec![Device {
                id: DeviceId::new("kraken-serial"),
                name: "Kraken".into(),
                model: "Kraken 2023".into(),
                kind: DeviceKind::LiquidCooler,
                online: true,
                readings: vec![
                    Reading::new("Liquid", 31.5, "°C", ReadingKind::Temperature),
                    Reading::new("Pump speed", 2_262.0, "rpm", ReadingKind::Speed),
                ],
                cooling_channels: vec![CoolingChannel {
                    id: ChannelId::new("pump"),
                    name: "Pump".into(),
                    source: TemperatureSource::Liquid,
                    min_duty: 20,
                    max_duty: 100,
                    points: vec![
                        CurvePoint {
                            temperature: 20,
                            duty: 40,
                        },
                        CurvePoint {
                            temperature: 59,
                            duty: 100,
                        },
                    ],
                    curve_state: CurveState::Unverified,
                }],
            }],
            sequence: 42,
            kraken_display: Default::default(),
            monitoring: Default::default(),
            host_control: HostControlSnapshot {
                state: HostControlState::Available,
                channels: vec![HostChannelCapability {
                    channel_id: ChannelId::new("case-fan"),
                    name: "Case fan".into(),
                    minimum_duty_percent: 20,
                }],
                active_policy: None,
                last_error: None,
            },
        }
    }

    #[test]
    fn request_variants_have_exact_json_and_round_trip() {
        assert_json_round_trip(&Request::GetSnapshot, r#"{"type":"get_snapshot"}"#);
        assert_json_round_trip(
            &Request::SetKrakenDisplay {
                device_id: DeviceId::new("kraken-serial"),
                mode: KrakenDisplayMode::CpuGpu,
            },
            r#"{"type":"set_kraken_display","device_id":"kraken-serial","mode":"cpu_gpu"}"#,
        );
        assert_json_round_trip(
            &Request::ApplyFirmwareCurve {
                device_id: DeviceId::new("kraken-serial"),
                channel_id: ChannelId::new("pump"),
                points: vec![
                    CurvePoint {
                        temperature: 20,
                        duty: 40,
                    },
                    CurvePoint {
                        temperature: 59,
                        duty: 100,
                    },
                ],
            },
            r#"{"type":"apply_firmware_curve","device_id":"kraken-serial","channel_id":"pump","points":[{"temperature":20,"duty":40},{"temperature":59,"duty":100}]}"#,
        );
        assert_json_round_trip(
            &Request::StartHostControl {
                complete_policy: representative_host_policy(),
            },
            r#"{"type":"start_host_control","complete_policy":{"channels":[{"channel_id":"case-fan","curve":{"source":"cpu_gpu_max","points":[{"temperature_millidegrees":30000,"duty_percent":25},{"temperature_millidegrees":80000,"duty_percent":100}]}}]}}"#,
        );
        assert_json_round_trip(
            &Request::UpdateHostControl {
                channel_policies: representative_host_policy().channels,
            },
            r#"{"type":"update_host_control","channel_policies":[{"channel_id":"case-fan","curve":{"source":"cpu_gpu_max","points":[{"temperature_millidegrees":30000,"duty_percent":25},{"temperature_millidegrees":80000,"duty_percent":100}]}}]}"#,
        );
        assert_json_round_trip(&Request::StopHostControl, r#"{"type":"stop_host_control"}"#);
        assert_json_round_trip(
            &Request::SetMonitoringAutoResume { enabled: false },
            r#"{"type":"set_monitoring_auto_resume","enabled":false}"#,
        );
    }

    #[test]
    fn monitoring_activation_round_trips_all_components_and_outcomes() {
        let points = (20..60)
            .map(|temperature| CurvePoint {
                temperature,
                duty: 45,
            })
            .collect();
        let request = Request::ActivateMonitoring {
            firmware_curves: vec![MonitoringFirmwareCurve {
                device_id: DeviceId::new("aio"),
                channel_id: ChannelId::new("pump"),
                points,
            }],
            host_policy: Some(representative_host_policy()),
            display: Some(MonitoringDisplaySelection {
                device_id: DeviceId::new("lcd"),
                mode: KrakenDisplayMode::CpuGpu,
            }),
        };
        let encoded = serde_json::to_value(&request).unwrap();
        assert_eq!(encoded["type"], "activate_monitoring");
        assert_eq!(
            encoded["firmware_curves"][0]["points"]
                .as_array()
                .unwrap()
                .len(),
            40
        );
        assert_eq!(
            encoded["host_policy"]["channels"][0]["channel_id"],
            "case-fan"
        );
        assert_eq!(encoded["display"]["mode"], "cpu_gpu");
        assert_eq!(serde_json::from_value::<Request>(encoded).unwrap(), request);
        assert_json_round_trip(
            &Request::ActivateMonitoring {
                firmware_curves: vec![],
                host_policy: None,
                display: None,
            },
            r#"{"type":"activate_monitoring","firmware_curves":[],"host_policy":null,"display":null}"#,
        );
        let response = Response::MonitoringActivated {
            outcomes: vec![
                MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::FirmwareCurve {
                        device_id: DeviceId::new("aio"),
                        channel_id: ChannelId::new("pump"),
                    },
                    status: MonitoringActivationStatus::Applied,
                    error: None,
                },
                MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::HostControl {},
                    status: MonitoringActivationStatus::Skipped,
                    error: Some(ErrorMessage::new("not selected")),
                },
                MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::Display {
                        device_id: DeviceId::new("lcd"),
                    },
                    status: MonitoringActivationStatus::Failed,
                    error: Some(ErrorMessage::new("offline")),
                },
                MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::Display {
                        device_id: DeviceId::new("pending"),
                    },
                    status: MonitoringActivationStatus::Pending,
                    error: None,
                },
                MonitoringActivationOutcome {
                    target: MonitoringActivationTarget::Display {
                        device_id: DeviceId::new("other"),
                    },
                    status: MonitoringActivationStatus::Unknown,
                    error: Some(ErrorMessage::new("write timed out")),
                },
            ],
        };
        assert_json_round_trip(
            &response,
            r#"{"type":"monitoring_activated","outcomes":[{"target":{"kind":"firmware_curve","device_id":"aio","channel_id":"pump"},"status":"applied","error":null},{"target":{"kind":"host_control"},"status":"skipped","error":"not selected"},{"target":{"kind":"display","device_id":"lcd"},"status":"failed","error":"offline"},{"target":{"kind":"display","device_id":"pending"},"status":"pending","error":null},{"target":{"kind":"display","device_id":"other"},"status":"unknown","error":"write timed out"}]}"#,
        );
    }

    #[test]
    fn monitoring_activation_rejects_invalid_nested_fields_and_sizes() {
        for invalid in [
            r#"{"type":"activate_monitoring","firmware_curves":[],"host_policy":null,"display":{"device_id":"lcd","mode":"cpu","extra":1}}"#,
            r#"{"type":"activate_monitoring","firmware_curves":[],"host_policy":null}"#,
            r#"{"type":"activate_monitoring","firmware_curves":[],"display":null}"#,
            r#"{"type":"activate_monitoring","firmware_curves":[],"host_policy":{"channels":[],"extra":1},"display":null}"#,
            r#"{"type":"activate_monitoring","firmware_curves":[{"device_id":"aio","channel_id":"pump","points":[],"extra":1}],"host_policy":null,"display":null}"#,
            r#"{"type":"activate_monitoring","firmware_curves":[{"device_id":"aio","channel_id":"pump","points":[]}],"host_policy":null,"display":null}"#,
        ] {
            assert!(
                serde_json::from_str::<Request>(invalid).is_err(),
                "{invalid}"
            );
        }
        let mut request = serde_json::json!({"type":"activate_monitoring","firmware_curves":[],"host_policy":null,"display":null});
        let point = serde_json::json!({"temperature":20,"duty":40});
        for len in [39, 41] {
            request["firmware_curves"] = serde_json::json!([{"device_id":"aio","channel_id":"pump","points":vec![point.clone();len]}]);
            assert!(serde_json::from_value::<Request>(request.clone()).is_err());
        }
        request["firmware_curves"] = serde_json::json!(vec![
            serde_json::json!({"device_id":"aio","channel_id":"pump","points":vec![point;40]});
            MAX_MONITORING_FIRMWARE_CURVES + 1
        ]);
        assert!(serde_json::from_value::<Request>(request).is_err());
        assert!(
            serde_json::to_value(Request::ActivateMonitoring {
                firmware_curves: vec![MonitoringFirmwareCurve {
                    device_id: DeviceId::new("aio"),
                    channel_id: ChannelId::new("pump"),
                    points: vec![]
                }],
                host_policy: None,
                display: None,
            })
            .is_err()
        );
        for invalid in [
            r#"{"type":"monitoring_activated","outcomes":[{"target":{"kind":"host_control","extra":1},"status":"applied","error":null}]}"#,
            r#"{"type":"monitoring_activated","outcomes":[{"target":{"kind":"display","device_id":"lcd"},"status":"applied","error":null,"extra":1}]}"#,
            r#"{"type":"monitoring_activated","outcomes":[{"target":{"kind":"display","device_id":"lcd"},"status":"future","error":null}]}"#,
            r#"{"type":"monitoring_activated","outcomes":[{"target":{"kind":"display","device_id":"lcd"},"status":"applied"}]}"#,
        ] {
            assert!(
                serde_json::from_str::<Response>(invalid).is_err(),
                "{invalid}"
            );
        }
        let oversized = serde_json::json!({"type":"monitoring_activated","outcomes":[{"target":{"kind":"host_control"},"status":"unknown","error":"é".repeat(257)}]});
        assert!(serde_json::from_value::<Response>(oversized).is_err());
        let outcome = MonitoringActivationOutcome {
            target: MonitoringActivationTarget::HostControl {},
            status: MonitoringActivationStatus::Applied,
            error: None,
        };
        let too_many = vec![outcome; MAX_MONITORING_ACTIVATION_OUTCOMES + 1];
        assert!(
            serde_json::to_value(Response::MonitoringActivated {
                outcomes: too_many.clone()
            })
            .is_err()
        );
        assert!(
            serde_json::from_value::<Response>(
                serde_json::json!({"type":"monitoring_activated","outcomes":too_many})
            )
            .is_err()
        );
    }

    #[test]
    fn batch_request_round_trips_multiple_groups_and_rejects_old_singular_shape() {
        let mut policies = representative_host_policy().channels;
        policies.push(HostChannelPolicy {
            channel_id: ChannelId::new("other"),
            curve: policies[0].curve.clone(),
        });
        let request = Request::UpdateHostControl {
            channel_policies: policies.clone(),
        };
        assert_eq!(
            serde_json::from_value::<Request>(serde_json::to_value(&request).unwrap()).unwrap(),
            request
        );
        assert!(
            serde_json::from_value::<Request>(serde_json::json!({
                "type": "update_host_control", "channel_policy": policies[0]
            }))
            .is_err()
        );
    }

    #[test]
    fn response_and_error_code_variants_have_exact_json_and_round_trip() {
        for (response, expected) in [
            (
                Response::FirmwareCurveApplied,
                r#"{"type":"firmware_curve_applied"}"#,
            ),
            (
                Response::KrakenDisplaySet,
                r#"{"type":"kraken_display_set"}"#,
            ),
            (
                Response::MonitoringAutoResumeSet,
                r#"{"type":"monitoring_auto_resume_set"}"#,
            ),
            (
                Response::HostControlStarted,
                r#"{"type":"host_control_started"}"#,
            ),
            (
                Response::HostControlUpdated,
                r#"{"type":"host_control_updated"}"#,
            ),
            (
                Response::HostControlStopped,
                r#"{"type":"host_control_stopped"}"#,
            ),
        ] {
            assert_json_round_trip(&response, expected);
        }

        for (code, wire_name) in [
            (ErrorCode::Unavailable, "unavailable"),
            (ErrorCode::PermissionDenied, "permission_denied"),
            (ErrorCode::Unsupported, "unsupported"),
            (ErrorCode::InvalidData, "invalid_data"),
            (ErrorCode::Timeout, "timeout"),
            (ErrorCode::UnknownOutcome, "unknown_outcome"),
            (ErrorCode::RestoreRequired, "restore_required"),
            (ErrorCode::Internal, "internal"),
        ] {
            assert_json_round_trip(&code, &format!(r#""{wire_name}""#));
            assert_json_round_trip(
                &Response::Error {
                    code,
                    message: ErrorMessage::new("diagnostic"),
                },
                &format!(r#"{{"type":"error","code":"{wire_name}","message":"diagnostic"}}"#),
            );
        }
    }

    #[test]
    fn representative_snapshot_has_exact_json_and_round_trips() {
        let snapshot = representative_snapshot();
        assert_json_round_trip(
            &Response::Snapshot { snapshot },
            r#"{"type":"snapshot","snapshot":{"devices":[{"id":"kraken-serial","name":"Kraken","model":"Kraken 2023","kind":"liquid_cooler","online":true,"readings":[{"label":"Liquid","value":31.5,"unit":"°C","kind":"temperature"},{"label":"Pump speed","value":2262.0,"unit":"rpm","kind":"speed"}],"cooling_channels":[{"id":"pump","name":"Pump","source":"liquid","min_duty":20,"max_duty":100,"points":[{"temperature":20,"duty":40},{"temperature":59,"duty":100}],"curve_state":"unverified"}]}],"sequence":42,"host_control":{"state":"available","channels":[{"channel_id":"case-fan","name":"Case fan","minimum_duty_percent":20}],"active_policy":null,"last_error":null},"kraken_display":{"device_id":null,"mode":"builtin_liquid","last_error":null},"monitoring":{"opted_in":false,"auto_resume":false,"targets":[]}}}"#,
        );
    }

    #[test]
    fn unknown_fields_and_variants_are_rejected() {
        for invalid in [
            r#"{"type":"get_snapshot","future":true}"#,
            r#"{"type":"set_monitoring_auto_resume","enabled":true,"future":true}"#,
            r#"{"type":"set_monitoring_auto_resume"}"#,
            r#"{"type":"activate_monitoring","firmware_curves":[],"host_policy":null,"display":null,"future":true}"#,
            r#"{"type":"set_kraken_display","device_id":"x","mode":"cpu","path":"/tmp/x"}"#,
            r#"{"type":"apply_firmware_curve","device_id":"device","channel_id":"pump","points":[],"future":true}"#,
            r#"{"type":"start_host_control","complete_policy":{"channels":[]},"future":true}"#,
            r#"{"type":"start_host_control","complete_policy":{"channels":[],"future":true}}"#,
            r#"{"type":"start_host_control","complete_policy":{"channels":[{"channel_id":"fan","curve":{"source":"cpu","points":[]},"future":true}]}}"#,
            r#"{"type":"start_host_control","complete_policy":{"channels":[{"channel_id":"fan","curve":{"source":"cpu","points":[{"temperature_millidegrees":30000,"duty_percent":20,"future":true}]}}]}}"#,
            r#"{"type":"update_host_control"}"#,
            r#"{"type":"update_host_control","channel_policy":{"channel_id":"fan","curve":{"source":"cpu","points":[]}}}"#,
            r#"{"type":"update_host_control","channel_policies":[{"channel_id":"fan","curve":{"source":"cpu","points":[]}}],"future":true}"#,
            r#"{"type":"heartbeat"}"#,
            r#"{"type":"heartbeat","future":true}"#,
            r#"{"type":"stop_host_control","future":true}"#,
            r#"{"type":"future_request"}"#,
        ] {
            assert!(
                serde_json::from_str::<Request>(invalid).is_err(),
                "{invalid}"
            );
        }

        for invalid in [
            r#"{"type":"firmware_curve_applied","future":true}"#,
            r#"{"type":"monitoring_auto_resume_set","future":true}"#,
            r#"{"type":"monitoring_activated","outcomes":[],"future":true}"#,
            r#"{"type":"kraken_display_set","future":true}"#,
            r#"{"type":"host_control_started","future":true}"#,
            r#"{"type":"host_control_updated","future":true}"#,
            r#"{"type":"heartbeat_acknowledged"}"#,
            r#"{"type":"heartbeat_acknowledged","future":true}"#,
            r#"{"type":"host_control_stopped","future":true}"#,
            r#"{"type":"snapshot","snapshot":{"devices":[],"sequence":1}}"#,
            r#"{"type":"snapshot","snapshot":{"devices":[],"sequence":1,"host_control":{"state":"disabled","channels":[],"last_error":null}}}"#,
            r#"{"type":"snapshot","snapshot":{"devices":[],"sequence":1,"host_control":{"state":"disabled","channels":[],"active_policy":null}}}"#,
            r#"{"type":"snapshot","snapshot":{"devices":[],"sequence":1,"host_control":{"state":"disabled","channels":[],"active_policy":null,"last_error":null}},"future":true}"#,
            r#"{"type":"snapshot","snapshot":{"devices":[],"sequence":1,"host_control":{"state":"disabled","channels":[],"active_policy":null,"last_error":null},"future":true}}"#,
            r#"{"type":"snapshot","snapshot":{"devices":[],"sequence":1,"host_control":{"state":"disabled","channels":[],"future":true}}}"#,
            r#"{"type":"error","code":"internal","message":"diagnostic","future":true}"#,
            r#"{"type":"future_response"}"#,
            r#"{"type":"error","code":"future_code","message":"diagnostic"}"#,
        ] {
            assert!(
                serde_json::from_str::<Response>(invalid).is_err(),
                "{invalid}"
            );
        }
    }

    #[test]
    fn snapshot_last_error_rejects_oversized_peer_diagnostic() {
        let oversized = "é".repeat(257);
        let response = serde_json::json!({"type":"snapshot", "snapshot": {
            "devices": [], "sequence": 1, "host_control": {
                "state": "restore_required", "channels": [], "active_policy": null,
                "last_error": oversized
            }
        }});
        assert!(serde_json::from_value::<Response>(response).is_err());
    }

    #[test]
    fn local_error_messages_truncate_without_splitting_utf8() {
        let ascii = ErrorMessage::new("x".repeat(MAX_ERROR_MESSAGE_BYTES + 10));
        assert_eq!(ascii.as_str().len(), MAX_ERROR_MESSAGE_BYTES);

        let internal = format!("{}étrailing", "x".repeat(MAX_ERROR_MESSAGE_BYTES - 1));
        let multibyte = ErrorMessage::new(internal);
        assert_eq!(multibyte.as_str(), "x".repeat(MAX_ERROR_MESSAGE_BYTES - 1));
        assert!(
            multibyte
                .as_str()
                .is_char_boundary(multibyte.as_str().len())
        );
    }

    #[test]
    fn peer_error_messages_accept_the_limit_and_reject_larger_multibyte_values() {
        let at_limit = "é".repeat(MAX_ERROR_MESSAGE_BYTES / 2);
        let encoded = serde_json::to_string(&at_limit).unwrap();
        let decoded = serde_json::from_str::<ErrorMessage>(&encoded).unwrap();
        assert_eq!(decoded.as_str(), at_limit);
        assert_eq!(decoded.as_str().len(), MAX_ERROR_MESSAGE_BYTES);

        let over_limit = "é".repeat(MAX_ERROR_MESSAGE_BYTES / 2 + 1);
        let encoded = serde_json::to_string(&over_limit).unwrap();
        assert!(serde_json::from_str::<ErrorMessage>(&encoded).is_err());

        let response = format!(r#"{{"type":"error","code":"internal","message":{encoded}}}"#);
        assert!(serde_json::from_str::<Response>(&response).is_err());
    }
}
