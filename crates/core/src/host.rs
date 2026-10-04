use std::fmt;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::ChannelId;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum HostTemperatureSource {
    Cpu,
    Gpu,
    CpuGpuMax,
}

impl fmt::Display for HostTemperatureSource {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::Cpu => "CPU",
            Self::Gpu => "GPU",
            Self::CpuGpuMax => "MAX(CPU,GPU)",
        })
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostCurvePoint {
    pub temperature_millidegrees: i32,
    pub duty_percent: u8,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostCurve {
    pub source: HostTemperatureSource,
    pub points: Vec<HostCurvePoint>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostChannelPolicy {
    pub channel_id: ChannelId,
    pub curve: HostCurve,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostControlPolicy {
    pub channels: Vec<HostChannelPolicy>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostChannelCapability {
    pub channel_id: ChannelId,
    pub name: String,
    pub minimum_duty_percent: u8,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum HostControlState {
    Disabled,
    Available,
    Running,
    Restoring,
    RestoreRequired,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HostControlSnapshot {
    pub state: HostControlState,
    pub channels: Vec<HostChannelCapability>,
    #[serde(deserialize_with = "required_active_policy")]
    pub active_policy: Option<HostControlPolicy>,
    #[serde(
        deserialize_with = "required_last_error",
        serialize_with = "bounded_last_error"
    )]
    pub last_error: Option<String>,
}

fn required_active_policy<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<HostControlPolicy>, D::Error> {
    Option::<HostControlPolicy>::deserialize(deserializer)
}

fn required_last_error<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<String>, D::Error> {
    use serde::de::Error as _;
    let message = Option::<String>::deserialize(deserializer)?;
    if message.as_ref().is_some_and(|text| text.len() > 512) {
        return Err(D::Error::custom("last_error exceeds 512 bytes"));
    }
    Ok(message)
}

fn bounded_last_error<S: Serializer>(
    message: &Option<String>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    match message {
        None => serializer.serialize_none(),
        Some(text) => {
            let mut end = text.len().min(512);
            while !text.is_char_boundary(end) {
                end -= 1;
            }
            serializer.serialize_some(&text[..end])
        }
    }
}

impl HostControlSnapshot {
    #[must_use]
    pub const fn disabled() -> Self {
        Self {
            state: HostControlState::Disabled,
            channels: Vec::new(),
            active_policy: None,
            last_error: None,
        }
    }
}

impl Default for HostControlSnapshot {
    fn default() -> Self {
        Self::disabled()
    }
}

#[cfg(test)]
mod tests {
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

    fn assert_rejected<T>(json: &str)
    where
        T: DeserializeOwned,
    {
        assert!(serde_json::from_str::<T>(json).is_err(), "{json}");
    }

    fn policy() -> HostControlPolicy {
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

    #[test]
    fn temperature_sources_have_exact_wire_and_display_values() {
        for (source, wire_name, display) in [
            (HostTemperatureSource::Cpu, "cpu", "CPU"),
            (HostTemperatureSource::Gpu, "gpu", "GPU"),
            (
                HostTemperatureSource::CpuGpuMax,
                "cpu_gpu_max",
                "MAX(CPU,GPU)",
            ),
        ] {
            assert_json_round_trip(&source, &format!(r#""{wire_name}""#));
            assert_eq!(source.to_string(), display);
        }
    }

    #[test]
    fn host_control_states_have_exact_wire_values() {
        for (state, wire_name) in [
            (HostControlState::Disabled, "disabled"),
            (HostControlState::Available, "available"),
            (HostControlState::Running, "running"),
            (HostControlState::Restoring, "restoring"),
            (HostControlState::RestoreRequired, "restore_required"),
        ] {
            assert_json_round_trip(&state, &format!(r#""{wire_name}""#));
        }
    }

    #[test]
    fn policies_and_snapshots_have_exact_json_and_round_trip() {
        assert_json_round_trip(
            &policy(),
            r#"{"channels":[{"channel_id":"case-fan","curve":{"source":"cpu_gpu_max","points":[{"temperature_millidegrees":30000,"duty_percent":25},{"temperature_millidegrees":80000,"duty_percent":100}]}}]}"#,
        );

        let snapshot = HostControlSnapshot {
            state: HostControlState::Available,
            channels: vec![HostChannelCapability {
                channel_id: ChannelId::new("case-fan"),
                name: "Case fan".into(),
                minimum_duty_percent: 20,
            }],
            active_policy: None,
            last_error: None,
        };
        assert_json_round_trip(
            &snapshot,
            r#"{"state":"available","channels":[{"channel_id":"case-fan","name":"Case fan","minimum_duty_percent":20}],"active_policy":null,"last_error":null}"#,
        );
        let running = HostControlSnapshot {
            state: HostControlState::Running,
            active_policy: Some(policy()),
            ..snapshot
        };
        assert_eq!(
            serde_json::from_str::<HostControlSnapshot>(&serde_json::to_string(&running).unwrap())
                .unwrap(),
            running
        );
        assert_eq!(
            HostControlSnapshot::default(),
            HostControlSnapshot::disabled()
        );
    }

    #[test]
    fn unknown_variants_and_fields_at_every_nested_level_are_rejected() {
        assert_rejected::<HostTemperatureSource>(r#""future_source""#);
        assert_rejected::<HostControlState>(r#""future_state""#);
        assert_rejected::<HostCurvePoint>(
            r#"{"temperature_millidegrees":30000,"duty_percent":25,"future":true}"#,
        );
        assert_rejected::<HostCurve>(r#"{"source":"cpu","points":[],"future":true}"#);
        assert_rejected::<HostCurve>(
            r#"{"source":"cpu","points":[{"temperature_millidegrees":30000,"duty_percent":25,"future":true}]}"#,
        );
        assert_rejected::<HostChannelPolicy>(
            r#"{"channel_id":"fan","curve":{"source":"cpu","points":[]},"future":true}"#,
        );
        assert_rejected::<HostChannelPolicy>(
            r#"{"channel_id":"fan","curve":{"source":"cpu","points":[],"future":true}}"#,
        );
        assert_rejected::<HostControlPolicy>(r#"{"channels":[],"future":true}"#);
        assert_rejected::<HostControlPolicy>(
            r#"{"channels":[{"channel_id":"fan","curve":{"source":"cpu","points":[]},"future":true}]}"#,
        );
        assert_rejected::<HostChannelCapability>(
            r#"{"channel_id":"fan","name":"Fan","minimum_duty_percent":20,"future":true}"#,
        );
        assert_rejected::<HostControlSnapshot>(r#"{"state":"disabled","channels":[]}"#);
        assert_rejected::<HostControlSnapshot>(
            r#"{"state":"disabled","channels":[],"active_policy":null}"#,
        );
        let oversized = serde_json::json!({"state":"disabled","channels":[],"active_policy":null,"last_error":"é".repeat(257)});
        assert!(serde_json::from_value::<HostControlSnapshot>(oversized).is_err());
        let boundary = HostControlSnapshot {
            last_error: Some("é".repeat(256)),
            ..HostControlSnapshot::disabled()
        };
        assert_eq!(
            serde_json::from_str::<HostControlSnapshot>(&serde_json::to_string(&boundary).unwrap())
                .unwrap(),
            boundary
        );
        let longer = HostControlSnapshot {
            last_error: Some("é".repeat(257)),
            ..HostControlSnapshot::disabled()
        };
        let wire = serde_json::to_value(longer).unwrap();
        assert_eq!(wire["last_error"].as_str().unwrap().len(), 512);
        assert_rejected::<HostControlSnapshot>(
            r#"{"state":"disabled","channels":[],"active_policy":null,"future":true}"#,
        );
        assert_rejected::<HostControlSnapshot>(
            r#"{"state":"available","channels":[{"channel_id":"fan","name":"Fan","minimum_duty_percent":20,"future":true}]}"#,
        );
    }
}
