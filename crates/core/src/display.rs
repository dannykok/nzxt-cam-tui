use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::DeviceId;

/// Service-owned LCD preset, saved by exact device identity after opt-in.
/// A selected face can be restored on service startup when auto-resume is on;
/// an unselected or replacement Kraken is never written automatically.
/// This value alone is not proof of the physical display state.
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum KrakenDisplayMode {
    #[default]
    BuiltinLiquid,
    Cpu,
    Gpu,
    Liquid,
    CpuGpu,
    CpuLiquid,
    GpuLiquid,
    CpuGpuLiquid,
}

impl KrakenDisplayMode {
    pub const ALL: [Self; 8] = [
        Self::BuiltinLiquid,
        Self::Cpu,
        Self::Gpu,
        Self::Liquid,
        Self::CpuGpu,
        Self::CpuLiquid,
        Self::GpuLiquid,
        Self::CpuGpuLiquid,
    ];

    pub const fn title(self) -> &'static str {
        match self {
            Self::BuiltinLiquid => "Built-in liquid",
            Self::Cpu => "CPU",
            Self::Gpu => "GPU",
            Self::Liquid => "Liquid",
            Self::CpuGpu => "CPU + GPU",
            Self::CpuLiquid => "CPU + liquid",
            Self::GpuLiquid => "GPU + liquid",
            Self::CpuGpuLiquid => "CPU + GPU + liquid",
        }
    }
}

/// Session mode/error plus eligible online 2023 Standard device, if any.
/// device_id is advertised even before the first selection; it becomes None
/// when the device is offline while mode and last_error remain available.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct KrakenDisplaySnapshot {
    pub device_id: Option<DeviceId>,
    pub mode: KrakenDisplayMode,
    #[serde(
        deserialize_with = "bounded_error_in",
        serialize_with = "bounded_error_out"
    )]
    pub last_error: Option<String>,
}

fn bounded_error_in<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<String>, D::Error> {
    use serde::de::Error as _;
    let error = Option::<String>::deserialize(deserializer)?;
    if error.as_ref().is_some_and(|error| error.len() > 512) {
        return Err(D::Error::custom("display last_error exceeds 512 bytes"));
    }
    Ok(error)
}

fn bounded_error_out<S: Serializer>(
    error: &Option<String>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    match error {
        None => serializer.serialize_none(),
        Some(error) => {
            let mut end = error.len().min(512);
            while !error.is_char_boundary(end) {
                end -= 1;
            }
            serializer.serialize_some(&error[..end])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_have_stable_wire_names_and_titles() {
        assert_eq!(KrakenDisplayMode::ALL.len(), 8);
        for mode in KrakenDisplayMode::ALL {
            assert!(!mode.title().is_empty());
            let wire = serde_json::to_string(&mode).unwrap();
            assert_eq!(
                serde_json::from_str::<KrakenDisplayMode>(&wire).unwrap(),
                mode
            );
        }
        assert_eq!(
            KrakenDisplaySnapshot::default().mode,
            KrakenDisplayMode::BuiltinLiquid
        );
        let snapshot = KrakenDisplaySnapshot {
            last_error: Some("é".repeat(257)),
            ..Default::default()
        };
        let json = serde_json::to_string(&snapshot).unwrap();
        assert_eq!(
            serde_json::from_str::<KrakenDisplaySnapshot>(&json)
                .unwrap()
                .last_error
                .unwrap()
                .len(),
            512
        );
        assert!(
            serde_json::from_str::<KrakenDisplaySnapshot>(&format!(
                r#"{{"device_id":null,"mode":"cpu","last_error":"{}"}}"#,
                "é".repeat(257)
            ))
            .is_err()
        );
    }
}
