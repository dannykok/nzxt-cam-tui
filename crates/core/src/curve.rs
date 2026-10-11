use std::fmt;

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CoolingChannel {
    pub id: ChannelId,
    pub name: String,
    pub source: TemperatureSource,
    pub min_duty: u8,
    pub max_duty: u8,
    pub points: Vec<CurvePoint>,
    pub curve_state: CurveState,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CurveState {
    Unverified,
    Applied,
}

impl CoolingChannel {
    pub fn firmware_curve(
        id: impl Into<String>,
        name: impl Into<String>,
        source: TemperatureSource,
        min_duty: u8,
        duties: impl IntoIterator<Item = u8>,
    ) -> Self {
        Self::curve(id, name, source, min_duty, duties, CurveState::Applied)
    }

    pub fn unverified_firmware_curve(
        id: impl Into<String>,
        name: impl Into<String>,
        source: TemperatureSource,
        min_duty: u8,
        duties: impl IntoIterator<Item = u8>,
    ) -> Self {
        Self::curve(id, name, source, min_duty, duties, CurveState::Unverified)
    }

    fn curve(
        id: impl Into<String>,
        name: impl Into<String>,
        source: TemperatureSource,
        min_duty: u8,
        duties: impl IntoIterator<Item = u8>,
        curve_state: CurveState,
    ) -> Self {
        let points = (20_u8..60)
            .zip(duties)
            .map(|(temperature, duty)| CurvePoint {
                temperature,
                duty: duty.clamp(min_duty, 100),
            })
            .collect();

        Self {
            id: ChannelId::new(id),
            name: name.into(),
            source,
            min_duty,
            max_duty: 100,
            points,
            curve_state,
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct ChannelId(pub String);

impl ChannelId {
    pub fn new(value: impl Into<String>) -> Self {
        Self(value.into())
    }
}

impl fmt::Display for ChannelId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TemperatureSource {
    #[default]
    Liquid,
    Cpu,
    Gpu,
}

impl TemperatureSource {
    pub fn is_liquid(&self) -> bool {
        *self == Self::Liquid
    }
}

/// The editor uses 40 points in either coolant or host-temperature space.
pub fn kraken_temperature(source: TemperatureSource, index: usize) -> u8 {
    match source {
        TemperatureSource::Liquid => 20 + index as u8,
        TemperatureSource::Cpu => 22 + 2 * index as u8,
        TemperatureSource::Gpu => 22 + 2 * index as u8,
    }
}

pub fn valid_kraken_curve(source: TemperatureSource, points: &[CurvePoint], minimum: u8) -> bool {
    matches!(source, TemperatureSource::Liquid | TemperatureSource::Cpu)
        && points.len() == 40
        && points.iter().enumerate().all(|(index, point)| {
            point.temperature == kraken_temperature(source, index)
                && (minimum..=100).contains(&point.duty)
        })
        && points.windows(2).all(|pair| pair[0].duty <= pair[1].duty)
        && points.last().is_some_and(|point| point.duty == 100)
}

impl fmt::Display for TemperatureSource {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::Liquid => "LIQUID",
            Self::Cpu => "CPU",
            Self::Gpu => "GPU",
        })
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CurvePoint {
    pub temperature: u8,
    pub duty: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn liquid_and_cpu_curves_have_distinct_validated_domains() {
        for source in [TemperatureSource::Liquid, TemperatureSource::Cpu] {
            let mut points: Vec<_> = (0..40)
                .map(|index| CurvePoint {
                    temperature: kraken_temperature(source, index),
                    duty: if index == 39 { 100 } else { 40 },
                })
                .collect();
            assert!(valid_kraken_curve(source, &points, 20));
            assert!(!valid_kraken_curve(TemperatureSource::Gpu, &points, 0));
            points[0].duty = 19;
            assert!(!valid_kraken_curve(source, &points, 20));
            points[0].duty = 40;
            points[39].duty = 99;
            assert!(!valid_kraken_curve(source, &points, 0));
        }
        assert_eq!(kraken_temperature(TemperatureSource::Cpu, 39), 100);
        assert_eq!(kraken_temperature(TemperatureSource::Liquid, 39), 59);
    }

    #[test]
    fn firmware_curve_preserves_temperature_mapping_and_clamps_duties() {
        let channel = CoolingChannel::firmware_curve(
            "pump",
            "Pump",
            TemperatureSource::Liquid,
            30,
            [10, 45, 120],
        );

        assert_eq!(channel.id, ChannelId::new("pump"));
        assert_eq!(channel.curve_state, CurveState::Applied);
        assert_eq!(
            channel.points,
            vec![
                CurvePoint {
                    temperature: 20,
                    duty: 30,
                },
                CurvePoint {
                    temperature: 21,
                    duty: 45,
                },
                CurvePoint {
                    temperature: 22,
                    duty: 100,
                },
            ]
        );
    }

    #[test]
    fn unverified_curve_and_domain_values_keep_their_display_values() {
        let channel = CoolingChannel::unverified_firmware_curve(
            "fan",
            "Fan",
            TemperatureSource::Cpu,
            20,
            [40],
        );

        assert_eq!(channel.curve_state, CurveState::Unverified);
        assert_eq!(ChannelId::new("fan").to_string(), "fan");
        assert_eq!(TemperatureSource::Liquid.to_string(), "LIQUID");
        assert_eq!(TemperatureSource::Cpu.to_string(), "CPU");
        assert_eq!(TemperatureSource::Gpu.to_string(), "GPU");
    }
}
