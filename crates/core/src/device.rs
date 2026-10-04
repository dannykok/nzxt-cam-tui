use std::fmt;

use serde::{Deserialize, Serialize};

use crate::{
    CoolingChannel, HostControlSnapshot, KrakenDisplaySnapshot, MonitoringSnapshot, Reading,
};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareSnapshot {
    pub devices: Vec<Device>,
    pub sequence: u64,
    pub host_control: HostControlSnapshot,
    #[serde(default)]
    pub kraken_display: KrakenDisplaySnapshot,
    pub monitoring: MonitoringSnapshot,
}

impl HardwareSnapshot {
    pub fn cooling_device_count(&self) -> usize {
        self.devices
            .iter()
            .filter(|device| !device.cooling_channels.is_empty())
            .count()
    }

    pub fn reading_count(&self) -> usize {
        self.devices
            .iter()
            .map(|device| device.readings.len())
            .sum()
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Device {
    pub id: DeviceId,
    pub name: String,
    pub model: String,
    pub kind: DeviceKind,
    pub online: bool,
    pub readings: Vec<Reading>,
    pub cooling_channels: Vec<CoolingChannel>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct DeviceId(pub String);

impl DeviceId {
    pub fn new(value: impl Into<String>) -> Self {
        Self(value.into())
    }
}

impl fmt::Display for DeviceId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceKind {
    LiquidCooler,
    FanController,
    LightingController,
}

impl DeviceKind {
    pub const ALL: [Self; 3] = [
        Self::LiquidCooler,
        Self::FanController,
        Self::LightingController,
    ];

    pub const fn title(self) -> &'static str {
        match self {
            Self::LiquidCooler => "AIO / LIQUID",
            Self::FanController => "FANS / AIRFLOW",
            Self::LightingController => "USB / LIGHTING",
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{ReadingKind, TemperatureSource};

    use super::*;

    #[test]
    fn snapshot_counts_readings_and_only_devices_with_cooling_channels() {
        let cooling = Device {
            id: DeviceId::new("cooler"),
            name: "Cooler".into(),
            model: "Model".into(),
            kind: DeviceKind::LiquidCooler,
            online: true,
            readings: vec![Reading::new("Liquid", 31.5, "°C", ReadingKind::Temperature)],
            cooling_channels: vec![CoolingChannel::firmware_curve(
                "pump",
                "Pump",
                TemperatureSource::Liquid,
                20,
                [50],
            )],
        };
        let telemetry_only = Device {
            id: DeviceId::new("lighting"),
            name: "Lighting".into(),
            model: "Model".into(),
            kind: DeviceKind::LightingController,
            online: true,
            readings: vec![
                Reading::new("Channels", 3.0, "", ReadingKind::ChannelCount),
                Reading::new("Duty", 50.0, "%", ReadingKind::Duty),
            ],
            cooling_channels: Vec::new(),
        };
        let snapshot = HardwareSnapshot {
            devices: vec![cooling, telemetry_only],
            sequence: 7,
            host_control: HostControlSnapshot::disabled(),
            kraken_display: Default::default(),
            monitoring: Default::default(),
        };

        assert_eq!(snapshot.cooling_device_count(), 1);
        assert_eq!(snapshot.reading_count(), 3);
    }

    #[test]
    fn device_identifiers_and_group_titles_keep_their_display_values() {
        assert_eq!(DeviceId::new("serial-1").to_string(), "serial-1");
        assert_eq!(DeviceKind::LiquidCooler.title(), "AIO / LIQUID");
        assert_eq!(DeviceKind::FanController.title(), "FANS / AIRFLOW");
        assert_eq!(DeviceKind::LightingController.title(), "USB / LIGHTING");
    }
}
