use std::{
    error::Error,
    fmt,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use nzxt_cam_protocol::{
    MonitoringActivationOutcome, MonitoringDisplaySelection, MonitoringFirmwareCurve,
};

use nzxt_cam_core::{
    ChannelId, CoolingChannel, CurvePoint, Device, DeviceId, DeviceKind, HardwareSnapshot,
    HostChannelCapability, HostChannelPolicy, HostControlPolicy, HostControlState, HostCurve,
    KrakenDisplayMode, KrakenDisplaySnapshot, Reading, ReadingKind, TemperatureSource,
};

pub trait HardwareBackend: Send + 'static {
    fn name(&self) -> &'static str;
    fn cancellation_handle(&self) -> Option<BackendCancellation> {
        None
    }
    fn refresh(&mut self) -> Result<HardwareSnapshot, BackendError>;
    fn activate_monitoring(
        &mut self,
        _curves: &[MonitoringFirmwareCurve],
        _host: Option<&HostControlPolicy>,
        _display: Option<&MonitoringDisplaySelection>,
    ) -> Result<Vec<MonitoringActivationOutcome>, BackendError> {
        Err(BackendError::with_kind(
            BackendErrorKind::Unsupported,
            "monitoring activation is unsupported",
        ))
    }
    fn set_monitoring_auto_resume(&mut self, _enabled: bool) -> Result<(), BackendError> {
        Err(BackendError::with_kind(
            BackendErrorKind::Unsupported,
            "monitoring auto-resume is unsupported",
        ))
    }
    fn set_kraken_display(
        &mut self,
        _device_id: &DeviceId,
        _mode: KrakenDisplayMode,
    ) -> Result<(), BackendError> {
        Err(BackendError::with_kind(
            BackendErrorKind::Unsupported,
            "Kraken display selection is unsupported by this backend",
        ))
    }
    fn apply_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), BackendError>;
    fn start_host_control(&mut self, _policy: &HostControlPolicy) -> Result<(), BackendError> {
        Err(BackendError::with_kind(
            BackendErrorKind::Unsupported,
            "host control is unsupported by this monitoring service",
        ))
    }
    fn update_host_control(
        &mut self,
        _channel_policies: &[HostChannelPolicy],
    ) -> Result<(), BackendError> {
        Err(BackendError::with_kind(
            BackendErrorKind::Unsupported,
            "host control is unsupported by this monitoring service",
        ))
    }
    fn stop_host_control(&mut self) -> Result<(), BackendError> {
        Err(BackendError::with_kind(
            BackendErrorKind::Unsupported,
            "host control is unsupported by this monitoring service",
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BackendErrorKind {
    Unavailable,
    PermissionDenied,
    Unsupported,
    InvalidData,
    Timeout,
    UnknownOutcome,
    RestoreRequired,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BackendError {
    kind: BackendErrorKind,
    message: String,
    invalidates_live_curve_claims: bool,
}

impl BackendError {
    pub fn new(message: impl Into<String>) -> Self {
        Self::with_kind(BackendErrorKind::Other, message)
    }

    pub fn with_kind(kind: BackendErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            invalidates_live_curve_claims: false,
        }
    }

    #[must_use]
    pub const fn invalidating_live_curve_claims(mut self) -> Self {
        self.invalidates_live_curve_claims = true;
        self
    }

    pub const fn kind(&self) -> BackendErrorKind {
        self.kind
    }

    pub const fn write_outcome_unknown(&self) -> bool {
        matches!(self.kind, BackendErrorKind::UnknownOutcome)
    }

    pub const fn invalidates_live_curve_claims(&self) -> bool {
        self.invalidates_live_curve_claims
    }
}

#[derive(Clone, Debug, Default)]
pub struct BackendCancellation {
    cancelled: Arc<AtomicBool>,
}

impl BackendCancellation {
    pub(crate) fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
    }

    pub(crate) fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::Acquire)
    }
}

impl fmt::Display for BackendError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl Error for BackendError {}

#[derive(Clone, Debug)]
pub struct DemoBackend {
    snapshot: HardwareSnapshot,
    tick: u64,
}

impl Default for DemoBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl DemoBackend {
    pub fn new() -> Self {
        let kraken = Device {
            id: DeviceId::new("kraken-2023"),
            name: "Kraken 2023".into(),
            model: "USB 1e71:300e".into(),
            kind: DeviceKind::LiquidCooler,
            online: true,
            readings: vec![
                Reading::new("Liquid", 31.8, "°C", ReadingKind::Temperature),
                Reading::new("Pump speed", 2_256.0, "rpm", ReadingKind::Speed),
                Reading::new("Pump duty", 60.0, "%", ReadingKind::Duty),
                Reading::new("Fan speed", 1_100.0, "rpm", ReadingKind::Speed),
                Reading::new("Fan duty", 50.0, "%", ReadingKind::Duty),
            ],
            cooling_channels: vec![
                CoolingChannel::firmware_curve(
                    "pump",
                    "Pump",
                    TemperatureSource::Liquid,
                    20,
                    curve_from_stops(&[
                        (20, 45),
                        (30, 50),
                        (35, 60),
                        (42, 75),
                        (50, 90),
                        (59, 100),
                    ]),
                ),
                CoolingChannel::firmware_curve(
                    "fan",
                    "Radiator fan",
                    TemperatureSource::Liquid,
                    0,
                    curve_from_stops(&[
                        (20, 25),
                        (28, 30),
                        (32, 40),
                        (36, 52),
                        (42, 70),
                        (50, 90),
                        (59, 100),
                    ]),
                ),
            ],
        };

        let fan_controller = Device {
            id: DeviceId::new("fan-controller-2022"),
            name: "RGB & Fan Controller".into(),
            model: "3-channel PWM".into(),
            kind: DeviceKind::FanController,
            online: true,
            readings: vec![
                Reading::new("Fan 1 speed", 920.0, "rpm", ReadingKind::Speed),
                Reading::new("Fan 1 duty", 36.0, "%", ReadingKind::Duty),
                Reading::new("Fan 2 speed", 875.0, "rpm", ReadingKind::Speed),
                Reading::new("Fan 2 duty", 36.0, "%", ReadingKind::Duty),
                Reading::new("Fan 3 speed", 0.0, "rpm", ReadingKind::Speed),
                Reading::new("Fan 3 duty", 0.0, "%", ReadingKind::Duty),
            ],
            cooling_channels: (1..=3)
                .map(|index| {
                    CoolingChannel::firmware_curve(
                        format!("fan-{index}"),
                        format!("Fan {index}"),
                        TemperatureSource::Cpu,
                        0,
                        curve_from_stops(&[(20, 25), (35, 35), (45, 50), (55, 75), (59, 100)]),
                    )
                })
                .collect(),
        };

        let host = Device {
            id: DeviceId::new("host-telemetry"),
            name: "Host telemetry".into(),
            model: "Simulated CPU / GPU / IT8689".into(),
            kind: DeviceKind::FanController,
            online: true,
            readings: vec![
                Reading::new("CPU Tctl", 42.0, "°C", ReadingKind::Temperature),
                Reading::new("NVIDIA GPU", 38.0, "°C", ReadingKind::Temperature),
                Reading::new("IT8689 fan 3", 860.0, "rpm", ReadingKind::Speed),
                Reading::new("IT8689 fan 4", 780.0, "rpm", ReadingKind::Speed),
            ],
            cooling_channels: Vec::new(),
        };

        let mut devices = vec![kraken, fan_controller, host];
        for index in 1..=3 {
            devices.push(Device {
                id: DeviceId::new(format!("rgb-controller-{index}")),
                name: format!("NZXT RGB 2023 #{index}"),
                model: "3-channel ARGB controller".into(),
                kind: DeviceKind::LightingController,
                online: true,
                readings: vec![Reading::new(
                    "ARGB channels",
                    3.0,
                    "",
                    ReadingKind::ChannelCount,
                )],
                cooling_channels: Vec::new(),
            });
        }

        Self {
            snapshot: HardwareSnapshot {
                devices,
                sequence: 0,
                kraken_display: KrakenDisplaySnapshot {
                    device_id: Some(DeviceId::new("kraken-2023")),
                    mode: KrakenDisplayMode::BuiltinLiquid,
                    last_error: None,
                },
                host_control: nzxt_cam_core::HostControlSnapshot {
                    state: HostControlState::Available,
                    channels: vec![
                        HostChannelCapability {
                            channel_id: ChannelId::new("case-fan-3"),
                            name: "Case fan candidate 3".into(),
                            minimum_duty_percent: 30,
                        },
                        HostChannelCapability {
                            channel_id: ChannelId::new("case-fan-4"),
                            name: "Case fan candidate 4".into(),
                            minimum_duty_percent: 35,
                        },
                    ],
                    active_policy: None,
                    last_error: None,
                },
                monitoring: Default::default(),
            },
            tick: 0,
        }
    }

    fn animate_readings(&mut self) {
        self.tick = self.tick.wrapping_add(1);
        self.snapshot.sequence = self.tick;

        let phase = (self.tick % 20) as f64;
        let triangle = if phase <= 10.0 { phase } else { 20.0 - phase };
        let centered = triangle - 5.0;

        if let Some(kraken) = self
            .snapshot
            .devices
            .iter_mut()
            .find(|device| device.id.0 == "kraken-2023")
        {
            set_reading(kraken, "Liquid", 31.8 + centered * 0.025);
            set_reading(kraken, "Pump speed", 2_256.0 + centered * 3.0);
            set_reading(kraken, "Fan speed", 1_100.0 + centered * 4.0);
        }

        if let Some(controller) = self
            .snapshot
            .devices
            .iter_mut()
            .find(|device| device.id.0 == "fan-controller-2022")
        {
            set_reading(controller, "Fan 1 speed", 920.0 + centered * 3.0);
            set_reading(controller, "Fan 2 speed", 875.0 - centered * 2.0);
        }
        if let Some(host) = self
            .snapshot
            .devices
            .iter_mut()
            .find(|device| device.id.0 == "host-telemetry")
        {
            set_reading(host, "CPU Tctl", 42.0 + centered * 0.1);
            set_reading(host, "NVIDIA GPU", 38.0 + centered * 0.08);
        }
    }
}

impl HardwareBackend for DemoBackend {
    fn name(&self) -> &'static str {
        "DEMO"
    }

    fn refresh(&mut self) -> Result<HardwareSnapshot, BackendError> {
        self.animate_readings();
        Ok(self.snapshot.clone())
    }

    fn set_kraken_display(
        &mut self,
        device_id: &DeviceId,
        mode: KrakenDisplayMode,
    ) -> Result<(), BackendError> {
        if self.snapshot.kraken_display.device_id.as_ref() != Some(device_id) {
            return Err(BackendError::with_kind(
                BackendErrorKind::Unavailable,
                "Kraken 2023 display unavailable",
            ));
        }
        self.snapshot.kraken_display.mode = mode;
        self.snapshot.kraken_display.last_error = None;
        Ok(())
    }

    fn apply_curve(
        &mut self,
        device_id: &DeviceId,
        channel_id: &ChannelId,
        points: &[CurvePoint],
    ) -> Result<(), BackendError> {
        let channel = self
            .snapshot
            .devices
            .iter_mut()
            .find(|device| &device.id == device_id)
            .and_then(|device| {
                device
                    .cooling_channels
                    .iter_mut()
                    .find(|channel| &channel.id == channel_id)
            })
            .ok_or_else(|| BackendError::new("selected cooling channel is no longer available"))?;

        channel.points = points.to_vec();
        Ok(())
    }

    fn start_host_control(&mut self, policy: &HostControlPolicy) -> Result<(), BackendError> {
        if self.snapshot.host_control.state != HostControlState::Available {
            return Err(BackendError::with_kind(
                BackendErrorKind::Unavailable,
                "demo host control is not available",
            ));
        }
        validate_demo_host_policy(&self.snapshot.host_control.channels, policy)?;
        self.snapshot.host_control.active_policy = Some(policy.clone());
        self.snapshot.host_control.state = HostControlState::Running;
        Ok(())
    }

    fn update_host_control(
        &mut self,
        channel_policies: &[HostChannelPolicy],
    ) -> Result<(), BackendError> {
        if self.snapshot.host_control.state != HostControlState::Running {
            return Err(BackendError::with_kind(
                BackendErrorKind::Unavailable,
                "demo host control is not running",
            ));
        }
        if channel_policies.is_empty() {
            return Err(invalid_demo_policy(
                "host update must contain at least one channel",
            ));
        }
        let mut proposed = self
            .snapshot
            .host_control
            .active_policy
            .clone()
            .ok_or_else(|| invalid_demo_policy("missing active policy"))?;
        let mut seen = std::collections::HashSet::new();
        for channel_policy in channel_policies {
            if !seen.insert(&channel_policy.channel_id) {
                return Err(invalid_demo_policy("duplicate fan channel"));
            }
            let target = proposed
                .channels
                .iter_mut()
                .find(|entry| entry.channel_id == channel_policy.channel_id)
                .ok_or_else(|| invalid_demo_policy("unknown fan channel"))?;
            *target = channel_policy.clone();
        }
        validate_demo_host_policy(&self.snapshot.host_control.channels, &proposed)?;
        self.snapshot.host_control.active_policy = Some(proposed);
        self.snapshot.host_control.last_error = None;
        Ok(())
    }

    fn stop_host_control(&mut self) -> Result<(), BackendError> {
        if self.snapshot.host_control.state == HostControlState::Available {
            return Ok(());
        }
        if self.snapshot.host_control.state != HostControlState::Running {
            return Err(BackendError::with_kind(
                BackendErrorKind::Unavailable,
                "demo host control is not running",
            ));
        }
        self.snapshot.host_control.active_policy = None;
        self.snapshot.host_control.state = HostControlState::Available;
        Ok(())
    }
}

fn validate_demo_host_policy(
    capabilities: &[HostChannelCapability],
    policy: &HostControlPolicy,
) -> Result<(), BackendError> {
    if policy.channels.len() != capabilities.len() {
        return Err(invalid_demo_policy(
            "host policy must contain every capability exactly once",
        ));
    }
    for capability in capabilities {
        let matches = policy
            .channels
            .iter()
            .filter(|channel| channel.channel_id == capability.channel_id)
            .collect::<Vec<_>>();
        if matches.len() != 1 {
            return Err(invalid_demo_policy(
                "host policy must contain every capability exactly once",
            ));
        }
        validate_demo_host_curve(&matches[0].curve, capability.minimum_duty_percent.min(100))?;
    }
    Ok(())
}

fn validate_demo_host_curve(curve: &HostCurve, minimum: u8) -> Result<(), BackendError> {
    if curve.points.len() < 2 || curve.points.len() > 64 {
        return Err(invalid_demo_policy("host curves require 2..=64 points"));
    }
    let mut previous_temperature = None;
    let mut previous_duty = minimum;
    for point in &curve.points {
        if !(0..=120_000).contains(&point.temperature_millidegrees)
            || previous_temperature
                .is_some_and(|previous| point.temperature_millidegrees <= previous)
            || !(minimum..=100).contains(&point.duty_percent)
            || point.duty_percent < previous_duty
        {
            return Err(invalid_demo_policy(
                "host curve temperatures and duties are invalid",
            ));
        }
        previous_temperature = Some(point.temperature_millidegrees);
        previous_duty = point.duty_percent;
    }
    if previous_duty != 100 {
        return Err(invalid_demo_policy(
            "host curve final duty must be exactly 100 percent",
        ));
    }
    Ok(())
}

fn invalid_demo_policy(message: impl Into<String>) -> BackendError {
    BackendError::with_kind(BackendErrorKind::InvalidData, message)
}

fn set_reading(device: &mut Device, label: &str, value: f64) {
    if let Some(reading) = device
        .readings
        .iter_mut()
        .find(|reading| reading.label == label)
    {
        reading.value = value;
    }
}

fn curve_from_stops(stops: &[(u8, u8)]) -> Vec<u8> {
    (20_u8..60)
        .map(|temperature| interpolate_stops(stops, temperature))
        .collect()
}

fn interpolate_stops(stops: &[(u8, u8)], temperature: u8) -> u8 {
    let Some(&(first_temperature, first_duty)) = stops.first() else {
        return 0;
    };
    if temperature <= first_temperature {
        return first_duty;
    }

    for window in stops.windows(2) {
        let (low_temperature, low_duty) = window[0];
        let (high_temperature, high_duty) = window[1];
        if temperature <= high_temperature {
            let span = u16::from(high_temperature - low_temperature);
            let position = u16::from(temperature - low_temperature);
            let low = u16::from(low_duty);
            let high = u16::from(high_duty);
            return (low + (high - low) * position / span) as u8;
        }
    }

    stops.last().map_or(0, |(_, duty)| *duty)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn demo_backend_exposes_two_cooling_devices_and_forty_point_curves() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().expect("demo refresh should work");

        assert_eq!(
            snapshot
                .devices
                .iter()
                .filter(|device| !device.cooling_channels.is_empty())
                .count(),
            2
        );
        for channel in snapshot
            .devices
            .iter()
            .flat_map(|device| &device.cooling_channels)
        {
            assert_eq!(channel.points.len(), 40);
            assert_eq!(channel.points.first().unwrap().temperature, 20);
            assert_eq!(channel.points.last().unwrap().temperature, 59);
        }
    }

    #[test]
    fn demo_host_control_validates_and_simulates_one_session() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        assert_eq!(snapshot.host_control.state, HostControlState::Available);
        assert_eq!(snapshot.host_control.channels.len(), 2);
        let policy = HostControlPolicy {
            channels: snapshot
                .host_control
                .channels
                .iter()
                .map(|capability| nzxt_cam_core::HostChannelPolicy {
                    channel_id: capability.channel_id.clone(),
                    curve: HostCurve {
                        source: nzxt_cam_core::HostTemperatureSource::Cpu,
                        points: vec![
                            nzxt_cam_core::HostCurvePoint {
                                temperature_millidegrees: 22_000,
                                duty_percent: capability.minimum_duty_percent,
                            },
                            nzxt_cam_core::HostCurvePoint {
                                temperature_millidegrees: 100_000,
                                duty_percent: 100,
                            },
                        ],
                    },
                })
                .collect(),
        };

        backend.start_host_control(&policy).unwrap();
        assert_eq!(
            backend.refresh().unwrap().host_control.state,
            HostControlState::Running
        );
        assert_eq!(
            backend.refresh().unwrap().host_control.active_policy,
            Some(policy)
        );
        backend.stop_host_control().unwrap();
        backend.stop_host_control().unwrap(); // idempotent once available
        assert!(
            backend
                .refresh()
                .unwrap()
                .host_control
                .active_policy
                .is_none()
        );
        assert_eq!(
            backend.refresh().unwrap().host_control.state,
            HostControlState::Available
        );
    }

    #[test]
    fn demo_rejects_incomplete_host_policy_without_changing_state() {
        let mut backend = DemoBackend::new();
        let error = backend
            .start_host_control(&HostControlPolicy { channels: vec![] })
            .unwrap_err();
        assert_eq!(error.kind(), BackendErrorKind::InvalidData);
        assert_eq!(
            backend.refresh().unwrap().host_control.state,
            HostControlState::Available
        );
    }

    #[test]
    fn demo_batch_is_all_or_nothing() {
        let mut backend = DemoBackend::new();
        let capabilities = backend.snapshot.host_control.channels.clone();
        let original = HostControlPolicy {
            channels: capabilities
                .iter()
                .map(|capability| HostChannelPolicy {
                    channel_id: capability.channel_id.clone(),
                    curve: HostCurve {
                        source: nzxt_cam_core::HostTemperatureSource::Cpu,
                        points: vec![
                            nzxt_cam_core::HostCurvePoint {
                                temperature_millidegrees: 20_000,
                                duty_percent: capability.minimum_duty_percent,
                            },
                            nzxt_cam_core::HostCurvePoint {
                                temperature_millidegrees: 100_000,
                                duty_percent: 100,
                            },
                        ],
                    },
                })
                .collect(),
        };
        backend.start_host_control(&original).unwrap();
        let mut changed = original.channels.clone();
        changed[0].curve.source = nzxt_cam_core::HostTemperatureSource::Gpu;
        changed[1].curve.source = nzxt_cam_core::HostTemperatureSource::CpuGpuMax;
        let mut invalid = changed[1].clone();
        invalid.curve.points[0].duty_percent = 0;
        for batch in [
            vec![],
            vec![changed[0].clone(), changed[0].clone()],
            vec![
                changed[0].clone(),
                HostChannelPolicy {
                    channel_id: ChannelId::new("unknown"),
                    ..changed[1].clone()
                },
            ],
            vec![changed[0].clone(), invalid],
        ] {
            assert_eq!(
                backend.update_host_control(&batch).unwrap_err().kind(),
                BackendErrorKind::InvalidData
            );
            assert_eq!(
                backend.snapshot.host_control.active_policy,
                Some(original.clone())
            );
        }
        backend.update_host_control(&changed).unwrap();
        assert_eq!(
            backend
                .snapshot
                .host_control
                .active_policy
                .unwrap()
                .channels,
            changed
        );
    }

    #[test]
    fn interpolation_connects_curve_stops() {
        assert_eq!(interpolate_stops(&[(20, 20), (30, 40)], 20), 20);
        assert_eq!(interpolate_stops(&[(20, 20), (30, 40)], 25), 30);
        assert_eq!(interpolate_stops(&[(20, 20), (30, 40)], 40), 40);
    }

    #[test]
    fn only_unknown_outcomes_report_an_unknown_write() {
        let timeout = BackendError::with_kind(BackendErrorKind::Timeout, "before dispatch");
        assert!(!timeout.write_outcome_unknown());
        assert!(!timeout.invalidates_live_curve_claims());

        let unknown = BackendError::with_kind(BackendErrorKind::UnknownOutcome, "after dispatch")
            .invalidating_live_curve_claims();
        assert!(unknown.write_outcome_unknown());
        assert!(unknown.invalidates_live_curve_claims());
    }
}
