//! Pure domain model shared by nzxt-cam components.

pub mod curve;
pub mod device;
pub mod display;
pub mod host;
pub mod monitoring;
pub mod sensor;

pub use curve::{ChannelId, CoolingChannel, CurvePoint, CurveState, TemperatureSource};
pub use device::{Device, DeviceId, DeviceKind, HardwareSnapshot};
pub use display::{KrakenDisplayMode, KrakenDisplaySnapshot};
pub use host::{
    HostChannelCapability, HostChannelPolicy, HostControlPolicy, HostControlSnapshot,
    HostControlState, HostCurve, HostCurvePoint, HostTemperatureSource,
};
pub use monitoring::{
    MAX_MONITORING_ERROR_BYTES, MonitoringActualState, MonitoringError, MonitoringSnapshot,
    MonitoringTargetIntent, MonitoringTargetSnapshot,
};
pub use sensor::{Reading, ReadingKind};
