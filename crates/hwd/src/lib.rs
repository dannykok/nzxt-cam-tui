//! Linux hardware-service implementation.
//!
//! The service owns a self-contained liquidctl hardware implementation and
//! serves snapshots, opt-in monitoring activation, firmware curves and Kraken
//! 2023 Standard LCD selections over local IPC. The display worker uploads
//! independently of client connections. Durable service intent authorizes
//! future-startup replay of Ready curves and exact LCD selections after guarded
//! host recovery; uncertain writes require review. Without intent, legacy host
//! resume continues and no LCD reset or selection is attempted.

pub mod config;
pub mod hardware;
pub mod host_control;
pub mod it8689;
mod it8689_control;
mod kraken_display;
pub mod liquidctl;
pub mod monitor_intent;
mod monitor_resume;
mod monitor_service;
mod nvidia;
mod ownership;
pub mod policy_store;
#[cfg(target_os = "linux")]
pub mod server;
pub mod telemetry;

pub use config::{ConfigError, HardwareConfig, HostChannelConfig, HostControlConfig};
pub use host_control::{
    HOST_CONTROL_DOWNWARD_HYSTERESIS_MILLIDEGREES, HOST_CONTROL_RECOVERY_PATH,
    HOST_CONTROL_SENSOR_FRESHNESS, HOST_CONTROL_TICK_INTERVAL, HostControlEngine,
    HostControlShutdownHandle, HostControlSysfs, HostControlWorker, HostSensorSample,
    HostSensorSource, MonotonicTimeSource, ProductionHostSensors, RecoveryRecord, RecoveryStore,
    SystemMonotonicTime, TimedTemperature, recover_service_production, restore_from_record,
    restore_production,
};
pub use it8689::DiscoveredIt8689;
pub use ownership::HOST_CONTROL_LOCK_PATH;
