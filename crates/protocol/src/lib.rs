//! Shared protocol boundary between the TUI and hardware service.
//!
//! Version 6 uses bounded, four-byte length-prefixed JSON framing, a fixed
//! handshake, and operation DTOs for the two binaries shipped together.

pub mod framing;
pub mod handshake;
pub mod operations;
pub mod time_budget;

pub use framing::{
    FrameError, JsonFrameCodec, MAX_FRAME_LENGTH, from_value, validate_frame_payload,
};
pub use handshake::{ClientHello, PROTOCOL_VERSION_V6, RejectionCode, ServerResponse};
pub use operations::{
    ErrorCode, ErrorMessage, MAX_ERROR_MESSAGE_BYTES, MAX_MONITORING_ACTIVATION_OUTCOMES,
    MAX_MONITORING_FIRMWARE_CURVES, MONITORING_FIRMWARE_CURVE_POINTS, MonitoringActivationOutcome,
    MonitoringActivationStatus, MonitoringActivationTarget, MonitoringDisplaySelection,
    MonitoringFirmwareCurve, Request, Response,
};
