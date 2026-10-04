//! Shared USB and request time budgets for the service and its client.
//!
//! Response deadlines cover every possible serial subprocess, including a
//! bounded wait for the shared LCD/USB lock before *each* command. Connecting,
//! handshaking and sending have independent deadlines: startup handshakes do
//! not wait for hardware initialization. These are limits, not retry policies.

use std::time::Duration;

use crate::{MAX_MONITORING_FIRMWARE_CURVES, Request};

/// Execution budget once liquidctl is spawned; lock waiting is separate.
pub const LIQUIDCTL_COMMAND_TIMEOUT: Duration = Duration::from_secs(5);
/// Enough for one whole competing command plus polling/cleanup headroom.
/// A continuously busy or stuck LCD worker cannot block a client indefinitely.
pub const USB_LOCK_ACQUIRE_TIMEOUT: Duration = Duration::from_secs(6);
/// Bounds cancellation latency while waiting for the synchronous USB mutex.
pub const USB_LOCK_POLL_INTERVAL: Duration = Duration::from_millis(10);
pub const SERIAL_USB_COMMAND_BUDGET: Duration =
    USB_LOCK_ACQUIRE_TIMEOUT.saturating_add(LIQUIDCTL_COMMAND_TIMEOUT);

pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
pub const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(2);
pub const SEND_TIMEOUT: Duration = Duration::from_secs(2);
/// Request dispatch, command cleanup and response delivery headroom.
pub const RESPONSE_MARGIN: Duration = Duration::from_secs(2);
/// Durable selection/in-flight/ready transitions, reserved per selected target.
pub const PERSISTENCE_MARGIN_PER_TARGET: Duration = Duration::from_secs(2);
/// Independent motherboard worker dispatch and persistence allowance.
pub const HOST_OPERATION_RESPONSE_TIMEOUT: Duration = Duration::from_secs(7);

pub const SNAPSHOT_USB_COMMANDS: u32 = 2; // list + status
pub const APPLY_USB_COMMANDS: u32 = 3; // fresh list + status + write
pub const DISPLAY_PREFLIGHT_USB_COMMANDS: u32 = 1; // fresh list; upload is asynchronous
pub const MAX_ACTIVATION_USB_COMMANDS: u32 =
    MAX_MONITORING_FIRMWARE_CURVES as u32 * APPLY_USB_COMMANDS + DISPLAY_PREFLIGHT_USB_COMMANDS;

pub const SNAPSHOT_RESPONSE_TIMEOUT: Duration = SERIAL_USB_COMMAND_BUDGET
    .saturating_mul(SNAPSHOT_USB_COMMANDS)
    .saturating_add(RESPONSE_MARGIN);
pub const APPLY_RESPONSE_TIMEOUT: Duration = SERIAL_USB_COMMAND_BUDGET
    .saturating_mul(APPLY_USB_COMMANDS)
    .saturating_add(PERSISTENCE_MARGIN_PER_TARGET)
    .saturating_add(RESPONSE_MARGIN);
pub const DISPLAY_RESPONSE_TIMEOUT: Duration = SERIAL_USB_COMMAND_BUDGET
    .saturating_mul(DISPLAY_PREFLIGHT_USB_COMMANDS)
    .saturating_add(PERSISTENCE_MARGIN_PER_TARGET)
    .saturating_add(RESPONSE_MARGIN);
/// Maximum bundle: 32 fresh list/status/write sequences, display preflight,
/// durable transitions for all targets, and independent host work.
pub const ACTIVATION_RESPONSE_TIMEOUT: Duration = SERIAL_USB_COMMAND_BUDGET
    .saturating_mul(MAX_ACTIVATION_USB_COMMANDS)
    .saturating_add(
        PERSISTENCE_MARGIN_PER_TARGET.saturating_mul(MAX_MONITORING_FIRMWARE_CURVES as u32 + 1),
    )
    .saturating_add(HOST_OPERATION_RESPONSE_TIMEOUT)
    .saturating_add(RESPONSE_MARGIN);

/// One bounded response budget per request; never a one-command shortcut for
/// writes or activation. The maximum activation allowance also covers partial
/// outcomes, so changing which targets are eligible cannot shorten a deadline.
#[must_use]
pub const fn response_timeout(request: &Request) -> Duration {
    match request {
        Request::GetSnapshot => SNAPSHOT_RESPONSE_TIMEOUT,
        Request::ApplyFirmwareCurve { .. } => APPLY_RESPONSE_TIMEOUT,
        Request::SetKrakenDisplay { .. } => DISPLAY_RESPONSE_TIMEOUT,
        Request::ActivateMonitoring { .. } => ACTIVATION_RESPONSE_TIMEOUT,
        Request::StartHostControl { .. }
        | Request::UpdateHostControl { .. }
        | Request::StopHostControl
        | Request::SetMonitoringAutoResume { .. } => HOST_OPERATION_RESPONSE_TIMEOUT,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nzxt_cam_core::{ChannelId, DeviceId, HostControlPolicy, KrakenDisplayMode};

    #[test]
    fn usb_and_response_budgets_cover_all_commands_and_margins() {
        assert_eq!(LIQUIDCTL_COMMAND_TIMEOUT, Duration::from_secs(5));
        assert!(USB_LOCK_ACQUIRE_TIMEOUT > LIQUIDCTL_COMMAND_TIMEOUT);
        assert!(!USB_LOCK_POLL_INTERVAL.is_zero());
        assert!(USB_LOCK_POLL_INTERVAL < USB_LOCK_ACQUIRE_TIMEOUT);
        let command = LIQUIDCTL_COMMAND_TIMEOUT + USB_LOCK_ACQUIRE_TIMEOUT;
        assert_eq!(SERIAL_USB_COMMAND_BUDGET, command);
        assert!(SNAPSHOT_RESPONSE_TIMEOUT > command * 2);
        assert!(APPLY_RESPONSE_TIMEOUT > command * 3 + PERSISTENCE_MARGIN_PER_TARGET);
        assert!(DISPLAY_RESPONSE_TIMEOUT > command + PERSISTENCE_MARGIN_PER_TARGET);
        assert_eq!(MAX_MONITORING_FIRMWARE_CURVES, 32);
        assert_eq!(MAX_ACTIVATION_USB_COMMANDS, 32 * 3 + 1);
        assert!(
            ACTIVATION_RESPONSE_TIMEOUT
                > command * (32 * 3 + 1)
                    + PERSISTENCE_MARGIN_PER_TARGET * 33
                    + HOST_OPERATION_RESPONSE_TIMEOUT
        );
        // Raising USB/activation budgets must not silently lengthen startup.
        assert!(CONNECT_TIMEOUT < SNAPSHOT_RESPONSE_TIMEOUT);
        assert!(HANDSHAKE_TIMEOUT < DISPLAY_RESPONSE_TIMEOUT);
        assert!(SEND_TIMEOUT < APPLY_RESPONSE_TIMEOUT);
    }

    #[test]
    fn every_request_uses_its_full_operation_budget() {
        let cases = [
            (Request::GetSnapshot, SNAPSHOT_RESPONSE_TIMEOUT),
            (
                Request::ApplyFirmwareCurve {
                    device_id: DeviceId::new("aio"),
                    channel_id: ChannelId::new("pump"),
                    points: Vec::new(),
                },
                APPLY_RESPONSE_TIMEOUT,
            ),
            (
                Request::SetKrakenDisplay {
                    device_id: DeviceId::new("aio"),
                    mode: KrakenDisplayMode::Cpu,
                },
                DISPLAY_RESPONSE_TIMEOUT,
            ),
            (
                Request::ActivateMonitoring {
                    firmware_curves: Vec::new(),
                    host_policy: None,
                    display: None,
                },
                ACTIVATION_RESPONSE_TIMEOUT,
            ),
            (
                Request::StartHostControl {
                    complete_policy: HostControlPolicy {
                        channels: Vec::new(),
                    },
                },
                HOST_OPERATION_RESPONSE_TIMEOUT,
            ),
            (
                Request::UpdateHostControl {
                    channel_policies: Vec::new(),
                },
                HOST_OPERATION_RESPONSE_TIMEOUT,
            ),
            (Request::StopHostControl, HOST_OPERATION_RESPONSE_TIMEOUT),
            (
                Request::SetMonitoringAutoResume { enabled: true },
                HOST_OPERATION_RESPONSE_TIMEOUT,
            ),
        ];
        for (request, expected) in cases {
            assert_eq!(response_timeout(&request), expected, "{request:?}");
        }
    }
}
