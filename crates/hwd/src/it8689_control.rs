//! IT8689 mode-2/PWM63 to manual/PWM128 transition and independent BIOS
//! restoration, without forcing full speed during takeover or restoration.
use crate::{
    hardware::HardwareError,
    host_control::{HostControlSysfs, fan_rpm_meets_baseline},
};

pub(crate) const ENTRY_PWM: u8 = 128;
pub(crate) const RPM_FLOOR: u64 = 700;

fn fault(message: impl Into<String>) -> HardwareError {
    HardwareError::new(message)
}

pub(crate) fn verify_mode(
    sysfs: &mut dyn HostControlSysfs,
    channel: u8,
    expected: u8,
) -> Result<(), HardwareError> {
    if sysfs.read_enable(channel)? == expected {
        Ok(())
    } else {
        Err(fault(format!("fan{channel} mode is not {expected}")))
    }
}
pub(crate) fn verify_pwm(
    sysfs: &mut dyn HostControlSysfs,
    channel: u8,
    expected: u8,
) -> Result<(), HardwareError> {
    if sysfs.read_pwm(channel)? == expected {
        Ok(())
    } else {
        Err(fault(format!("fan{channel} PWM is not {expected}")))
    }
}
pub(crate) fn rpm(
    sysfs: &mut dyn HostControlSysfs,
    channel: u8,
    baseline: Option<u64>,
) -> Result<u64, HardwareError> {
    let value = sysfs.read_fan(channel)?;
    if baseline.map_or(value >= RPM_FLOOR, |base| {
        fan_rpm_meets_baseline(base, value)
    }) {
        Ok(value)
    } else {
        Err(fault(format!("fan{channel} RPM below safety threshold")))
    }
}

/// Do not check cancellation between proven mode 1 and the immediately adjacent
/// protective PWM write; on uncertain mode write, rescue only after fresh proof.
pub(crate) fn take_manual(
    sysfs: &mut dyn HostControlSysfs,
    channel: u8,
    check: &mut impl FnMut() -> Result<(), HardwareError>,
) -> Result<(), HardwareError> {
    check()?;
    verify_mode(sysfs, channel, 2)?;
    check()?;
    verify_pwm(sysfs, channel, 63)?;
    check()?;
    let transition = sysfs
        .write_enable(channel, 1)
        .and_then(|()| verify_mode(sysfs, channel, 1));
    if let Err(error) = transition {
        if matches!(sysfs.read_enable(channel), Ok(1)) {
            let _ = sysfs
                .write_pwm(channel, ENTRY_PWM)
                .and_then(|()| verify_pwm(sysfs, channel, ENTRY_PWM));
        }
        return Err(error);
    }
    sysfs.write_pwm(channel, ENTRY_PWM)?;
    verify_pwm(sysfs, channel, ENTRY_PWM)?;
    check()?;
    Ok(())
}

/// Only fan4's mode 0 + actual PWM255 is a known manual alias. Never write 0.
pub(crate) fn manual_observed(channel: u8, mode: u8, pwm: u8) -> bool {
    mode == 1 || channel == 4 && mode == 0 && pwm == 255
}

/// `controlled=false` permits read-only verification only. A failed BIOS
/// takeover never guesses a previous duty: use the freshly observed manual PWM
/// except dormant original63, for which PWM128 is the protective duty.
pub(crate) fn restore_one(
    sysfs: &mut dyn HostControlSysfs,
    channel: u8,
    original: u8,
    controlled: bool,
) -> Result<(), HardwareError> {
    let mode = sysfs.read_enable(channel)?;
    let current = sysfs.read_pwm(channel)?;
    if !controlled && (mode != 2 || current != original) {
        return Err(fault(format!("untouched fan{channel} changed")));
    }
    if mode == 2 && current == original {
        // Already restored (or never acquired).
    } else if controlled && manual_observed(channel, mode, current) {
        if current == 0 {
            return Err(fault("cannot safely recover from unexpected PWM0"));
        }
        let mut first_error = None;
        if current != original
            && let Err(error) = sysfs
                .write_pwm(channel, original)
                .and_then(|()| verify_pwm(sysfs, channel, original))
        {
            first_error = Some(error);
        }
        // Even an uncertain original write must be followed by BIOS takeover.
        let mode_write = sysfs.write_enable(channel, 2);
        let readback = sysfs.read_enable(channel);
        if mode_write.is_err() || !matches!(readback, Ok(2)) {
            first_error.get_or_insert_with(|| {
                fault(format!(
                    "fan{channel} BIOS mode restoration failed: {mode_write:?}, {readback:?}"
                ))
            });
            if matches!(readback, Ok(1)) {
                let working = if current == original || current == 255 {
                    ENTRY_PWM
                } else {
                    current
                };
                let _ = sysfs
                    .write_pwm(channel, working)
                    .and_then(|()| verify_pwm(sysfs, channel, working));
            } else if channel == 4 && matches!(readback, Ok(0)) {
                // Only a fresh mode0/PWM255 pair proves the fan4 manual alias.
                if verify_pwm(sysfs, channel, 255).is_ok() {
                    let _ = sysfs
                        .write_pwm(channel, ENTRY_PWM)
                        .and_then(|()| verify_pwm(sysfs, channel, ENTRY_PWM));
                }
            }
            return Err(first_error.expect("failed restoration"));
        }
        let final_checks = verify_mode(sysfs, channel, 2)
            .and_then(|()| verify_pwm(sysfs, channel, original))
            .and_then(|()| rpm(sysfs, channel, None).map(|_| ()));
        return match first_error {
            Some(error) => Err(error),
            None => final_checks,
        };
    } else if mode == 2 {
        return Err(fault(format!(
            "fan{channel} BIOS PWM differs from saved original"
        )));
    } else {
        return Err(fault(format!(
            "fan{channel} mode is unknown; no speculative writes"
        )));
    }
    verify_mode(sysfs, channel, 2)?;
    verify_pwm(sysfs, channel, original)?;
    rpm(sysfs, channel, None)?;
    Ok(())
}
