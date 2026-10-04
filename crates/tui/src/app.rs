use std::collections::{BTreeMap, BTreeSet};

use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use nzxt_cam_core::{
    ChannelId, CoolingChannel, CurvePoint, CurveState, Device, DeviceId, HardwareSnapshot,
    HostChannelCapability, HostChannelPolicy, HostControlPolicy, HostControlState, HostCurve,
    HostCurvePoint, HostTemperatureSource, KrakenDisplayMode, ReadingKind, TemperatureSource,
};
use nzxt_cam_protocol::{
    MAX_MONITORING_FIRMWARE_CURVES, MONITORING_FIRMWARE_CURVE_POINTS, MonitoringActivationOutcome,
    MonitoringActivationStatus, MonitoringActivationTarget, MonitoringDisplaySelection,
    MonitoringFirmwareCurve,
};

use crate::{
    config::AppConfig,
    profile::{
        BuiltinProfile, CurveProfile, HostCurveProfile, MAX_PROFILE_NAME_CHARS, ProfileBinding,
        ProfileCurve, ProfileIntent, ProfileLibrary, ProfileRef, ProfileTarget,
        built_in_host_profiles, built_in_profiles,
    },
};

pub type CurveKey = (DeviceId, ChannelId);

const HOST_POINT_COUNT: usize = 40;
const HOST_FIRST_TEMPERATURE_MILLIDEGREES: i32 = 22_000;
const HOST_TEMPERATURE_STEP_MILLIDEGREES: i32 = 2_000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EditorMode {
    #[default]
    Firmware,
    Host,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HostEditorChannel {
    pub capability: HostChannelCapability,
    pub curve: HostCurve,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HostPolicyEditor {
    pub channels: Vec<HostEditorChannel>,
    pub selected_channel: usize,
    pub selected_point: usize,
    baseline: HostControlPolicy,
}

impl HostPolicyEditor {
    fn new(capabilities: &[HostChannelCapability]) -> Self {
        let mut seen = BTreeSet::new();
        let channels = capabilities
            .iter()
            .filter(|capability| seen.insert(capability.channel_id.clone()))
            .cloned()
            .map(|capability| HostEditorChannel {
                curve: conservative_host_curve(capability.minimum_duty_percent),
                capability,
            })
            .collect::<Vec<_>>();
        let baseline = policy_from_host_channels(&channels);
        Self {
            channels,
            selected_channel: 0,
            selected_point: 0,
            baseline,
        }
    }

    pub fn active_channel(&self) -> Option<&HostEditorChannel> {
        self.channels.get(self.selected_channel)
    }

    pub fn selected_point(&self) -> Option<HostCurvePoint> {
        self.active_channel()?
            .curve
            .points
            .get(self.selected_point)
            .copied()
    }

    pub fn policy(&self) -> HostControlPolicy {
        policy_from_host_channels(&self.channels)
    }

    pub fn is_dirty(&self) -> bool {
        self.policy() != self.baseline
    }

    fn reconcile(&mut self, capabilities: &[HostChannelCapability]) {
        let selected_id = self
            .active_channel()
            .map(|channel| channel.capability.channel_id.clone());
        let mut old = std::mem::take(&mut self.channels)
            .into_iter()
            .map(|channel| (channel.capability.channel_id.clone(), channel))
            .collect::<BTreeMap<_, _>>();
        let mut seen = BTreeSet::new();
        self.channels = capabilities
            .iter()
            .filter(|capability| seen.insert(capability.channel_id.clone()))
            .cloned()
            .map(|capability| {
                if let Some(mut channel) = old.remove(&capability.channel_id) {
                    channel.capability = capability;
                    normalize_host_curve(
                        &mut channel.curve,
                        channel.capability.minimum_duty_percent,
                    );
                    channel
                } else {
                    HostEditorChannel {
                        curve: conservative_host_curve(capability.minimum_duty_percent),
                        capability,
                    }
                }
            })
            .collect();
        self.selected_channel = selected_id
            .and_then(|id| {
                self.channels
                    .iter()
                    .position(|channel| channel.capability.channel_id == id)
            })
            .unwrap_or_else(|| {
                self.selected_channel
                    .min(self.channels.len().saturating_sub(1))
            });
        self.selected_point = self.selected_point.min(
            self.active_channel()
                .map_or(0, |channel| channel.curve.points.len())
                .saturating_sub(1),
        );
        reconcile_host_baseline(&mut self.baseline, &self.channels);
    }

    fn adopt_running(&mut self, policy: &HostControlPolicy) {
        for channel in &mut self.channels {
            let id = &channel.capability.channel_id;
            let Some(actual) = policy.channels.iter().find(|entry| &entry.channel_id == id) else {
                continue;
            };
            let old = self
                .baseline
                .channels
                .iter_mut()
                .find(|entry| &entry.channel_id == id);
            let dirty = old
                .as_ref()
                .is_some_and(|entry| channel.curve != entry.curve);
            if !dirty {
                channel.curve = actual.curve.clone();
            }
            if let Some(entry) = old {
                entry.curve = actual.curve.clone();
            } else {
                self.baseline.channels.push(actual.clone());
            }
        }
    }

    fn record_applied(&mut self, policy: HostControlPolicy) {
        self.adopt_running(&policy);
    }

    fn record_channel_applied(&mut self, accepted: &HostChannelPolicy) {
        if let Some(baseline) = self
            .baseline
            .channels
            .iter_mut()
            .find(|entry| entry.channel_id == accepted.channel_id)
        {
            baseline.curve = accepted.curve.clone();
        }
    }

    pub fn selected_is_dirty(&self) -> bool {
        self.active_channel().is_some_and(|channel| {
            self.baseline
                .channels
                .iter()
                .find(|entry| entry.channel_id == channel.capability.channel_id)
                .is_none_or(|entry| entry.curve != channel.curve)
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AppCommand {
    ApplyCurve {
        key: CurveKey,
        points: Vec<CurvePoint>,
        profile: Option<ProfileRef>,
    },
    RenameProfile {
        id: u64,
        name: String,
    },
    RememberHostProfiles {
        channel_policies: Vec<HostChannelPolicy>,
    },
    StartHostControl {
        policy: HostControlPolicy,
    },
    UpdateHostControl {
        channel_policies: Vec<HostChannelPolicy>,
    },
    StopHostControl,
    ActivateMonitoring {
        firmware_curves: Vec<MonitoringFirmwareCurve>,
        host_policy: Option<HostControlPolicy>,
        display: Option<MonitoringDisplaySelection>,
    },
    SetMonitoringAutoResume {
        enabled: bool,
    },
    SaveConfirmApply {
        enabled: bool,
    },
    SetKrakenDisplay {
        device_id: DeviceId,
        mode: KrakenDisplayMode,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modal {
    Help,
    Profiles,
    HostProfiles,
    HostApplyScope,
    RenameProfile,
    ConfirmApply,
    KrakenDisplay,
    Settings,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HostApplyScope {
    pub selected: HostChannelCapability,
    pub curve: HostCurve,
    targets: Vec<HostChannelCapability>,
    active_policy: Option<HostControlPolicy>,
    pub all_groups: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StatusKind {
    Info,
    Success,
    Error,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StatusMessage {
    pub kind: StatusKind,
    pub text: String,
}

impl StatusMessage {
    fn info(text: impl Into<String>) -> Self {
        Self {
            kind: StatusKind::Info,
            text: text.into(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct FirmwareConfirmation {
    command: AppCommand,
    key: CurveKey,
    points: Vec<CurvePoint>,
    min_duty: u8,
    max_duty: u8,
    source: TemperatureSource,
}

#[derive(Debug)]
pub struct App {
    pub snapshot: HardwareSnapshot,
    pub backend_name: String,
    pub selected_cooling_device: usize,
    pub selected_channel: usize,
    pub selected_point: usize,
    pub editor_mode: EditorMode,
    pub host_editor: HostPolicyEditor,
    pub host_control_state: HostControlState,
    pub host_status_trusted: bool,
    pub modal: Option<Modal>,
    pub profile_cursor: usize,
    pub profile_name_input: String,
    pub profiles: Vec<CurveProfile>,
    pub exit: bool,
    pub status: StatusMessage,
    applied_curves: BTreeMap<CurveKey, Vec<CurvePoint>>,
    dirty_curves: BTreeSet<CurveKey>,
    active_profiles: BTreeMap<CurveKey, ProfileRef>,
    host_active_profiles: BTreeMap<ChannelId, ProfileRef>,
    host_identity_curves: BTreeMap<ChannelId, HostCurve>,
    profile_library: ProfileLibrary,
    rename_target: Option<(EditorMode, ProfileRef, Option<ChannelId>, Option<CurveKey>)>,
    pub confirm_apply: bool,
    pub snapshot_received: bool,
    pub activation_busy: bool,
    pub activation_attempted: bool,
    pub auto_resume_busy: bool,
    pending_confirmation: Option<FirmwareConfirmation>,
    host_profile_target: Option<ChannelId>,
    pub host_apply_scope: Option<HostApplyScope>,
    pub host_operation_busy: bool,
    pub display_operation_busy: bool,
    pub display_cursor: usize,
}

impl App {
    #[cfg(test)]
    pub fn new(snapshot: HardwareSnapshot, backend_name: impl Into<String>) -> Self {
        Self::with_custom_profiles(snapshot, backend_name, Vec::new())
    }

    #[cfg(test)]
    pub fn with_custom_profiles(
        snapshot: HardwareSnapshot,
        backend_name: impl Into<String>,
        custom_profiles: Vec<CurveProfile>,
    ) -> Self {
        Self::with_runtime_config(
            snapshot,
            backend_name,
            custom_profiles,
            AppConfig {
                confirm_apply: false,
                default_profiles: Vec::new(),
            },
        )
    }

    #[cfg(test)]
    pub fn with_runtime_config(
        snapshot: HardwareSnapshot,
        backend_name: impl Into<String>,
        custom_profiles: Vec<CurveProfile>,
        mut config: AppConfig,
    ) -> Self {
        let backend_name = backend_name.into();
        if backend_name == "DEMO" {
            config.default_profiles.clear();
        }
        let library = ProfileLibrary::from_legacy(&custom_profiles, &config.default_profiles)
            .expect("valid legacy profile fixtures");
        Self::with_profile_library(snapshot, backend_name, library, config)
    }

    pub fn with_profile_library(
        mut snapshot: HardwareSnapshot,
        backend_name: impl Into<String>,
        library: ProfileLibrary,
        config: AppConfig,
    ) -> Self {
        let backend_name = backend_name.into();
        let confirm_apply = config.confirm_apply;
        let mut profiles = built_in_profiles();
        profiles.extend(library.firmware_profiles());
        let mut active_profiles = BTreeMap::new();
        for device in &mut snapshot.devices {
            for channel in &mut device.cooling_channels {
                let key = (device.id.clone(), channel.id.clone());
                if channel.curve_state == CurveState::Unverified
                    && let Some(binding) = library.bindings.iter().find(|binding| {
                        binding.target
                            == ProfileTarget::Firmware {
                                device_id: key.0.clone(),
                                channel_id: key.1.clone(),
                            }
                    })
                    && let ProfileCurve::Firmware(points) = &binding.curve
                {
                    channel.points = points.clone();
                    clamp_points(&mut channel.points, channel.min_duty, channel.max_duty);
                    active_profiles.insert(key, binding.profile);
                    continue;
                }
                if let Some((index, _)) = profiles
                    .iter()
                    .enumerate()
                    .find(|(_, profile)| profile_matches_channel(profile, channel))
                {
                    active_profiles.insert(key, profile_ref_for_index(&library, index));
                }
            }
        }
        let mut host_editor = HostPolicyEditor::new(&snapshot.host_control.channels);
        let mut host_active_profiles = BTreeMap::new();
        let mut host_identity_curves = BTreeMap::new();
        for channel in &mut host_editor.channels {
            if let Some(binding) = library.bindings.iter().find(|binding| {
                binding.target
                    == ProfileTarget::Host {
                        channel_id: channel.capability.channel_id.clone(),
                    }
            }) && let ProfileCurve::Host(curve) = &binding.curve
            {
                channel.curve = curve.clone();
                normalize_host_curve(&mut channel.curve, channel.capability.minimum_duty_percent);
                host_active_profiles.insert(channel.capability.channel_id.clone(), binding.profile);
                host_identity_curves
                    .insert(channel.capability.channel_id.clone(), channel.curve.clone());
            }
        }
        host_editor.baseline = host_editor.policy();
        if let Some(policy) = &snapshot.host_control.active_policy {
            host_editor.record_applied(policy.clone());
            for channel in &host_editor.channels {
                if !library.bindings.iter().any(|b| {
                    b.target
                        == ProfileTarget::Host {
                            channel_id: channel.capability.channel_id.clone(),
                        }
                        && b.curve == ProfileCurve::Host(channel.curve.clone())
                }) {
                    host_active_profiles.remove(&channel.capability.channel_id);
                    host_identity_curves.remove(&channel.capability.channel_id);
                }
            }
        }
        let initial_host_error = snapshot.host_control.last_error.clone();
        let host_control_state = snapshot.host_control.state;
        let host_status_trusted = backend_name == "DEMO"
            || !snapshot.devices.is_empty()
            || !snapshot.host_control.channels.is_empty();
        let applied_curves = snapshot
            .devices
            .iter()
            .flat_map(|device| {
                device.cooling_channels.iter().map(|channel| {
                    (
                        (device.id.clone(), channel.id.clone()),
                        channel.points.clone(),
                    )
                })
            })
            .collect();
        Self {
            snapshot,
            backend_name: backend_name.clone(),
            selected_cooling_device: 0,
            selected_channel: 0,
            selected_point: 0,
            editor_mode: EditorMode::Firmware,
            host_editor,
            host_control_state,
            host_status_trusted,
            modal: None,
            profile_cursor: 1,
            profile_name_input: String::new(),
            profiles,
            exit: false,
            status: if let Some(error) = initial_host_error {
                StatusMessage {
                    kind: StatusKind::Error,
                    text: format!("Fan control: {error}"),
                }
            } else if backend_name == "DEMO" {
                StatusMessage::info("Demo mode active — hardware writes disabled")
            } else {
                StatusMessage::info("Connecting to monitoring service…")
            },
            applied_curves,
            dirty_curves: BTreeSet::new(),
            active_profiles,
            host_active_profiles,
            host_identity_curves,
            profile_library: library,
            rename_target: None,
            confirm_apply,
            snapshot_received: backend_name == "DEMO",
            activation_busy: false,
            activation_attempted: false,
            auto_resume_busy: false,
            pending_confirmation: None,
            host_profile_target: None,
            host_apply_scope: None,
            host_operation_busy: false,
            display_operation_busy: false,
            display_cursor: 0,
        }
    }

    pub fn set_profile_library(&mut self, library: ProfileLibrary) {
        let cursor_ref = match self.modal {
            Some(Modal::Profiles) => self.firmware_ref(self.profile_cursor),
            Some(Modal::HostProfiles) => self.host_ref(self.profile_cursor),
            _ => None,
        };
        self.profiles = built_in_profiles();
        self.profiles.extend(library.firmware_profiles());
        self.profile_library = library;
        if let Some(reference) = cursor_ref
            && let Some(index) = match self.modal {
                Some(Modal::Profiles) => {
                    (0..self.profiles.len()).find(|&i| self.firmware_ref(i) == Some(reference))
                }
                Some(Modal::HostProfiles) => {
                    (0..self.host_profiles().len()).find(|&i| self.host_ref(i) == Some(reference))
                }
                _ => None,
            }
        {
            self.profile_cursor = index;
        }
    }

    pub fn profile_bindings_applied(&mut self, bindings: &[ProfileBinding]) {
        for binding in bindings {
            match (&binding.target, &binding.curve) {
                (
                    ProfileTarget::Firmware {
                        device_id,
                        channel_id,
                    },
                    ProfileCurve::Firmware(points),
                ) => {
                    let key = (device_id.clone(), channel_id.clone());
                    if self.find_channel(&key).is_some_and(|c| c.points == *points)
                        && !self.active_profiles.get(&key).is_some_and(|old| {
                            *old != binding.profile
                                && self.firmware_reference_matches(
                                    *old,
                                    self.find_channel(&key).unwrap(),
                                )
                        })
                    {
                        self.active_profiles.insert(key, binding.profile);
                    }
                }
                (ProfileTarget::Host { channel_id }, ProfileCurve::Host(curve))
                    if self
                        .host_editor
                        .channels
                        .iter()
                        .any(|c| &c.capability.channel_id == channel_id && c.curve == *curve)
                        && !self
                            .host_active_profiles
                            .get(channel_id)
                            .is_some_and(|old| {
                                *old != binding.profile
                                    && self
                                        .host_editor
                                        .channels
                                        .iter()
                                        .find(|c| &c.capability.channel_id == channel_id)
                                        .is_some_and(|c| self.host_reference_matches(*old, c))
                            }) =>
                {
                    self.host_active_profiles
                        .insert(channel_id.clone(), binding.profile);
                    self.host_identity_curves
                        .insert(channel_id.clone(), curve.clone());
                }
                _ => {}
            }
        }
    }

    pub fn host_profile_intents(&self, policies: &[HostChannelPolicy]) -> Vec<ProfileIntent> {
        policies
            .iter()
            .map(|policy| {
                let preferred = self
                    .host_editor
                    .channels
                    .iter()
                    .find(|channel| channel.capability.channel_id == policy.channel_id)
                    .filter(|channel| channel.curve == policy.curve)
                    .and_then(|channel| {
                        self.host_active_profiles
                            .get(&policy.channel_id)
                            .copied()
                            .filter(|reference| self.host_reference_matches(*reference, channel))
                    });
                ProfileIntent {
                    target: ProfileTarget::Host {
                        channel_id: policy.channel_id.clone(),
                    },
                    curve: ProfileCurve::Host(policy.curve.clone()),
                    preferred,
                }
            })
            .collect()
    }

    pub fn rename_succeeded(&mut self, id: u64) {
        if let Some((mode, ProfileRef::Custom(target), channel, _)) = self.rename_target.clone()
            && id == target
        {
            self.profile_cursor = match mode {
                EditorMode::Firmware => (0..self.profiles.len())
                    .find(|&i| self.firmware_ref(i) == Some(ProfileRef::Custom(id))),
                EditorMode::Host => (0..self.host_profiles().len())
                    .find(|&i| self.host_ref(i) == Some(ProfileRef::Custom(id))),
            }
            .unwrap_or(self.profile_cursor);
            self.modal = Some(match mode {
                EditorMode::Firmware => Modal::Profiles,
                EditorMode::Host => Modal::HostProfiles,
            });
            self.host_profile_target = channel;
            self.profile_name_input.clear();
            self.rename_target = None;
            self.status = StatusMessage::info("Profile renamed");
        }
    }

    fn firmware_ref(&self, index: usize) -> Option<ProfileRef> {
        self.profiles.get(index)?;
        Some(profile_ref_for_index(&self.profile_library, index))
    }

    fn host_ref(&self, index: usize) -> Option<ProfileRef> {
        if index < 3 {
            return Some(ProfileRef::BuiltIn(builtin_for_index(index)));
        }
        self.host_profiles().get(index)?;
        self.profile_library
            .profiles
            .iter()
            .filter(|p| matches!(p.curve, ProfileCurve::Host(_)))
            .nth(index - 3)
            .map(|p| ProfileRef::Custom(p.id))
    }

    fn firmware_reference_matches(&self, reference: ProfileRef, channel: &CoolingChannel) -> bool {
        self.profile_library.name(reference).is_some() &&
        match reference {
            ProfileRef::BuiltIn(_) => self.profiles.iter().enumerate().find(|(i, _)| self.firmware_ref(*i) == Some(reference))
                .is_some_and(|(_, profile)| profile_matches_channel(profile, channel)),
            ProfileRef::Custom(id) => self.profile_library.profiles.iter().find(|p| p.id == id)
                .is_some_and(|p| matches!(&p.curve, ProfileCurve::Firmware(points) if points_match_channel(points, channel))),
        }
    }

    fn host_reference_matches(&self, reference: ProfileRef, channel: &HostEditorChannel) -> bool {
        if self
            .host_identity_curves
            .get(&channel.capability.channel_id)
            .is_some_and(|original| original != &channel.curve)
        {
            return false;
        }
        match reference {
            ProfileRef::BuiltIn(builtin) => built_in_host_profiles(channel.curve.source, channel.capability.minimum_duty_percent)
                .get(builtin_index(builtin)).is_some_and(|p| p.curve == channel.curve),
            ProfileRef::Custom(id) => self.profile_library.profiles.iter().find(|p| p.id == id)
                .is_some_and(|p| matches!(&p.curve, ProfileCurve::Host(curve) if {
                    let mut normalized = curve.clone();
                    normalize_host_curve(&mut normalized, channel.capability.minimum_duty_percent);
                    normalized == channel.curve
                })),
        }
    }

    pub fn cooling_device_indices(&self) -> Vec<usize> {
        self.snapshot
            .devices
            .iter()
            .enumerate()
            .filter_map(|(index, device)| (!device.cooling_channels.is_empty()).then_some(index))
            .collect()
    }

    pub fn active_device(&self) -> Option<&Device> {
        let index = *self
            .cooling_device_indices()
            .get(self.selected_cooling_device)?;
        self.snapshot.devices.get(index)
    }

    pub fn active_channel(&self) -> Option<&CoolingChannel> {
        self.active_device()?
            .cooling_channels
            .get(self.selected_channel)
    }

    pub fn active_curve_key(&self) -> Option<CurveKey> {
        Some((
            self.active_device()?.id.clone(),
            self.active_channel()?.id.clone(),
        ))
    }

    pub fn is_active_curve_dirty(&self) -> bool {
        self.active_curve_key()
            .is_some_and(|key| self.dirty_curves.contains(&key))
    }

    pub fn active_curve_is_verified(&self) -> bool {
        self.active_channel()
            .is_some_and(|channel| channel.curve_state == CurveState::Applied)
    }

    #[cfg(test)]
    pub fn dirty_curve_count(&self) -> usize {
        self.dirty_curves.len()
    }

    pub fn selected_curve_point(&self) -> Option<CurvePoint> {
        self.active_channel()?
            .points
            .get(self.selected_point)
            .copied()
    }

    /// Name of the exact selected working curve, never a stale applied claim.
    pub fn active_profile_name(&self) -> &str {
        let Some(channel) = self.active_channel() else {
            return "Custom";
        };
        self.active_curve_key()
            .and_then(|key| self.active_profiles.get(&key).copied())
            .filter(|reference| self.firmware_reference_matches(*reference, channel))
            .and_then(|reference| self.profile_library.name(reference))
            .unwrap_or("Custom")
    }

    pub fn active_profile_is_modified(&self) -> bool {
        self.active_channel()
            .zip(self.active_curve_key())
            .is_some_and(|(channel, key)| {
                self.active_profiles
                    .get(&key)
                    .is_some_and(|reference| !self.firmware_reference_matches(*reference, channel))
            })
    }

    pub fn selected_profile(&self) -> Option<&CurveProfile> {
        self.profiles.get(self.profile_cursor)
    }

    pub fn host_policy_is_dirty(&self) -> bool {
        self.host_editor.is_dirty()
    }

    pub fn selected_host_channel(&self) -> Option<&HostEditorChannel> {
        self.host_editor.active_channel()
    }

    pub fn selected_host_point(&self) -> Option<HostCurvePoint> {
        self.host_editor.selected_point()
    }

    pub fn host_source_temperature_celsius(&self) -> Option<f64> {
        let source = self.selected_host_channel()?.curve.source;
        let cpu = self.host_temperature_reading("CPU Tctl");
        let gpu = self.host_temperature_reading("NVIDIA GPU");
        match source {
            HostTemperatureSource::Cpu => cpu,
            HostTemperatureSource::Gpu => gpu,
            HostTemperatureSource::CpuGpuMax => Some(cpu?.max(gpu?)),
        }
    }

    /// Curve target at the displayed sensor temperature, not applied/read-back PWM.
    pub fn host_target_duty(&self) -> Option<u8> {
        let channel = self.selected_host_channel()?;
        let temperature = self.host_source_temperature_celsius()?;
        Some(evaluate_host_curve(
            &channel.curve,
            (temperature * 1_000.0).round() as i32,
        ))
    }

    fn host_temperature_reading(&self, label: &str) -> Option<f64> {
        self.snapshot
            .devices
            .iter()
            .flat_map(|device| &device.readings)
            .find(|reading| reading.kind == ReadingKind::Temperature && reading.label == label)
            .map(|reading| reading.value)
            .filter(|value| value.is_finite())
    }

    pub fn handle_key_event(&mut self, key_event: KeyEvent) -> Option<AppCommand> {
        if key_event.modifiers.contains(KeyModifiers::CONTROL)
            && matches!(key_event.code, KeyCode::Char('c' | 'C'))
        {
            self.exit = true;
            return None;
        }

        if self.needs_opt_in() {
            if self.modal == Some(Modal::Settings) {
                return self.handle_settings_key(key_event);
            }
            if self.modal == Some(Modal::Help) {
                if matches!(key_event.code, KeyCode::Esc | KeyCode::Char('?')) {
                    self.modal = None;
                }
                if matches!(key_event.code, KeyCode::Char('q' | 'Q')) {
                    self.exit = true;
                }
                return None;
            }
            match key_event.code {
                KeyCode::Enter if self.modal.is_none() => return self.activate_monitoring(),
                KeyCode::Char('o' | 'O') if self.modal.is_none() => {
                    self.modal = Some(Modal::Settings)
                }
                KeyCode::Char('?') if self.modal.is_none() => self.modal = Some(Modal::Help),
                KeyCode::Char('q' | 'Q') | KeyCode::Esc if self.modal.is_none() => self.exit = true,
                _ => {}
            }
            return None;
        }
        if self.modal == Some(Modal::Settings) {
            return self.handle_settings_key(key_event);
        }
        let library_navigation = matches!(self.modal, Some(Modal::Profiles | Modal::HostProfiles))
            && matches!(
                key_event.code,
                KeyCode::Up | KeyCode::Down | KeyCode::Char('r' | 'R')
            );
        if !self.host_status_trusted
            && !library_navigation
            && self.modal != Some(Modal::HostApplyScope)
            && self.modal != Some(Modal::RenameProfile)
            && !matches!(
                key_event.code,
                KeyCode::Char('q' | 'Q' | '?' | 'n' | 'N' | 'p' | 'P')
                    | KeyCode::Esc
                    | KeyCode::Left
                    | KeyCode::Right
                    | KeyCode::Home
                    | KeyCode::End
                    | KeyCode::Tab
                    | KeyCode::BackTab
                    | KeyCode::Char('c' | 'C' | '[' | ']' | 'd' | 'D' | 'o' | 'O')
            )
        {
            self.apply_failed("Host-control STATUS UNKNOWN — wait for a fresh service snapshot");
            return None;
        }
        if let Some(modal) = self.modal {
            return match modal {
                Modal::Help => {
                    match key_event.code {
                        KeyCode::Esc | KeyCode::Char('?') => self.modal = None,
                        KeyCode::Char('q' | 'Q') => self.exit = true,
                        _ => {}
                    }
                    None
                }
                Modal::Profiles => self.handle_profile_picker_key(key_event),
                Modal::HostProfiles => self.handle_host_profile_picker_key(key_event),
                Modal::HostApplyScope => self.handle_host_apply_scope_key(key_event),
                Modal::RenameProfile => self.handle_profile_name_key(key_event),
                Modal::ConfirmApply => self.handle_apply_confirmation_key(key_event),
                Modal::KrakenDisplay => self.handle_display_picker_key(key_event),
                Modal::Settings => self.handle_settings_key(key_event),
            };
        }

        match key_event.code {
            KeyCode::Char('q' | 'Q') | KeyCode::Esc => self.exit = true,
            KeyCode::Char('?') => self.modal = Some(Modal::Help),
            KeyCode::Char('d' | 'D') => self.open_display_picker(),
            KeyCode::Char('o' | 'O') => self.modal = Some(Modal::Settings),
            KeyCode::Tab => self.select_next_control(),
            KeyCode::BackTab => self.select_previous_control(),
            _ if self.editor_mode == EditorMode::Host => {
                return self.handle_host_editor_key(key_event);
            }
            KeyCode::Char('p' | 'P') => self.open_profile_picker(),
            KeyCode::Char('c' | 'C') | KeyCode::Char(']') => self.select_next_channel(),
            KeyCode::Char('[') => self.select_previous_channel(),
            KeyCode::Left => self.select_previous_point(),
            KeyCode::Right => self.select_next_point(),
            KeyCode::Home => self.select_first_point(),
            KeyCode::End => self.select_last_point(),
            KeyCode::Up => {
                let step = if key_event.modifiers.contains(KeyModifiers::SHIFT) {
                    5
                } else {
                    1
                };
                self.change_selected_duty(step);
            }
            KeyCode::Down => {
                let step = if key_event.modifiers.contains(KeyModifiers::SHIFT) {
                    -5
                } else {
                    -1
                };
                self.change_selected_duty(step);
            }
            KeyCode::Char('r' | 'R') => self.reset_active_curve(),
            KeyCode::Enter => return self.apply_command(),
            _ => {}
        }

        None
    }

    pub fn needs_opt_in(&self) -> bool {
        self.backend_name != "DEMO" && !self.snapshot.monitoring.opted_in
    }

    fn activate_monitoring(&mut self) -> Option<AppCommand> {
        if !self.snapshot_received || !self.host_status_trusted {
            self.apply_failed("Wait for a fresh service snapshot before activating monitoring");
            return None;
        }
        if self.activation_busy || self.activation_attempted {
            self.status = StatusMessage::info(
                "Activation already requested; review service status before retrying",
            );
            return None;
        }
        let firmware_curves = self
            .snapshot
            .devices
            .iter()
            .filter(|d| d.online && d.kind == nzxt_cam_core::DeviceKind::LiquidCooler)
            .flat_map(|d| {
                d.cooling_channels
                    .iter()
                    .filter(|c| {
                        c.points.len() == MONITORING_FIRMWARE_CURVE_POINTS
                            && c.points.iter().enumerate().all(|(index, point)| {
                                point.temperature == 20 + index as u8
                                    && (c.min_duty..=c.max_duty).contains(&point.duty)
                            })
                            && c.points.last().is_some_and(|point| point.duty == 100)
                    })
                    .map(move |c| MonitoringFirmwareCurve {
                        device_id: d.id.clone(),
                        channel_id: c.id.clone(),
                        points: c.points.clone(),
                    })
            })
            .take(MAX_MONITORING_FIRMWARE_CURVES + 1)
            .collect::<Vec<_>>();
        if firmware_curves.len() > MAX_MONITORING_FIRMWARE_CURVES {
            self.apply_failed("Too many AIO curves for a single activation; no request sent");
            return None;
        }
        let host_policy = (self.host_control_state == HostControlState::Available
            && !self.host_editor.channels.is_empty()
            && self.host_editor.channels.len() == self.snapshot.host_control.channels.len())
        .then(|| self.host_editor.policy())
        .filter(|policy| {
            policy.channels.iter().all(|channel| {
                let minimum = self
                    .host_editor
                    .channels
                    .iter()
                    .find(|entry| entry.capability.channel_id == channel.channel_id)
                    .map(|entry| entry.capability.minimum_duty_percent);
                let Some(minimum) = minimum else { return false };
                let points = &channel.curve.points;
                (2..=64).contains(&points.len())
                    && points.last().is_some_and(|point| point.duty_percent == 100)
                    && points.iter().all(|point| {
                        (0..=120_000).contains(&point.temperature_millidegrees)
                            && (minimum..=100).contains(&point.duty_percent)
                    })
                    && points.windows(2).all(|pair| {
                        pair[0].temperature_millidegrees < pair[1].temperature_millidegrees
                            && pair[0].duty_percent <= pair[1].duty_percent
                    })
            })
        });
        let display = self
            .snapshot
            .kraken_display
            .device_id
            .clone()
            .map(|device_id| MonitoringDisplaySelection {
                device_id,
                mode: self.snapshot.kraken_display.mode,
            });
        if firmware_curves.is_empty() && host_policy.is_none() && display.is_none() {
            self.apply_failed(
                "No available AIO curves, motherboard fan control, or Kraken display to activate",
            );
            return None;
        }
        self.activation_attempted = true;
        self.status = StatusMessage::info("Requesting monitoring activation…");
        Some(AppCommand::ActivateMonitoring {
            firmware_curves,
            host_policy,
            display,
        })
    }

    pub fn activation_result(
        &mut self,
        result: Result<Vec<MonitoringActivationOutcome>, crate::backend::BackendError>,
    ) {
        self.activation_busy = false;
        match result {
            Ok(outcomes) => {
                for outcome in &outcomes {
                    if outcome.status == MonitoringActivationStatus::Unknown
                        && let MonitoringActivationTarget::FirmwareCurve {
                            device_id,
                            channel_id,
                        } = &outcome.target
                        && let Some(channel) =
                            self.find_channel_mut(&(device_id.clone(), channel_id.clone()))
                    {
                        channel.curve_state = CurveState::Unverified;
                    }
                }
                let applied = outcomes
                    .iter()
                    .filter(|o| o.status == MonitoringActivationStatus::Applied)
                    .count();
                let pending = outcomes
                    .iter()
                    .filter(|o| o.status == MonitoringActivationStatus::Pending)
                    .count();
                let not_applied = outcomes.len() - applied - pending;
                let details = outcomes
                    .iter()
                    .filter(|o| o.status != MonitoringActivationStatus::Applied)
                    .map(|o| {
                        let target = match &o.target {
                            MonitoringActivationTarget::FirmwareCurve { channel_id, .. } => {
                                format!("AIO {}", channel_id.0)
                            }
                            MonitoringActivationTarget::HostControl {} => "motherboard".into(),
                            MonitoringActivationTarget::Display { .. } => "display".into(),
                        };
                        let state = match o.status {
                            MonitoringActivationStatus::Pending => "queued, not uploaded",
                            MonitoringActivationStatus::Skipped => "skipped",
                            MonitoringActivationStatus::Failed => "failed",
                            MonitoringActivationStatus::Unknown => "outcome unknown",
                            MonitoringActivationStatus::Applied => unreachable!(),
                        };
                        format!(
                            "{target}: {state}{}",
                            o.error
                                .as_ref()
                                .map_or(String::new(), |e| format!(" ({e})"))
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("; ");
                self.status = StatusMessage {
                    kind: if applied > 0 && pending == 0 && not_applied == 0 {
                        StatusKind::Success
                    } else if not_applied == 0 && pending > 0 {
                        StatusKind::Info
                    } else {
                        StatusKind::Error
                    },
                    text: format!(
                        "Activation: {applied} applied, {pending} queued (not uploaded), {not_applied} not applied{}{}; awaiting service snapshot",
                        if details.is_empty() { "" } else { ": " },
                        details
                    ),
                };
            }
            Err(error) => {
                if !error.write_outcome_unknown() {
                    // A definite failure did not leave an unknown request in
                    // flight. Permit a new explicit Enter only after refresh.
                    self.activation_attempted = false;
                    self.snapshot_received = false;
                }
                self.apply_failed(format!(
                    "Activation outcome uncertain or failed: {error}; check service status before retrying"
                ));
            }
        }
    }

    fn handle_settings_key(&mut self, key: KeyEvent) -> Option<AppCommand> {
        match key.code {
            KeyCode::Esc | KeyCode::Char('o' | 'O') => self.modal = None,
            KeyCode::Char('q' | 'Q') => self.exit = true,
            KeyCode::Char('c' | 'C') => {
                return Some(AppCommand::SaveConfirmApply {
                    enabled: !self.confirm_apply,
                });
            }
            KeyCode::Char('a' | 'A')
                if !self.auto_resume_busy
                    && self.snapshot_received
                    && self.host_status_trusted
                    && !self.needs_opt_in()
                    && self.backend_name != "DEMO" =>
            {
                return Some(AppCommand::SetMonitoringAutoResume {
                    enabled: !self.snapshot.monitoring.auto_resume,
                });
            }
            KeyCode::Char('t' | 'T')
                if self.snapshot_received || (self.host_status_trusted && !self.needs_opt_in()) =>
            {
                return self.host_stop_command();
            }
            _ => {}
        }
        None
    }

    fn open_display_picker(&mut self) {
        if !self.host_status_trusted || self.snapshot.kraken_display.device_id.is_none() {
            self.apply_failed("No available Kraken 2023 display; refresh the service snapshot");
            return;
        }
        self.display_cursor = KrakenDisplayMode::ALL
            .iter()
            .position(|mode| *mode == self.snapshot.kraken_display.mode)
            .unwrap_or(0);
        self.modal = Some(Modal::KrakenDisplay);
    }

    fn handle_display_picker_key(&mut self, key: KeyEvent) -> Option<AppCommand> {
        let count = KrakenDisplayMode::ALL.len();
        match key.code {
            KeyCode::Esc | KeyCode::Char('d' | 'D') => self.modal = None,
            KeyCode::Char('q' | 'Q') => self.exit = true,
            KeyCode::Up | KeyCode::Left => {
                self.display_cursor = (self.display_cursor + count - 1) % count
            }
            KeyCode::Down | KeyCode::Right => {
                self.display_cursor = (self.display_cursor + 1) % count
            }
            KeyCode::Home => self.display_cursor = 0,
            KeyCode::End => self.display_cursor = count - 1,
            KeyCode::Enter => {
                if self.display_operation_busy {
                    self.apply_failed("A display change is already in progress");
                    return None;
                }
                let Some(device_id) = self.snapshot.kraken_display.device_id.clone() else {
                    self.modal = None;
                    self.apply_failed("Kraken display is no longer available");
                    return None;
                };
                if !self.host_status_trusted {
                    self.modal = None;
                    self.apply_failed("Display status unknown; wait for a fresh snapshot");
                    return None;
                }
                self.modal = None;
                return Some(AppCommand::SetKrakenDisplay {
                    device_id,
                    mode: KrakenDisplayMode::ALL[self.display_cursor],
                });
            }
            _ => {}
        }
        None
    }

    pub fn display_change_succeeded(&mut self) {
        self.display_operation_busy = false;
        self.status = StatusMessage {
            kind: StatusKind::Success,
            text: "Kraken display changed".into(),
        };
    }

    pub fn display_change_failed(&mut self, error: &str) {
        self.display_operation_busy = false;
        self.apply_failed(format!("Kraken display: {error}"));
    }

    fn open_host_editor(&mut self) {
        // Navigation is never permission to control hardware. Keep this view
        // reachable even when the service advertises no motherboard channels.
        self.editor_mode = EditorMode::Host;
        self.modal = None;
        self.status = StatusMessage::info(if !self.host_status_trusted {
            "Status unknown"
        } else if self.host_control_state == HostControlState::Disabled {
            "Motherboard control disabled"
        } else if self.host_editor.channels.is_empty() {
            "No motherboard fan channels"
        } else {
            "Motherboard fans"
        });
    }

    fn handle_host_editor_key(&mut self, key_event: KeyEvent) -> Option<AppCommand> {
        match key_event.code {
            KeyCode::Char('p' | 'P') => self.open_host_profile_picker(),
            KeyCode::Char('c' | 'C') | KeyCode::Char(']') => self.select_next_host_channel(),
            KeyCode::Char('[') => self.select_previous_host_channel(),
            KeyCode::Left => {
                self.host_editor.selected_point = self.host_editor.selected_point.saturating_sub(1)
            }
            KeyCode::Right => {
                let count = self
                    .host_editor
                    .active_channel()
                    .map_or(0, |channel| channel.curve.points.len());
                if self.host_editor.selected_point + 1 < count {
                    self.host_editor.selected_point += 1;
                }
            }
            KeyCode::Home => self.host_editor.selected_point = 0,
            KeyCode::End => {
                if let Some(last) = self
                    .host_editor
                    .active_channel()
                    .and_then(|channel| channel.curve.points.len().checked_sub(1))
                {
                    self.host_editor.selected_point = last;
                }
            }
            KeyCode::Up | KeyCode::Down => {
                let magnitude = if key_event.modifiers.contains(KeyModifiers::SHIFT) {
                    5
                } else {
                    1
                };
                let delta = if key_event.code == KeyCode::Up {
                    magnitude
                } else {
                    -magnitude
                };
                self.change_selected_host_duty(delta);
            }
            KeyCode::Char('s' | 'S') => self.cycle_host_source(),
            KeyCode::Char('r' | 'R') => self.reset_selected_host_curve(),
            KeyCode::Enter => return self.host_enter_command(),
            _ => {}
        }
        None
    }

    fn host_edits_allowed(&mut self) -> bool {
        let reason = if self.host_operation_busy {
            Some("Fan operation in progress")
        } else if !self.host_status_trusted {
            Some("Status unknown")
        } else {
            match self.host_control_state {
                HostControlState::Running => None,
                HostControlState::Restoring => Some("Restoring BIOS control"),
                HostControlState::RestoreRequired => Some("Recovery required"),
                HostControlState::Disabled => Some("Control disabled"),
                HostControlState::Available if self.selected_host_channel().is_none() => {
                    Some("No fan channels")
                }
                HostControlState::Available => None,
            }
        };
        if let Some(reason) = reason {
            self.apply_failed(reason);
            false
        } else {
            true
        }
    }

    pub fn host_profiles(&self) -> Vec<HostCurveProfile> {
        self.selected_host_channel()
            .map_or_else(Vec::new, |channel| {
                let mut profiles = built_in_host_profiles(
                    channel.curve.source,
                    channel.capability.minimum_duty_percent,
                );
                profiles.extend(self.profile_library.profiles.iter().filter_map(|profile| {
                    let ProfileCurve::Host(curve) = &profile.curve else {
                        return None;
                    };
                    let mut curve = curve.clone();
                    normalize_host_curve(&mut curve, channel.capability.minimum_duty_percent);
                    Some(HostCurveProfile {
                        name: profile.name.clone(),
                        curve,
                    })
                }));
                profiles
            })
    }

    pub fn active_host_profile_name(&self) -> Option<String> {
        let channel = self.selected_host_channel()?;
        if let Some(reference) = self
            .host_active_profiles
            .get(&channel.capability.channel_id)
            .copied()
        {
            if !self.host_reference_matches(reference, channel) {
                return Some("Custom".into());
            }
            if let ProfileRef::BuiltIn(_) = reference {
                let matches = built_in_host_profiles(
                    channel.curve.source,
                    channel.capability.minimum_duty_percent,
                )
                .into_iter()
                .filter(|p| p.curve == channel.curve)
                .count();
                if matches != 1 {
                    return None;
                }
            }
            return self.profile_library.name(reference).map(str::to_owned);
        }
        let profiles = built_in_host_profiles(
            channel.curve.source,
            channel.capability.minimum_duty_percent,
        );
        let mut matching = profiles
            .iter()
            .filter(|profile| profile.curve == channel.curve);
        if let Some(profile) = matching.next() {
            return matching.next().is_none().then(|| profile.name.clone());
        }
        Some(
            if channel.curve == conservative_host_curve(channel.capability.minimum_duty_percent) {
                "Default".into()
            } else {
                "Custom".into()
            },
        )
    }

    fn open_host_profile_picker(&mut self) {
        // Browsing/renaming saved names is independent of hardware availability;
        // loading a curve still rechecks the live host-control guard.
        if self.selected_host_channel().is_none() {
            return;
        }
        let active = self.active_host_profile_name();
        let selected_ref = self.selected_host_channel().and_then(|c| {
            self.host_active_profiles
                .get(&c.capability.channel_id)
                .copied()
        });
        self.profile_cursor = (0..self.host_profiles().len())
            .find(|&index| self.host_ref(index) == selected_ref)
            .or_else(|| {
                self.host_profiles()
                    .iter()
                    .position(|profile| Some(profile.name.as_str()) == active.as_deref())
            })
            .unwrap_or(1);
        self.host_profile_target = self
            .selected_host_channel()
            .map(|channel| channel.capability.channel_id.clone());
        self.modal = Some(Modal::HostProfiles);
    }

    fn handle_host_profile_picker_key(&mut self, key_event: KeyEvent) -> Option<AppCommand> {
        let count = self.host_profiles().len();
        match key_event.code {
            KeyCode::Esc | KeyCode::Char('p' | 'P') => {
                self.modal = None;
                self.host_profile_target = None;
            }
            KeyCode::Char('q' | 'Q') => self.exit = true,
            KeyCode::Up | KeyCode::Left if count > 0 => {
                self.profile_cursor = (self.profile_cursor + count - 1) % count
            }
            KeyCode::Down | KeyCode::Right if count > 0 => {
                self.profile_cursor = (self.profile_cursor + 1) % count
            }
            KeyCode::Home if count > 0 => self.profile_cursor = 0,
            KeyCode::End if count > 0 => self.profile_cursor = count - 1,
            KeyCode::Char('r' | 'R') => {
                self.open_rename(self.host_ref(self.profile_cursor), EditorMode::Host)
            }
            KeyCode::Enter => return self.load_host_profile(),
            _ => {}
        }
        None
    }

    fn load_host_profile(&mut self) -> Option<AppCommand> {
        if !self.host_edits_allowed() {
            return None;
        }
        let channel_id = self
            .selected_host_channel()
            .map(|channel| &channel.capability.channel_id);
        if self.host_profile_target.as_ref() != channel_id {
            self.modal = None;
            self.host_profile_target = None;
            self.apply_failed("Fan selection changed; choose a profile again");
            return None;
        }
        let profile = self.host_profiles().get(self.profile_cursor).cloned()?;
        let reference = self.host_ref(self.profile_cursor)?;
        if let Some(channel) = self
            .host_editor
            .channels
            .get_mut(self.host_editor.selected_channel)
        {
            channel.curve = profile.curve;
            self.host_active_profiles
                .insert(channel.capability.channel_id.clone(), reference);
            self.host_identity_curves
                .insert(channel.capability.channel_id.clone(), channel.curve.clone());
            self.host_editor.selected_point = self
                .host_editor
                .selected_point
                .min(channel.curve.points.len().saturating_sub(1));
        }
        self.modal = None;
        self.host_profile_target = None;
        self.status = StatusMessage::info(format!("{} loaded", profile.name));
        if self.host_control_state == HostControlState::Running {
            self.open_host_apply_scope();
        }
        None
    }

    fn select_next_host_channel(&mut self) {
        let count = self.host_editor.channels.len();
        if count > 0 {
            self.host_editor.selected_channel = (self.host_editor.selected_channel + 1) % count;
            self.host_editor.selected_point = self.host_editor.selected_point.min(
                self.host_editor.channels[self.host_editor.selected_channel]
                    .curve
                    .points
                    .len()
                    .saturating_sub(1),
            );
        }
    }

    fn select_previous_host_channel(&mut self) {
        let count = self.host_editor.channels.len();
        if count > 0 {
            self.host_editor.selected_channel =
                (self.host_editor.selected_channel + count - 1) % count;
            self.host_editor.selected_point = self.host_editor.selected_point.min(
                self.host_editor.channels[self.host_editor.selected_channel]
                    .curve
                    .points
                    .len()
                    .saturating_sub(1),
            );
        }
    }

    fn change_selected_host_duty(&mut self, delta: i16) {
        if !self.host_edits_allowed() {
            return;
        }
        let point_index = self.host_editor.selected_point;
        let Some(channel) = self
            .host_editor
            .channels
            .get_mut(self.host_editor.selected_channel)
        else {
            return;
        };
        let points = &mut channel.curve.points;
        if point_index >= points.len() {
            return;
        }
        if point_index + 1 == points.len() {
            self.status = StatusMessage::info("The final host point is locked at 100%");
            return;
        }
        let old_duty = points[point_index].duty_percent;
        let duty = (i16::from(old_duty) + delta).clamp(
            i16::from(channel.capability.minimum_duty_percent.clamp(30, 100)),
            100,
        ) as u8;
        if duty == old_duty {
            return;
        }
        points[point_index].duty_percent = duty;
        let mut following = 0;
        if duty > old_duty {
            for point in &mut points[point_index + 1..] {
                if point.duty_percent < duty {
                    point.duty_percent = duty;
                    following += 1;
                }
            }
        } else {
            for point in &mut points[..point_index] {
                if point.duty_percent > duty {
                    point.duty_percent = duty;
                    following += 1;
                }
            }
        }
        self.status = StatusMessage::info(if following == 0 {
            format!("Duty {duty}%")
        } else if duty > old_duty {
            format!("Duty {duty}% · {following} hotter points raised")
        } else {
            format!("Duty {duty}% · {following} cooler points lowered")
        });
    }

    fn cycle_host_source(&mut self) {
        if !self.host_edits_allowed() {
            return;
        }
        let Some(channel) = self
            .host_editor
            .channels
            .get_mut(self.host_editor.selected_channel)
        else {
            return;
        };
        channel.curve.source = match channel.curve.source {
            HostTemperatureSource::Cpu => HostTemperatureSource::Gpu,
            HostTemperatureSource::Gpu => HostTemperatureSource::CpuGpuMax,
            HostTemperatureSource::CpuGpuMax => HostTemperatureSource::Cpu,
        };
        self.status = StatusMessage::info(format!("Source {}", channel.curve.source));
    }

    fn reset_selected_host_curve(&mut self) {
        if !self.host_edits_allowed() {
            return;
        }
        let Some(channel) = self
            .host_editor
            .channels
            .get_mut(self.host_editor.selected_channel)
        else {
            return;
        };
        channel.curve = conservative_host_curve(channel.capability.minimum_duty_percent);
        self.status = StatusMessage::info("Curve reset");
    }

    fn open_host_apply_scope(&mut self) {
        if self.host_operation_busy
            || !self.host_status_trusted
            || self.host_control_state != HostControlState::Running
        {
            self.apply_failed("Wait for running fan status / pending operation");
            return;
        }
        let Some(channel) = self.selected_host_channel() else {
            self.apply_failed("No fan group selected");
            return;
        };
        self.host_apply_scope = Some(HostApplyScope {
            selected: channel.capability.clone(),
            curve: channel.curve.clone(),
            targets: self
                .host_editor
                .channels
                .iter()
                .map(|c| c.capability.clone())
                .collect(),
            active_policy: self.snapshot.host_control.active_policy.clone(),
            all_groups: false,
        });
        self.modal = Some(Modal::HostApplyScope);
    }

    fn handle_host_apply_scope_key(&mut self, key: KeyEvent) -> Option<AppCommand> {
        match key.code {
            KeyCode::Esc => {
                self.modal = None;
                self.host_apply_scope = None;
            }
            KeyCode::Char('q' | 'Q') => {
                self.modal = None;
                self.host_apply_scope = None;
                self.exit = true;
            }
            KeyCode::Up | KeyCode::Left | KeyCode::Home => {
                if let Some(scope) = &mut self.host_apply_scope {
                    scope.all_groups = false;
                }
            }
            KeyCode::Down | KeyCode::Right | KeyCode::End => {
                if let Some(scope) = &mut self.host_apply_scope {
                    scope.all_groups = true;
                }
            }
            KeyCode::Enter => return self.confirm_host_apply_scope(),
            _ => {}
        }
        None
    }

    fn confirm_host_apply_scope(&mut self) -> Option<AppCommand> {
        self.modal = None;
        let scope = self.host_apply_scope.take()?;
        let current = self.selected_host_channel();
        let fresh_targets = self
            .host_editor
            .channels
            .iter()
            .map(|c| &c.capability)
            .collect::<Vec<_>>();
        let pinned_targets = scope.targets.iter().collect::<Vec<_>>();
        if self.editor_mode != EditorMode::Host
            || !self.host_status_trusted
            || self.host_operation_busy
            || self.host_control_state != HostControlState::Running
            || self.snapshot.host_control.active_policy != scope.active_policy
            || current.is_none_or(|c| c.capability != scope.selected || c.curve != scope.curve)
            || fresh_targets != pinned_targets
        {
            self.apply_failed("Fan selection or capabilities changed; choose apply scope again");
            return None;
        }
        let targets = if scope.all_groups {
            &self.host_editor.channels[..]
        } else {
            std::slice::from_ref(current.unwrap())
        };
        let channel_policies = targets
            .iter()
            .map(|channel| {
                let mut curve = scope.curve.clone();
                for point in &mut curve.points {
                    point.duty_percent = point
                        .duty_percent
                        .clamp(channel.capability.minimum_duty_percent.clamp(30, 100), 100);
                }
                HostChannelPolicy {
                    channel_id: channel.capability.channel_id.clone(),
                    curve,
                }
            })
            .collect::<Vec<_>>();
        let selected_reference = self
            .host_active_profiles
            .get(&scope.selected.channel_id)
            .copied()
            .filter(|reference| {
                self.host_editor
                    .channels
                    .iter()
                    .find(|c| c.capability.channel_id == scope.selected.channel_id)
                    .is_some_and(|c| self.host_reference_matches(*reference, c))
            });
        // Confirmation is the first point where other groups' working drafts change.
        if scope.all_groups {
            for accepted in &channel_policies {
                if let Some(channel) = self
                    .host_editor
                    .channels
                    .iter_mut()
                    .find(|c| c.capability.channel_id == accepted.channel_id)
                {
                    channel.curve = accepted.curve.clone();
                    if let Some(reference) = selected_reference {
                        self.host_active_profiles
                            .insert(accepted.channel_id.clone(), reference);
                        self.host_identity_curves
                            .insert(accepted.channel_id.clone(), accepted.curve.clone());
                    } else {
                        self.host_active_profiles.remove(&accepted.channel_id);
                        self.host_identity_curves.remove(&accepted.channel_id);
                    }
                }
            }
        }
        let changed = channel_policies.iter().any(|accepted| {
            scope
                .active_policy
                .as_ref()
                .and_then(|policy| {
                    policy
                        .channels
                        .iter()
                        .find(|c| c.channel_id == accepted.channel_id)
                })
                .is_none_or(|c| c.curve != accepted.curve)
        });
        if !changed {
            self.status = StatusMessage::info("Fan curve already applied — remembering profile");
            return Some(AppCommand::RememberHostProfiles { channel_policies });
        }
        self.status = StatusMessage::info(if scope.all_groups {
            format!("Updating {} fan groups…", channel_policies.len())
        } else {
            "Updating selected fan group…".into()
        });
        Some(AppCommand::UpdateHostControl { channel_policies })
    }

    fn host_stop_command(&mut self) -> Option<AppCommand> {
        if self.host_operation_busy {
            // The worker queues this explicitly requested stop behind the mutation.
            self.status = StatusMessage::info("Stop queued after fan operation…");
            return Some(AppCommand::StopHostControl);
        }
        if self.host_control_state == HostControlState::Available && self.host_status_trusted {
            self.status = StatusMessage::info("Fan control is available (not running)");
            return None;
        }
        if self.host_control_state == HostControlState::Disabled && self.host_status_trusted {
            self.apply_failed("Control disabled");
            return None;
        }
        self.status = StatusMessage::info("Stopping fan control…");
        Some(AppCommand::StopHostControl)
    }

    fn host_enter_command(&mut self) -> Option<AppCommand> {
        if self.host_operation_busy || !self.host_status_trusted {
            self.apply_failed("Wait for fan status / pending operation");
            return None;
        }
        match self.host_control_state {
            HostControlState::Available => {
                if self.host_editor.channels.is_empty() {
                    self.apply_failed("No fan channels");
                    return None;
                }
                let policy = self.host_editor.policy();
                self.status = StatusMessage::info("Starting fan control…");
                Some(AppCommand::StartHostControl { policy })
            }
            HostControlState::Running => {
                self.open_host_apply_scope();
                None
            }
            HostControlState::Disabled => {
                self.apply_failed("Control disabled");
                None
            }
            HostControlState::Restoring => {
                self.apply_failed("Restoring BIOS control");
                None
            }
            HostControlState::RestoreRequired => {
                self.apply_failed("Recovery required; O for Settings");
                None
            }
        }
    }

    fn open_profile_picker(&mut self) {
        if self.active_channel().is_none() {
            return;
        }
        if let Some(index) = self
            .profiles
            .iter()
            .position(|profile| profile.name == self.active_profile_name())
        {
            self.profile_cursor = index;
        }
        self.modal = Some(Modal::Profiles);
    }

    fn open_rename(&mut self, reference: Option<ProfileRef>, mode: EditorMode) {
        let Some(ProfileRef::Custom(id)) = reference else {
            self.apply_failed("Built-in profiles cannot be renamed");
            return;
        };
        let channel = (mode == EditorMode::Host)
            .then(|| self.host_profile_target.clone())
            .flatten();
        self.rename_target = Some((
            mode,
            ProfileRef::Custom(id),
            channel,
            self.active_curve_key(),
        ));
        self.profile_name_input = self
            .profile_library
            .name(ProfileRef::Custom(id))
            .unwrap_or("")
            .to_owned();
        self.modal = Some(Modal::RenameProfile);
    }

    fn handle_profile_picker_key(&mut self, key_event: KeyEvent) -> Option<AppCommand> {
        let count = self.profiles.len();
        match key_event.code {
            KeyCode::Esc | KeyCode::Char('p' | 'P') => self.modal = None,
            KeyCode::Char('q' | 'Q') => self.exit = true,
            KeyCode::Up | KeyCode::Left if count > 0 => {
                self.profile_cursor = (self.profile_cursor + count - 1) % count;
            }
            KeyCode::Down | KeyCode::Right if count > 0 => {
                self.profile_cursor = (self.profile_cursor + 1) % count;
            }
            KeyCode::Home if count > 0 => self.profile_cursor = 0,
            KeyCode::End if count > 0 => self.profile_cursor = count - 1,
            KeyCode::Char('r' | 'R') => {
                self.open_rename(self.firmware_ref(self.profile_cursor), EditorMode::Firmware)
            }
            KeyCode::Enter => return self.apply_selected_profile(),
            _ => {}
        }
        None
    }

    fn handle_profile_name_key(&mut self, key_event: KeyEvent) -> Option<AppCommand> {
        match key_event.code {
            KeyCode::Esc => {
                self.modal = None;
                self.profile_name_input.clear();
                self.rename_target = None;
            }
            KeyCode::Backspace => {
                self.profile_name_input.pop();
            }
            KeyCode::Enter => return self.rename_profile_command(),
            KeyCode::Char(character)
                if !key_event
                    .modifiers
                    .intersects(KeyModifiers::CONTROL | KeyModifiers::ALT)
                    && !character.is_control()
                    && self.profile_name_input.chars().count() < MAX_PROFILE_NAME_CHARS =>
            {
                self.profile_name_input.push(character);
            }
            _ => {}
        }
        None
    }

    fn handle_apply_confirmation_key(&mut self, key_event: KeyEvent) -> Option<AppCommand> {
        match key_event.code {
            KeyCode::Char('y' | 'Y') => {
                self.modal = None;
                let pending = self.pending_confirmation.take()?;
                let valid = self.editor_mode == EditorMode::Firmware
                    && self.active_curve_key().as_ref() == Some(&pending.key)
                    && self.active_device().is_some_and(|device| device.online)
                    && self.find_channel(&pending.key).is_some_and(|channel| {
                        channel.points == pending.points
                            && channel.min_duty == pending.min_duty
                            && channel.max_duty == pending.max_duty
                            && channel.source == pending.source
                    });
                if !valid {
                    self.apply_failed("Curve or target changed; confirm the firmware write again");
                    return None;
                }
                self.status = StatusMessage::info(self.curve_send_message());
                Some(pending.command)
            }
            KeyCode::Char('n' | 'N') | KeyCode::Esc => {
                self.modal = None;
                self.pending_confirmation = None;
                self.status =
                    StatusMessage::info("Curve apply cancelled — no hardware changes made");
                None
            }
            _ => None,
        }
    }

    fn apply_selected_profile(&mut self) -> Option<AppCommand> {
        let profile = self.selected_profile()?.clone();
        let key = self.active_curve_key()?;
        let device_index = *self
            .cooling_device_indices()
            .get(self.selected_cooling_device)?;
        let channel = self
            .snapshot
            .devices
            .get_mut(device_index)?
            .cooling_channels
            .get_mut(self.selected_channel)?;
        for (point, preset) in channel.points.iter_mut().zip(&profile.points) {
            point.duty = preset.duty.clamp(channel.min_duty, channel.max_duty);
        }
        if let Some(reference) = self.firmware_ref(self.profile_cursor) {
            self.active_profiles.insert(key.clone(), reference);
        }
        self.sync_dirty_state(&key);
        self.modal = None;
        self.apply_command()
    }

    fn rename_profile_command(&mut self) -> Option<AppCommand> {
        let name = self.profile_name_input.trim().to_owned();
        if name.is_empty() {
            self.apply_failed("Profile name cannot be empty");
            return None;
        }
        let (mode, reference, channel, firmware_key) = self.rename_target.as_ref()?;
        if *mode != self.editor_mode
            || (*mode == EditorMode::Host
                && (self.host_profile_target != *channel
                    || self
                        .selected_host_channel()
                        .map(|c| &c.capability.channel_id)
                        != channel.as_ref()))
            || (*mode == EditorMode::Firmware
                && self.active_curve_key().as_ref() != firmware_key.as_ref())
        {
            self.rename_target = None;
            self.modal = None;
            self.apply_failed("Selection changed; choose profile again");
            return None;
        }
        let ProfileRef::Custom(id) = reference else {
            return None;
        };
        Some(AppCommand::RenameProfile { id: *id, name })
    }

    pub fn update_telemetry(&mut self, fresh: HardwareSnapshot) {
        let fresh_host_control = fresh.host_control.clone();
        let fresh_display = fresh.kraken_display.clone();
        let previous_display_device = self.snapshot.kraken_display.device_id.clone();
        let selected_key = self.active_curve_key();
        let selected_point = self.selected_point;
        let mut old_devices = std::mem::take(&mut self.snapshot.devices)
            .into_iter()
            .map(|device| (device.id.clone(), device))
            .collect::<BTreeMap<_, _>>();
        let mut devices = Vec::with_capacity(fresh.devices.len());

        for mut fresh_device in fresh.devices {
            let mut old_channels = old_devices
                .remove(&fresh_device.id)
                .map(|device| {
                    device
                        .cooling_channels
                        .into_iter()
                        .map(|channel| (channel.id.clone(), channel))
                        .collect::<BTreeMap<_, _>>()
                })
                .unwrap_or_default();

            for fresh_channel in &mut fresh_device.cooling_channels {
                normalize_channel(fresh_channel);
                let reported_points = fresh_channel.points.clone();
                let key = (fresh_device.id.clone(), fresh_channel.id.clone());
                if let Some(old_channel) = old_channels.remove(&fresh_channel.id) {
                    let dirty = self.dirty_curves.contains(&key);
                    if dirty || fresh_channel.curve_state == CurveState::Unverified {
                        fresh_channel.points = old_channel.points;
                        clamp_points(
                            &mut fresh_channel.points,
                            fresh_channel.min_duty,
                            fresh_channel.max_duty,
                        );
                    }

                    let baseline = self
                        .applied_curves
                        .entry(key.clone())
                        .or_insert_with(|| reported_points.clone());
                    clamp_points(baseline, fresh_channel.min_duty, fresh_channel.max_duty);
                    if fresh_channel.curve_state == CurveState::Applied {
                        if dirty && fresh_channel.points == *baseline {
                            fresh_channel.points = reported_points.clone();
                            *baseline = reported_points;
                        } else if !dirty {
                            *baseline = reported_points;
                        }
                    }
                } else {
                    if fresh_channel.curve_state == CurveState::Unverified {
                        if let Some(binding) =
                            self.profile_library.bindings.iter().find(|binding| {
                                binding.target
                                    == ProfileTarget::Firmware {
                                        device_id: key.0.clone(),
                                        channel_id: key.1.clone(),
                                    }
                            })
                        {
                            if let ProfileCurve::Firmware(points) = &binding.curve {
                                fresh_channel.points = points.clone();
                                clamp_points(
                                    &mut fresh_channel.points,
                                    fresh_channel.min_duty,
                                    fresh_channel.max_duty,
                                );
                                self.active_profiles.insert(key.clone(), binding.profile);
                            }
                        } else if let Some((index, _)) =
                            self.profiles.iter().enumerate().find(|(_, profile)| {
                                profile_matches_channel(profile, fresh_channel)
                            })
                        {
                            self.active_profiles
                                .insert(key.clone(), self.firmware_ref(index).unwrap());
                        }
                    }
                    self.applied_curves
                        .insert(key, fresh_channel.points.clone());
                }
            }
            devices.push(fresh_device);
        }

        let live_keys = devices
            .iter()
            .flat_map(|device| {
                device
                    .cooling_channels
                    .iter()
                    .map(|channel| (device.id.clone(), channel.id.clone()))
            })
            .collect::<BTreeSet<_>>();
        self.applied_curves.retain(|key, _| live_keys.contains(key));
        self.dirty_curves.retain(|key| live_keys.contains(key));
        self.active_profiles
            .retain(|key, _| live_keys.contains(key));
        let existing_host_ids = self
            .host_editor
            .channels
            .iter()
            .map(|c| c.capability.channel_id.clone())
            .collect::<BTreeSet<_>>();
        self.host_editor.reconcile(&fresh_host_control.channels);
        for channel in &mut self.host_editor.channels {
            if existing_host_ids.contains(&channel.capability.channel_id) {
                continue;
            }
            if let Some(binding) = self.profile_library.bindings.iter().find(|b| {
                b.target
                    == ProfileTarget::Host {
                        channel_id: channel.capability.channel_id.clone(),
                    }
            }) && let ProfileCurve::Host(curve) = &binding.curve
            {
                channel.curve = curve.clone();
                normalize_host_curve(&mut channel.curve, channel.capability.minimum_duty_percent);
                if let Some(baseline) = self
                    .host_editor
                    .baseline
                    .channels
                    .iter_mut()
                    .find(|c| c.channel_id == channel.capability.channel_id)
                {
                    baseline.curve = channel.curve.clone();
                }
                self.host_active_profiles
                    .insert(channel.capability.channel_id.clone(), binding.profile);
                self.host_identity_curves
                    .insert(channel.capability.channel_id.clone(), channel.curve.clone());
            }
        }
        self.host_active_profiles.retain(|id, _| {
            self.host_editor
                .channels
                .iter()
                .any(|c| &c.capability.channel_id == id)
        });
        self.host_identity_curves.retain(|id, _| {
            self.host_editor
                .channels
                .iter()
                .any(|c| &c.capability.channel_id == id)
        });
        if fresh_host_control.state == HostControlState::Running
            && let Some(policy) = &fresh_host_control.active_policy
        {
            self.host_editor.adopt_running(policy);
        }
        if fresh_host_control.last_error.is_some()
            && fresh_host_control.last_error != self.snapshot.host_control.last_error
        {
            self.apply_failed(format!(
                "Fan control: {}",
                fresh_host_control.last_error.as_deref().unwrap()
            ));
        }
        if fresh_display.last_error.is_some()
            && fresh_display.last_error != self.snapshot.kraken_display.last_error
        {
            self.apply_failed(format!(
                "Kraken display: {}",
                fresh_display.last_error.as_deref().unwrap()
            ));
        } else if self.snapshot.kraken_display.last_error.is_some()
            && fresh_display.last_error.is_none()
            && self.status.text.starts_with("Kraken display:")
        {
            self.status = StatusMessage::info("Kraken display updates resumed");
        }
        self.host_control_state = fresh_host_control.state;
        self.host_status_trusted = fresh_host_control.state != HostControlState::Running
            || fresh_host_control.active_policy.is_some();
        if self.modal == Some(Modal::KrakenDisplay)
            && fresh_display.device_id != previous_display_device
        {
            self.modal = None;
            self.apply_failed("Kraken display target changed; reopen the picker");
        }
        if self.backend_name != "DEMO"
            && !fresh.monitoring.opted_in
            && !matches!(self.modal, None | Some(Modal::Help | Modal::Settings))
        {
            self.modal = None;
            self.pending_confirmation = None;
            self.host_apply_scope = None;
        }
        self.snapshot_received = true;
        self.snapshot = HardwareSnapshot {
            devices,
            sequence: fresh.sequence,
            host_control: fresh_host_control,
            kraken_display: fresh_display,
            monitoring: fresh.monitoring,
        };
        for key in self.dirty_curves.clone() {
            self.sync_dirty_state(&key);
        }

        if selected_key
            .as_ref()
            .is_none_or(|key| !self.restore_selection(key))
        {
            self.clamp_selection();
        }
        self.selected_point = selected_point;
        self.clamp_point();
    }

    fn curve_send_message(&self) -> &'static str {
        if self.backend_name == "DEMO" {
            "Sending curve to simulator…"
        } else {
            "Sending curve to monitoring service…"
        }
    }

    pub fn backend_connected(&mut self) {
        // A reconnect must not replace a fresh controller-fault diagnostic with
        // a generic success banner; unchanged later snapshots won't repeat it.
        if let Some(error) = &self.snapshot.host_control.last_error {
            self.status = StatusMessage {
                kind: StatusKind::Error,
                text: format!("Fan control: {error}"),
            };
        } else if let Some(error) = &self.snapshot.kraken_display.last_error {
            self.status = StatusMessage {
                kind: StatusKind::Error,
                text: format!("Kraken display: {error}"),
            };
        } else if self.backend_name != "DEMO" {
            self.status = StatusMessage {
                kind: StatusKind::Success,
                text: "Monitoring service connected".into(),
            };
        }
    }

    pub fn invalidate_live_curve_claims(&mut self) {
        for channel in self
            .snapshot
            .devices
            .iter_mut()
            .flat_map(|device| &mut device.cooling_channels)
        {
            channel.curve_state = CurveState::Unverified;
        }
    }

    pub fn mark_host_status_unknown(&mut self) {
        self.host_status_trusted = false;
    }

    pub fn mark_host_restore_required(&mut self) {
        self.host_status_trusted = true;
        self.host_control_state = HostControlState::RestoreRequired;
        self.snapshot.host_control.state = HostControlState::RestoreRequired;
    }

    pub fn host_start_succeeded(&mut self, policy: HostControlPolicy) {
        self.host_editor.record_applied(policy.clone());
        self.snapshot.host_control.active_policy = Some(policy);
        self.snapshot.host_control.last_error = None;
        self.host_status_trusted = true;
        self.host_control_state = HostControlState::Running;
        self.snapshot.host_control.state = HostControlState::Running;
        self.status = StatusMessage {
            kind: StatusKind::Success,
            text: "Fan control started".into(),
        };
    }

    pub fn host_update_succeeded(&mut self, accepted: Vec<HostChannelPolicy>) {
        for channel_policy in &accepted {
            self.host_editor.record_channel_applied(channel_policy);
            if let Some(active) = self.snapshot.host_control.active_policy.as_mut()
                && let Some(channel) = active
                    .channels
                    .iter_mut()
                    .find(|channel| channel.channel_id == channel_policy.channel_id)
            {
                *channel = channel_policy.clone();
            }
        }
        self.snapshot.host_control.last_error = None;
        self.status = StatusMessage {
            kind: StatusKind::Success,
            text: if accepted.len() == 1 {
                "Fan group updated".into()
            } else {
                format!("{} fan groups updated", accepted.len())
            },
        };
    }

    pub fn host_stop_succeeded(&mut self) {
        self.snapshot.host_control.active_policy = None;
        self.snapshot.host_control.last_error = None;
        self.host_status_trusted = true;
        self.host_control_state = HostControlState::Available;
        self.snapshot.host_control.state = HostControlState::Available;
        self.status = StatusMessage {
            kind: StatusKind::Success,
            text: "Fan control stopped".into(),
        };
    }

    pub fn host_operation_failed(&mut self, error: &crate::backend::BackendError) {
        match error.kind() {
            crate::backend::BackendErrorKind::RestoreRequired => self.mark_host_restore_required(),
            crate::backend::BackendErrorKind::UnknownOutcome => self.mark_host_status_unknown(),
            _ => {}
        }
        self.apply_failed(format!("Fan control: {error}"));
    }

    pub fn apply_succeeded(&mut self, key: CurveKey, points: Vec<CurvePoint>) {
        if let Some(channel) = self.find_channel_mut(&key) {
            channel.curve_state = CurveState::Applied;
            self.applied_curves.insert(key.clone(), points);
            self.sync_dirty_state(&key);
        }
        self.status = StatusMessage {
            kind: StatusKind::Success,
            text: if self.backend_name == "DEMO" {
                "Curve accepted by simulator"
            } else {
                "Curve accepted by monitoring service"
            }
            .into(),
        };
    }

    pub fn apply_outcome_unknown(&mut self, key: &CurveKey, error: impl Into<String>) {
        if let Some(channel) = self.find_channel_mut(key) {
            channel.curve_state = CurveState::Unverified;
        }
        self.apply_failed(error);
    }

    pub fn apply_failed(&mut self, error: impl Into<String>) {
        self.status = StatusMessage {
            kind: StatusKind::Error,
            text: error.into(),
        };
    }

    pub fn waiting_for_pending_operations(&mut self, count: usize) {
        self.status = StatusMessage::info(format!(
            "Waiting for {count} pending hardware operation(s), quit again to detach immediately"
        ));
    }

    pub fn curve_write_queued(&mut self) {
        self.status = StatusMessage::info(
            "A curve write is in progress — queued the latest curve for this channel",
        );
    }

    pub fn queued_curve_write_started(&mut self) {
        self.status = StatusMessage::info("Applying the latest queued curve…");
    }

    fn select_next_control(&mut self) {
        let count = self.cooling_device_indices().len();
        if self.editor_mode == EditorMode::Host {
            self.select_firmware_control(0);
        } else if count == 0 || self.selected_cooling_device + 1 >= count {
            self.open_host_editor();
        } else {
            self.select_firmware_control(self.selected_cooling_device + 1);
        }
    }

    fn select_previous_control(&mut self) {
        let count = self.cooling_device_indices().len();
        if self.editor_mode == EditorMode::Host {
            self.select_firmware_control(count.saturating_sub(1));
        } else if count == 0 || self.selected_cooling_device == 0 {
            self.open_host_editor();
        } else {
            self.select_firmware_control(self.selected_cooling_device - 1);
        }
    }

    fn select_firmware_control(&mut self, index: usize) {
        self.editor_mode = EditorMode::Firmware;
        // Visiting the motherboard view must not silently retarget a Kraken
        // fan edit to its pump when returning to that same firmware device.
        if self.selected_cooling_device != index {
            self.selected_channel = 0;
        }
        self.selected_cooling_device = index;
        self.clamp_selection();
        self.status = StatusMessage::info("No firmware cooling devices are available");
        self.announce_selection();
    }

    fn select_next_channel(&mut self) {
        let count = self
            .active_device()
            .map_or(0, |device| device.cooling_channels.len());
        if count > 0 {
            self.selected_channel = (self.selected_channel + 1) % count;
            self.clamp_point();
            self.announce_selection();
        }
    }

    fn select_previous_channel(&mut self) {
        let count = self
            .active_device()
            .map_or(0, |device| device.cooling_channels.len());
        if count > 0 {
            self.selected_channel = (self.selected_channel + count - 1) % count;
            self.clamp_point();
            self.announce_selection();
        }
    }

    fn select_previous_point(&mut self) {
        if self.selected_point > 0 {
            self.selected_point -= 1;
        }
    }

    fn select_next_point(&mut self) {
        let count = self
            .active_channel()
            .map_or(0, |channel| channel.points.len());
        if self.selected_point + 1 < count {
            self.selected_point += 1;
        }
    }

    fn select_first_point(&mut self) {
        self.selected_point = 0;
    }

    fn select_last_point(&mut self) {
        if let Some(last) = self
            .active_channel()
            .and_then(|channel| channel.points.len().checked_sub(1))
        {
            self.selected_point = last;
        }
    }

    fn change_selected_duty(&mut self, delta: i16) {
        let Some(device_index) = self
            .cooling_device_indices()
            .get(self.selected_cooling_device)
            .copied()
        else {
            return;
        };

        let key;
        let new_duty;
        let raised;
        let mut following = 0;
        let failsafe;
        {
            let Some(device) = self.snapshot.devices.get_mut(device_index) else {
                return;
            };
            let Some(channel) = device.cooling_channels.get_mut(self.selected_channel) else {
                return;
            };
            let point_index = self.selected_point;
            let Some(point) = channel.points.get(point_index) else {
                return;
            };
            let old_duty = point.duty;
            // The firmware validator requires the 59°C endpoint at 100%.
            // Keep it fixed, and allow an older invalid draft to be repaired.
            failsafe = point_index + 1 == channel.points.len()
                && point.temperature == 59
                && channel.max_duty == 100;
            new_duty = if failsafe {
                100
            } else {
                (i16::from(old_duty) + delta)
                    .clamp(i16::from(channel.min_duty), i16::from(channel.max_duty))
                    as u8
            };
            if new_duty == old_duty {
                if failsafe {
                    self.status = StatusMessage::info("The final firmware point is locked at 100%");
                }
                return;
            }
            channel.points[point_index].duty = new_duty;
            raised = new_duty > old_duty;
            if raised {
                for point in &mut channel.points[point_index + 1..] {
                    if point.duty < new_duty {
                        point.duty = new_duty;
                        following += 1;
                    }
                }
            } else {
                for point in &mut channel.points[..point_index] {
                    if point.duty > new_duty {
                        point.duty = new_duty;
                        following += 1;
                    }
                }
            }
            key = (device.id.clone(), channel.id.clone());
        }

        self.sync_dirty_state(&key);
        self.status = StatusMessage::info(if failsafe {
            "The final firmware point is locked at 100%".into()
        } else if following == 0 {
            format!("Duty {new_duty}%")
        } else if raised {
            format!("Duty {new_duty}% · {following} hotter points raised")
        } else {
            format!("Duty {new_duty}% · {following} cooler points lowered")
        });
    }

    fn reset_active_curve(&mut self) {
        let Some(key) = self.active_curve_key() else {
            return;
        };
        let Some(applied) = self.applied_curves.get(&key).cloned() else {
            return;
        };
        if let Some(channel) = self.find_channel_mut(&key) {
            channel.points = applied;
        }
        self.dirty_curves.remove(&key);
        self.status = StatusMessage::info("Unsaved edits were reset");
    }

    fn apply_command(&mut self) -> Option<AppCommand> {
        if !self.active_device()?.online {
            self.status = StatusMessage::info(
                "Device is offline — the curve remains loaded but was not applied",
            );
            return None;
        }

        let key = self.active_curve_key()?;
        let points = self.find_channel(&key)?.points.clone();
        let profile = if self.active_profile_is_modified() {
            None
        } else {
            self.active_profiles.get(&key).copied()
        };
        let command = AppCommand::ApplyCurve {
            key: key.clone(),
            points,
            profile,
        };
        if self.confirm_apply && self.backend_name != "DEMO" {
            let channel = self.find_channel(&key)?;
            let pending = FirmwareConfirmation {
                command,
                key,
                points: channel.points.clone(),
                min_duty: channel.min_duty,
                max_duty: channel.max_duty,
                source: channel.source,
            };
            self.pending_confirmation = Some(pending);
            self.modal = Some(Modal::ConfirmApply);
            self.status =
                StatusMessage::info("Confirm firmware curve write with Y, or press N to cancel");
            None
        } else {
            self.status = StatusMessage::info(self.curve_send_message());
            Some(command)
        }
    }

    fn sync_dirty_state(&mut self, key: &CurveKey) {
        let is_dirty = self
            .find_channel(key)
            .zip(self.applied_curves.get(key))
            .is_some_and(|(channel, applied)| channel.points != *applied);

        if is_dirty {
            self.dirty_curves.insert(key.clone());
        } else {
            self.dirty_curves.remove(key);
        }
    }

    fn find_channel(&self, key: &CurveKey) -> Option<&CoolingChannel> {
        self.snapshot
            .devices
            .iter()
            .find(|device| device.id == key.0)?
            .cooling_channels
            .iter()
            .find(|channel| channel.id == key.1)
    }

    fn find_channel_mut(&mut self, key: &CurveKey) -> Option<&mut CoolingChannel> {
        self.snapshot
            .devices
            .iter_mut()
            .find(|device| device.id == key.0)?
            .cooling_channels
            .iter_mut()
            .find(|channel| channel.id == key.1)
    }

    fn restore_selection(&mut self, key: &CurveKey) -> bool {
        let mut cooling_device = 0;
        for device in &self.snapshot.devices {
            if device.cooling_channels.is_empty() {
                continue;
            }
            if device.id == key.0
                && let Some(channel) = device
                    .cooling_channels
                    .iter()
                    .position(|channel| channel.id == key.1)
            {
                self.selected_cooling_device = cooling_device;
                self.selected_channel = channel;
                return true;
            }
            cooling_device += 1;
        }
        false
    }

    fn clamp_selection(&mut self) {
        let device_count = self.cooling_device_indices().len();
        if device_count == 0 {
            self.selected_cooling_device = 0;
            self.selected_channel = 0;
            self.selected_point = 0;
            return;
        }
        self.selected_cooling_device = self.selected_cooling_device.min(device_count - 1);
        let channel_count = self
            .active_device()
            .map_or(0, |device| device.cooling_channels.len());
        self.selected_channel = self.selected_channel.min(channel_count.saturating_sub(1));
        self.clamp_point();
    }

    fn clamp_point(&mut self) {
        let point_count = self
            .active_channel()
            .map_or(0, |channel| channel.points.len());
        self.selected_point = self.selected_point.min(point_count.saturating_sub(1));
    }

    fn announce_selection(&mut self) {
        if let (Some(device), Some(channel)) = (self.active_device(), self.active_channel()) {
            self.status =
                StatusMessage::info(format!("Editing {} / {}", device.name, channel.name));
        }
    }
}

fn builtin_for_index(index: usize) -> BuiltinProfile {
    match index {
        0 => BuiltinProfile::Silent,
        1 => BuiltinProfile::Progressive,
        _ => BuiltinProfile::Performance,
    }
}
fn builtin_index(profile: BuiltinProfile) -> usize {
    match profile {
        BuiltinProfile::Silent => 0,
        BuiltinProfile::Progressive => 1,
        BuiltinProfile::Performance => 2,
    }
}
fn profile_ref_for_index(library: &ProfileLibrary, index: usize) -> ProfileRef {
    if index < 3 {
        ProfileRef::BuiltIn(builtin_for_index(index))
    } else {
        ProfileRef::Custom(
            library
                .profiles
                .iter()
                .filter(|p| matches!(p.curve, ProfileCurve::Firmware(_)))
                .nth(index - 3)
                .expect("displayed firmware profile exists")
                .id,
        )
    }
}

fn profile_matches_channel(profile: &CurveProfile, channel: &CoolingChannel) -> bool {
    points_match_channel(&profile.points, channel)
}

fn points_match_channel(points: &[CurvePoint], channel: &CoolingChannel) -> bool {
    channel.points.len() == points.len()
        && channel.points.iter().zip(points).all(|(actual, saved)| {
            actual.temperature == saved.temperature
                && actual.duty == saved.duty.clamp(channel.min_duty, channel.max_duty)
        })
}

fn normalize_channel(channel: &mut CoolingChannel) {
    channel.max_duty = channel.max_duty.min(100);
    channel.min_duty = channel.min_duty.min(channel.max_duty);
    clamp_points(&mut channel.points, channel.min_duty, channel.max_duty);
}

fn clamp_points(points: &mut [CurvePoint], min_duty: u8, max_duty: u8) {
    for point in points {
        point.duty = point.duty.clamp(min_duty, max_duty);
    }
}

fn conservative_host_curve(minimum_duty_percent: u8) -> HostCurve {
    let minimum = minimum_duty_percent.min(100);
    let stops = [
        (22_i32, 40_u8),
        (40, 40),
        (50, 50),
        (60, 65),
        (70, 80),
        (80, 90),
        (100, 100),
    ];
    let points = (0..HOST_POINT_COUNT)
        .map(|index| {
            let temperature = HOST_FIRST_TEMPERATURE_MILLIDEGREES
                + i32::try_from(index).expect("host point index fits")
                    * HOST_TEMPERATURE_STEP_MILLIDEGREES;
            HostCurvePoint {
                temperature_millidegrees: temperature,
                duty_percent: interpolate_host_stops(&stops, temperature / 1_000).max(minimum),
            }
        })
        .collect();
    HostCurve {
        source: HostTemperatureSource::Cpu,
        points,
    }
}

fn interpolate_host_stops(stops: &[(i32, u8)], temperature: i32) -> u8 {
    if temperature <= stops[0].0 {
        return stops[0].1;
    }
    for window in stops.windows(2) {
        let (low_temperature, low_duty) = window[0];
        let (high_temperature, high_duty) = window[1];
        if temperature <= high_temperature {
            let span = high_temperature - low_temperature;
            let position = temperature - low_temperature;
            return u8::try_from(
                i32::from(low_duty)
                    + (i32::from(high_duty) - i32::from(low_duty)) * position / span,
            )
            .expect("interpolated host duty is bounded");
        }
    }
    stops.last().map_or(100, |(_, duty)| *duty)
}

fn normalize_host_curve(curve: &mut HostCurve, minimum_duty_percent: u8) {
    // Service-owned policies may have any supported 2..=64-point domain.
    // Reconciliation must not replace one with a default after Stop or clamp
    // the selection to 40 points during every running-policy refresh.
    if !(2..=64).contains(&curve.points.len())
        || curve
            .points
            .iter()
            .any(|point| !(0..=120_000).contains(&point.temperature_millidegrees))
        || curve
            .points
            .windows(2)
            .any(|points| points[0].temperature_millidegrees >= points[1].temperature_millidegrees)
    {
        *curve = conservative_host_curve(minimum_duty_percent);
        return;
    }
    let minimum = minimum_duty_percent.min(100);
    let mut previous = minimum;
    for point in &mut curve.points {
        point.duty_percent = point.duty_percent.clamp(previous, 100);
        previous = point.duty_percent;
    }
    if let Some(last) = curve.points.last_mut() {
        last.duty_percent = 100;
    }
}

fn policy_from_host_channels(channels: &[HostEditorChannel]) -> HostControlPolicy {
    HostControlPolicy {
        channels: channels
            .iter()
            .map(|channel| HostChannelPolicy {
                channel_id: channel.capability.channel_id.clone(),
                curve: channel.curve.clone(),
            })
            .collect(),
    }
}

fn reconcile_host_baseline(baseline: &mut HostControlPolicy, channels: &[HostEditorChannel]) {
    let mut old = std::mem::take(&mut baseline.channels)
        .into_iter()
        .map(|channel| (channel.channel_id.clone(), channel.curve))
        .collect::<BTreeMap<_, _>>();
    baseline.channels = channels
        .iter()
        .map(|channel| {
            let mut curve = old
                .remove(&channel.capability.channel_id)
                .unwrap_or_else(|| {
                    conservative_host_curve(channel.capability.minimum_duty_percent)
                });
            normalize_host_curve(&mut curve, channel.capability.minimum_duty_percent);
            HostChannelPolicy {
                channel_id: channel.capability.channel_id.clone(),
                curve,
            }
        })
        .collect();
}

fn evaluate_host_curve(curve: &HostCurve, temperature_millidegrees: i32) -> u8 {
    let Some(first) = curve.points.first() else {
        return 0;
    };
    if temperature_millidegrees <= first.temperature_millidegrees {
        return first.duty_percent;
    }
    for points in curve.points.windows(2) {
        let low = points[0];
        let high = points[1];
        if temperature_millidegrees <= high.temperature_millidegrees {
            let span = i64::from(high.temperature_millidegrees - low.temperature_millidegrees);
            let offset = i64::from(temperature_millidegrees - low.temperature_millidegrees);
            let duty_span = i64::from(high.duty_percent) - i64::from(low.duty_percent);
            return u8::try_from(i64::from(low.duty_percent) + duty_span * offset / span)
                .expect("interpolated host duty is bounded");
        }
    }
    curve.points.last().map_or(0, |point| point.duty_percent)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::{DemoBackend, HardwareBackend};

    fn app() -> App {
        let mut backend = DemoBackend::new();
        App::new(backend.refresh().unwrap(), backend.name())
    }

    fn opted_in(mut snapshot: HardwareSnapshot) -> HardwareSnapshot {
        snapshot.monitoring.opted_in = true;
        snapshot
    }

    fn settings_stop(app: &mut App) -> Option<AppCommand> {
        app.snapshot.monitoring.opted_in = true;
        app.snapshot_received = true;
        if app.modal.is_some() {
            app.handle_key_event(key(KeyCode::Esc));
        }
        app.handle_key_event(key(KeyCode::Char('o')));
        assert_eq!(app.modal, Some(Modal::Settings));
        let command = app.handle_key_event(key(KeyCode::Char('t')));
        app.handle_key_event(key(KeyCode::Esc));
        command
    }

    fn key(code: KeyCode) -> KeyEvent {
        KeyEvent::new(code, KeyModifiers::NONE)
    }

    fn live_gated_app() -> App {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot.clone(), "MONITORING SERVICE");
        app.update_telemetry(snapshot);
        app
    }

    #[test]
    fn live_gated_enter_sends_one_complete_selection_and_does_not_fake_opt_in() {
        let mut app = live_gated_app();
        assert!(app.needs_opt_in());
        assert!(app.handle_key_event(key(KeyCode::Tab)).is_none());
        assert_eq!(app.editor_mode, EditorMode::Firmware);
        assert!(app.handle_key_event(key(KeyCode::Char('d'))).is_none());
        assert_eq!(app.modal, None);
        assert!(app.handle_key_event(key(KeyCode::Char('x'))).is_none());
        let command = app.handle_key_event(key(KeyCode::Enter)).unwrap();
        let AppCommand::ActivateMonitoring {
            firmware_curves,
            host_policy,
            display,
        } = command
        else {
            panic!("wrong request")
        };
        assert_eq!(firmware_curves.len(), 2);
        assert!(
            firmware_curves
                .iter()
                .all(|c| c.points.len() == 40 && c.device_id.0 == "kraken-2023")
        );
        assert_eq!(host_policy.unwrap().channels.len(), 2);
        assert_eq!(display.unwrap().mode, app.snapshot.kraken_display.mode);
        assert!(!app.snapshot.monitoring.opted_in);
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        app.activation_result(Ok(vec![
            MonitoringActivationOutcome {
                target: MonitoringActivationTarget::FirmwareCurve {
                    device_id: DeviceId::new("kraken-2023"),
                    channel_id: ChannelId::new("pump"),
                },
                status: MonitoringActivationStatus::Applied,
                error: None,
            },
            MonitoringActivationOutcome {
                target: MonitoringActivationTarget::HostControl {},
                status: MonitoringActivationStatus::Failed,
                error: Some(nzxt_cam_protocol::ErrorMessage::new("fans unavailable")),
            },
        ]));
        assert_eq!(app.status.kind, StatusKind::Error);
        assert!(
            app.status
                .text
                .contains("1 applied, 0 queued (not uploaded), 1 not applied")
        );
        assert!(app.status.text.contains("motherboard: failed"));
        assert!(app.needs_opt_in());
        let mut next = app.snapshot.clone();
        next.monitoring.opted_in = true;
        app.update_telemetry(next);
        assert!(!app.needs_opt_in());
    }

    #[test]
    fn definite_activation_failure_allows_explicit_retry_only_after_refresh() {
        let mut app = live_gated_app();
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::ActivateMonitoring { .. })
        ));
        app.activation_result(Err(crate::backend::BackendError::with_kind(
            crate::backend::BackendErrorKind::Unavailable,
            "request not sent",
        )));
        assert!(!app.activation_attempted);
        assert!(!app.snapshot_received);
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        let fresh = app.snapshot.clone();
        app.update_telemetry(fresh);
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::ActivateMonitoring { .. })
        ));
    }

    #[test]
    fn uncertain_activation_stays_blocked_after_refresh() {
        let mut app = live_gated_app();
        app.handle_key_event(key(KeyCode::Enter));
        app.activation_result(Err(crate::backend::BackendError::with_kind(
            crate::backend::BackendErrorKind::UnknownOutcome,
            "reply lost",
        )));
        let fresh = app.snapshot.clone();
        app.update_telemetry(fresh);
        assert!(app.activation_attempted);
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
    }

    #[test]
    fn activation_uses_valid_variable_length_host_draft_and_skips_invalid_aio() {
        let mut app = live_gated_app();
        app.snapshot.devices[0].cooling_channels[0].points.pop();
        let channel = &mut app.host_editor.channels[0].curve;
        channel.points = vec![
            HostCurvePoint {
                temperature_millidegrees: 20_000,
                duty_percent: 40,
            },
            HostCurvePoint {
                temperature_millidegrees: 60_000,
                duty_percent: 70,
            },
            HostCurvePoint {
                temperature_millidegrees: 100_000,
                duty_percent: 100,
            },
        ];
        let Some(AppCommand::ActivateMonitoring {
            firmware_curves,
            host_policy,
            ..
        }) = app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("expected activation")
        };
        assert_eq!(firmware_curves.len(), 1);
        assert_eq!(firmware_curves[0].channel_id.0, "fan");
        assert_eq!(host_policy.unwrap().channels[0].curve.points.len(), 3);
    }

    #[test]
    fn explicit_settings_stop_remains_available_with_unopted_service_if_board_is_running() {
        let mut app = live_gated_app();
        app.host_control_state = HostControlState::Running;
        app.handle_key_event(key(KeyCode::Char('o')));
        assert_eq!(
            app.handle_key_event(key(KeyCode::Char('t'))),
            Some(AppCommand::StopHostControl)
        );
        assert!(!app.snapshot.monitoring.opted_in);
    }

    #[test]
    fn queued_display_is_never_reported_applied() {
        let mut app = live_gated_app();
        app.activation_result(Ok(vec![MonitoringActivationOutcome {
            target: MonitoringActivationTarget::Display {
                device_id: DeviceId::new("lcd"),
            },
            status: MonitoringActivationStatus::Pending,
            error: None,
        }]));
        assert_eq!(app.status.kind, StatusKind::Info);
        assert!(
            app.status
                .text
                .contains("0 applied, 1 queued (not uploaded)")
        );
        assert!(app.status.text.contains("display: queued, not uploaded"));
        assert!(app.needs_opt_in());
    }

    #[test]
    fn empty_selection_and_settings_do_not_start_control() {
        let mut app = live_gated_app();
        app.snapshot.devices.clear();
        app.snapshot.kraken_display.device_id = None;
        app.snapshot.host_control.channels.clear();
        app.host_editor.channels.clear();
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.status.kind, StatusKind::Error);
        assert!(!app.activation_attempted);
        app.handle_key_event(key(KeyCode::Char('o')));
        assert_eq!(app.modal, Some(Modal::Settings));
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Char('c'))),
            Some(AppCommand::SaveConfirmApply { .. })
        ));
        assert!(app.handle_key_event(key(KeyCode::Char('a'))).is_none());
        assert!(app.handle_key_event(key(KeyCode::Char('t'))).is_none());
        app.handle_key_event(key(KeyCode::Esc));
        let mut fresh = app.snapshot.clone();
        fresh.monitoring.opted_in = true;
        app.update_telemetry(fresh);
        app.handle_key_event(key(KeyCode::Char('o')));
        assert_eq!(
            app.handle_key_event(key(KeyCode::Char('a'))),
            Some(AppCommand::SetMonitoringAutoResume { enabled: true })
        );
        assert!(!app.snapshot.monitoring.auto_resume);
    }

    #[test]
    fn display_picker_targets_service_device_and_does_not_guess_selected_mode() {
        let mut app = app();
        let target = app.snapshot.kraken_display.device_id.clone().unwrap();
        assert_eq!(app.handle_key_event(key(KeyCode::Char('d'))), None);
        assert_eq!(app.modal, Some(Modal::KrakenDisplay));
        app.handle_key_event(key(KeyCode::Down));
        assert_eq!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::SetKrakenDisplay {
                device_id: target,
                mode: KrakenDisplayMode::ALL[1],
            })
        );
        assert_eq!(
            app.snapshot.kraken_display.mode,
            KrakenDisplayMode::BuiltinLiquid
        );
        app.display_operation_busy = true;
        app.handle_key_event(key(KeyCode::Char('d')));
        app.handle_key_event(key(KeyCode::Down));
        assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
        app.display_change_succeeded();
        assert_eq!(app.status.kind, StatusKind::Success);
        assert_eq!(app.status.text, "Kraken display changed");
        let mut fresh = app.snapshot.clone();
        fresh.kraken_display.mode = KrakenDisplayMode::ALL[1];
        app.update_telemetry(fresh);
        assert_eq!(app.snapshot.kraken_display.mode, KrakenDisplayMode::ALL[1]);
    }

    #[test]
    fn display_error_survives_reconnect_and_clears_when_service_recovers() {
        let mut app = app();
        app.snapshot.kraken_display.last_error = Some("USB busy".into());
        app.backend_connected();
        assert!(app.status.text.contains("Kraken display: USB busy"));
        let mut fresh = app.snapshot.clone();
        fresh.kraken_display.last_error = None;
        app.update_telemetry(fresh);
        assert_eq!(app.status.text, "Kraken display updates resumed");
    }

    #[test]
    fn display_picker_closes_on_disconnect_or_replacement() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::Char('d')));
        let mut fresh = app.snapshot.clone();
        fresh.kraken_display.device_id = Some(DeviceId::new("replacement"));
        app.update_telemetry(fresh);
        assert_eq!(app.modal, None);
        assert!(app.status.text.contains("target changed"));

        app.handle_key_event(key(KeyCode::Char('d')));
        let mut fresh = app.snapshot.clone();
        fresh.kraken_display.device_id = None;
        app.update_telemetry(fresh);
        assert_eq!(app.modal, None);
        assert_eq!(app.handle_key_event(key(KeyCode::Char('d'))), None);
        assert_eq!(app.modal, None);
    }

    #[test]
    fn arrows_select_and_adjust_curve_points() {
        let mut app = app();
        let original = app.selected_curve_point().unwrap();

        app.handle_key_event(key(KeyCode::Right));
        assert_eq!(app.selected_point, 1);
        app.handle_key_event(key(KeyCode::Up));

        assert_eq!(app.selected_curve_point().unwrap().duty, original.duty + 1);
        assert!(app.is_active_curve_dirty());
    }

    #[test]
    fn shift_arrow_adjusts_duty_by_five_points() {
        let mut app = app();
        let original = app.selected_curve_point().unwrap().duty;
        let event = KeyEvent::new(KeyCode::Up, KeyModifiers::SHIFT);

        app.handle_key_event(event);

        assert_eq!(app.selected_curve_point().unwrap().duty, original + 5);
    }

    #[test]
    fn duty_changes_are_clamped_to_channel_limits() {
        let mut app = app();
        for _ in 0..200 {
            app.handle_key_event(key(KeyCode::Down));
        }
        assert_eq!(app.selected_curve_point().unwrap().duty, 20);

        for _ in 0..200 {
            app.handle_key_event(key(KeyCode::Up));
        }
        assert_eq!(app.selected_curve_point().unwrap().duty, 100);
    }

    #[test]
    fn service_status_uses_plain_monitoring_service_wording() {
        let mut demo = DemoBackend::new();
        let snapshot = demo.refresh().unwrap();
        let mut app = App::with_runtime_config(
            opted_in(snapshot),
            "MONITORING SERVICE",
            vec![],
            AppConfig::default(),
        );
        assert_eq!(app.status.text, "Connecting to monitoring service…");
        app.backend_connected();
        assert_eq!(app.status.text, "Monitoring service connected");
        app.handle_key_event(key(KeyCode::Up));
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Char('y'))),
            Some(AppCommand::ApplyCurve { .. })
        ));
        assert_eq!(app.status.text, "Sending curve to monitoring service…");
        assert!(!app.status.text.to_lowercase().contains("live"));
        assert!(!app.status.text.to_lowercase().contains("backend"));
    }

    fn firmware_edit_fixture(channel_index: usize) -> (App, HardwareSnapshot) {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        for point in &mut snapshot.devices[0].cooling_channels[channel_index].points {
            point.duty = match point.temperature {
                20..=29 => 40,
                30..=32 => 50,
                33..=35 => 60,
                36..=58 => 80,
                _ => 100,
            };
        }
        snapshot.monitoring.opted_in = true;
        let mut app = App::new(snapshot.clone(), "LIVE-TEST");
        app.selected_channel = channel_index;
        (app, snapshot)
    }

    #[test]
    fn aio_raise_and_lower_propagate_only_to_conflicting_neighbors() {
        for channel_index in [0, 1] {
            for (index, code, changed, value, message) in [
                (10, KeyCode::Up, 10..13, 55, "2 hotter points raised"),
                (15, KeyCode::Down, 13..16, 55, "2 cooler points lowered"),
            ] {
                let (mut app, before) = firmware_edit_fixture(channel_index);
                let host_before = app.host_editor.policy();
                let mut expected = before.devices.clone();
                for point in &mut expected[0].cooling_channels[channel_index].points[changed] {
                    point.duty = value;
                }
                app.selected_point = index;
                assert!(
                    app.handle_key_event(KeyEvent::new(code, KeyModifiers::SHIFT))
                        .is_none()
                );
                assert_eq!(app.snapshot.devices, expected);
                assert_eq!(app.host_editor.policy(), host_before);
                assert_eq!(app.selected_channel, channel_index);
                assert_eq!(app.selected_point, index);
                assert!(app.status.text.contains(message));
                assert_eq!(app.dirty_curve_count(), 1);
                assert!(
                    app.active_channel()
                        .unwrap()
                        .points
                        .windows(2)
                        .all(|p| p[0].duty <= p[1].duty)
                );
            }
        }
    }

    #[test]
    fn aio_propagation_keeps_channel_bounds_and_failsafe_for_every_editable_point() {
        for channel_index in [0, 1] {
            for index in 0..39 {
                for (code, modifier) in [
                    (KeyCode::Up, KeyModifiers::NONE),
                    (KeyCode::Down, KeyModifiers::NONE),
                    (KeyCode::Up, KeyModifiers::SHIFT),
                    (KeyCode::Down, KeyModifiers::SHIFT),
                ] {
                    let (mut app, _) = firmware_edit_fixture(channel_index);
                    app.selected_point = index;
                    assert!(
                        app.handle_key_event(KeyEvent::new(code, modifier))
                            .is_none()
                    );
                    let curve = app.active_channel().unwrap();
                    assert!(curve.points.windows(2).all(|p| p[0].duty <= p[1].duty));
                    assert!(
                        curve
                            .points
                            .iter()
                            .all(|p| (curve.min_duty..=curve.max_duty).contains(&p.duty))
                    );
                    assert_eq!(curve.points.last().unwrap().duty, 100);
                    assert!(
                        curve
                            .points
                            .iter()
                            .enumerate()
                            .all(|(i, p)| usize::from(p.temperature) == 20 + i)
                    );
                }
            }
        }
    }

    #[test]
    fn aio_propagation_uses_firmware_limits_not_host_minimum() {
        for channel_index in [0, 1] {
            let (mut app, _) = firmware_edit_fixture(channel_index);
            app.selected_point = 15;
            for _ in 0..30 {
                app.handle_key_event(KeyEvent::new(KeyCode::Down, KeyModifiers::SHIFT));
            }
            let curve = app.active_channel().unwrap();
            let minimum = if channel_index == 0 { 20 } else { 0 };
            assert_eq!(curve.min_duty, minimum);
            assert!(curve.points[..=15].iter().all(|p| p.duty == minimum));
            assert!(curve.points[16..39].iter().all(|p| p.duty == 80));
            let before = curve.clone();
            app.handle_key_event(key(KeyCode::Down));
            assert_eq!(app.active_channel().unwrap(), &before);
            for _ in 0..30 {
                app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::SHIFT));
            }
            assert!(
                app.active_channel().unwrap().points[15..]
                    .iter()
                    .all(|p| p.duty == 100)
            );
        }
        let (mut app, mut snapshot) = firmware_edit_fixture(0);
        snapshot.devices[0].cooling_channels[0].max_duty = 80;
        app.update_telemetry(snapshot);
        for _ in 0..30 {
            app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::SHIFT));
        }
        assert!(
            app.active_channel()
                .unwrap()
                .points
                .iter()
                .all(|p| p.duty == 80)
        );
    }

    #[test]
    fn host_and_aio_follow_identical_edits_when_channel_limits_match() {
        let (mut aio, _) = firmware_edit_fixture(0);
        aio.snapshot.devices[0].cooling_channels[0].min_duty = 30;
        let mut host = app();
        host.handle_key_event(key(KeyCode::BackTab));
        for (point, firmware) in host.host_editor.channels[0]
            .curve
            .points
            .iter_mut()
            .zip(&aio.active_channel().unwrap().points)
        {
            point.duty_percent = firmware.duty;
        }
        for index in 0..40 {
            aio.selected_point = index;
            host.host_editor.selected_point = index;
            for code in [KeyCode::Up, KeyCode::Down, KeyCode::Down] {
                let event = KeyEvent::new(code, KeyModifiers::SHIFT);
                assert_eq!(aio.handle_key_event(event), None);
                assert_eq!(host.handle_key_event(event), None);
                let firmware: Vec<_> = aio
                    .active_channel()
                    .unwrap()
                    .points
                    .iter()
                    .map(|p| p.duty)
                    .collect();
                let motherboard: Vec<_> = host
                    .selected_host_channel()
                    .unwrap()
                    .curve
                    .points
                    .iter()
                    .map(|p| p.duty_percent)
                    .collect();
                assert_eq!(firmware, motherboard, "point {index}, {code:?}");
            }
        }
    }

    #[test]
    fn aio_failsafe_is_locked_and_an_invalid_endpoint_can_be_repaired() {
        for channel_index in [0, 1] {
            let (mut app, mut snapshot) = firmware_edit_fixture(channel_index);
            app.handle_key_event(key(KeyCode::End));
            let before = app.active_channel().unwrap().clone();
            for code in [KeyCode::Up, KeyCode::Down] {
                assert!(
                    app.handle_key_event(KeyEvent::new(code, KeyModifiers::SHIFT))
                        .is_none()
                );
                assert_eq!(app.active_channel().unwrap(), &before);
                assert!(!app.is_active_curve_dirty());
                assert!(app.status.text.contains("locked at 100%"));
            }
            snapshot.devices[0].cooling_channels[channel_index]
                .points
                .last_mut()
                .unwrap()
                .duty = 99;
            let mut app = App::new(snapshot, "LIVE-TEST");
            app.selected_channel = channel_index;
            app.handle_key_event(key(KeyCode::End));
            app.handle_key_event(key(KeyCode::Up));
            assert_eq!(app.selected_curve_point().unwrap().duty, 100);
            assert!(app.is_active_curve_dirty());
        }
    }

    #[test]
    fn aio_propagated_drafts_survive_refresh_and_still_require_apply_confirmation() {
        for channel_index in [0, 1] {
            let (mut app, snapshot) = firmware_edit_fixture(channel_index);
            app.confirm_apply = true;
            app.selected_point = 10;
            app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::SHIFT));
            let points = app.active_channel().unwrap().points.clone();
            let key_expected = app.active_curve_key().unwrap();
            app.update_telemetry(snapshot);
            assert_eq!(app.active_channel().unwrap().points, points);
            assert!(app.is_active_curve_dirty());
            assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
            assert_eq!(app.modal, Some(Modal::ConfirmApply));
            assert_eq!(app.handle_key_event(key(KeyCode::Char('n'))), None);
            assert_eq!(app.active_channel().unwrap().points, points);
            assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
            let Some(AppCommand::ApplyCurve {
                key: actual,
                points: submitted,
                ..
            }) = app.handle_key_event(key(KeyCode::Char('y')))
            else {
                panic!("expected only the confirmed firmware curve");
            };
            assert_eq!(actual, key_expected);
            assert_eq!(submitted, points);
            app.apply_succeeded(actual, submitted);
            assert!(!app.is_active_curve_dirty());
        }
    }

    #[test]
    fn tab_cycles_cooling_devices_and_c_cycles_channels() {
        let mut app = app();
        let first_device = app.active_device().unwrap().id.clone();
        app.handle_key_event(key(KeyCode::Tab));
        assert_ne!(app.active_device().unwrap().id, first_device);

        let first_channel = app.active_channel().unwrap().id.clone();
        app.handle_key_event(key(KeyCode::Char('c')));
        assert_ne!(app.active_channel().unwrap().id, first_channel);
    }

    #[test]
    fn tab_and_shift_tab_include_host_view_in_both_directions() {
        let mut app = app();
        let count = app.cooling_device_indices().len();
        assert!(count > 1);
        for index in 1..count {
            assert!(app.handle_key_event(key(KeyCode::Tab)).is_none());
            assert_eq!(app.editor_mode, EditorMode::Firmware);
            assert_eq!(app.selected_cooling_device, index);
        }
        assert!(app.handle_key_event(key(KeyCode::Tab)).is_none());
        assert_eq!(app.editor_mode, EditorMode::Host);
        assert!(app.handle_key_event(key(KeyCode::Tab)).is_none());
        assert_eq!(app.editor_mode, EditorMode::Firmware);
        assert_eq!(app.selected_cooling_device, 0);
        assert!(app.handle_key_event(key(KeyCode::BackTab)).is_none());
        assert_eq!(app.editor_mode, EditorMode::Host);
        for index in (0..count).rev() {
            assert!(app.handle_key_event(key(KeyCode::BackTab)).is_none());
            assert_eq!(app.editor_mode, EditorMode::Firmware);
            assert_eq!(app.selected_cooling_device, index);
        }
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(app.editor_mode, EditorMode::Host);
    }

    #[test]
    fn one_kraken_and_disabled_airflow_are_both_reachable_without_control_commands() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot
            .devices
            .retain(|device| device.id.0 == "kraken-2023" || device.cooling_channels.is_empty());
        snapshot.host_control = Default::default();
        let mut app = App::new(snapshot, "DEMO");
        let kraken = app.active_device().unwrap().id.clone();
        app.handle_key_event(key(KeyCode::Char('c')));
        let fan = app.active_channel().unwrap().id.clone();
        assert_eq!(fan.0, "fan");
        app.handle_key_event(key(KeyCode::Right));
        let point = app.selected_point;
        assert_eq!(app.cooling_device_indices().len(), 1);
        assert!(app.handle_key_event(key(KeyCode::Tab)).is_none());
        assert_eq!(app.editor_mode, EditorMode::Host);
        assert!(app.host_editor.channels.is_empty());
        assert!(app.status.text.contains("disabled"));
        for code in [
            KeyCode::Enter,
            KeyCode::Up,
            KeyCode::Down,
            KeyCode::Char('s'),
            KeyCode::Char('r'),
        ] {
            assert!(app.handle_key_event(key(code)).is_none());
        }
        assert_eq!(app.host_control_state, HostControlState::Disabled);
        assert!(app.host_editor.channels.is_empty());
        app.handle_key_event(key(KeyCode::Tab));
        assert_eq!(app.editor_mode, EditorMode::Firmware);
        assert_eq!(app.active_device().unwrap().id, kraken);
        assert_eq!(app.active_channel().unwrap().id, fan);
        assert_eq!(app.selected_point, point);
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(app.editor_mode, EditorMode::Host);
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(app.editor_mode, EditorMode::Firmware);
        assert_eq!(app.active_channel().unwrap().id, fan);
        assert_eq!(app.selected_point, point);
        for code in [KeyCode::Char('h'), KeyCode::Char('H')] {
            assert!(app.handle_key_event(key(code)).is_none());
            assert_eq!(app.editor_mode, EditorMode::Firmware);
            assert_eq!(app.active_channel().unwrap().id, fan);
        }
    }

    #[test]
    fn empty_unknown_and_disappearing_controls_remain_navigable_without_start() {
        let empty = HardwareSnapshot {
            devices: Vec::new(),
            sequence: 0,
            host_control: Default::default(),
            kraken_display: Default::default(),
            monitoring: Default::default(),
        };
        let mut app = App::new(empty.clone(), "DEMO");
        for _ in 0..2 {
            app.handle_key_event(key(KeyCode::Tab));
            assert_eq!(app.editor_mode, EditorMode::Host);
            assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
            app.handle_key_event(key(KeyCode::Tab));
            assert_eq!(app.editor_mode, EditorMode::Firmware);
        }
        app.mark_host_status_unknown();
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(app.editor_mode, EditorMode::Host);
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(!app.host_status_trusted);
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(app.editor_mode, EditorMode::Firmware);

        let mut backend = DemoBackend::new();
        app.update_telemetry(backend.refresh().unwrap());
        app.handle_key_event(key(KeyCode::BackTab));
        assert!(!app.host_editor.channels.is_empty());
        app.update_telemetry(empty);
        assert_eq!(app.editor_mode, EditorMode::Host);
        assert!(app.host_editor.channels.is_empty());
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        // Even an inconsistent Available snapshot may not submit an empty policy.
        app.host_control_state = HostControlState::Available;
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
    }

    #[test]
    fn host_r_resets_only_the_draft_curve_without_starting_control() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        let original = app.selected_host_channel().unwrap().curve.clone();
        let other = app.host_editor.channels[1].curve.clone();
        app.handle_key_event(key(KeyCode::Up));
        app.handle_key_event(key(KeyCode::Char('s')));
        assert_ne!(app.selected_host_channel().unwrap().curve, original);
        assert!(app.handle_key_event(key(KeyCode::Char('r'))).is_none());
        let channel = app.selected_host_channel().unwrap();
        assert_eq!(
            channel.curve,
            conservative_host_curve(channel.capability.minimum_duty_percent)
        );
        assert_eq!(app.host_editor.channels[1].curve, other);
        assert_eq!(app.host_control_state, HostControlState::Available);
    }

    #[test]
    fn reset_restores_last_applied_curve() {
        let mut app = app();
        let original = app.selected_curve_point().unwrap();
        app.handle_key_event(key(KeyCode::Up));
        assert!(app.is_active_curve_dirty());

        app.handle_key_event(key(KeyCode::Char('r')));
        assert_eq!(app.selected_curve_point().unwrap(), original);
        assert!(!app.is_active_curve_dirty());
    }

    #[test]
    fn live_apply_requires_explicit_yes_confirmation() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.devices[0].cooling_channels[0].curve_state = CurveState::Unverified;
        let mut app = App::with_runtime_config(
            opted_in(snapshot),
            "LIQUIDCTL",
            Vec::new(),
            AppConfig::default(),
        );

        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.modal, Some(Modal::ConfirmApply));
        assert!(app.handle_key_event(key(KeyCode::Char('n'))).is_none());
        assert_eq!(app.modal, None);

        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        let command = app.handle_key_event(key(KeyCode::Char('y')));
        assert!(matches!(command, Some(AppCommand::ApplyCurve { .. })));
        assert_eq!(app.modal, None);
    }

    #[test]
    fn firmware_confirmation_rejects_stale_target_curve_and_capabilities() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        for scenario in [
            "removed", "offline", "minimum", "maximum", "source", "points",
        ] {
            let mut app = App::with_runtime_config(
                opted_in(snapshot.clone()),
                "LIVE",
                vec![],
                AppConfig::default(),
            );
            assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
            assert_eq!(app.modal, Some(Modal::ConfirmApply), "{scenario}");
            let mut fresh = opted_in(snapshot.clone());
            match scenario {
                "removed" => {
                    fresh.devices[0].cooling_channels.remove(0);
                }
                "offline" => fresh.devices[0].online = false,
                "minimum" => fresh.devices[0].cooling_channels[0].min_duty += 1,
                "maximum" => fresh.devices[0].cooling_channels[0].max_duty -= 1,
                "source" => fresh.devices[0].cooling_channels[0].source = TemperatureSource::Cpu,
                "points" => fresh.devices[0].cooling_channels[0].points[0].duty += 1,
                _ => unreachable!(),
            }
            app.update_telemetry(fresh);
            assert_eq!(
                app.handle_key_event(key(KeyCode::Char('y'))),
                None,
                "{scenario}"
            );
            assert_eq!(app.modal, None, "{scenario}");
            assert!(app.pending_confirmation.is_none(), "{scenario}");
            assert!(app.status.text.contains("changed"), "{scenario}");
        }
        let mut app = App::with_runtime_config(
            opted_in(snapshot.clone()),
            "LIVE",
            vec![],
            AppConfig::default(),
        );
        let original = snapshot.devices[0].cooling_channels[0].points.clone();
        assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
        app.update_telemetry(opted_in(snapshot));
        assert!(matches!(app.handle_key_event(key(KeyCode::Char('Y'))),
            Some(AppCommand::ApplyCurve { points, .. }) if points == original));
        assert!(app.pending_confirmation.is_none());
    }

    #[test]
    fn offline_firmware_never_opens_confirmation() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.devices[0].online = false;
        let mut app =
            App::with_runtime_config(opted_in(snapshot), "LIVE", vec![], AppConfig::default());
        assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
        assert_eq!(app.modal, None);
        assert!(app.pending_confirmation.is_none());
    }

    #[test]
    fn configured_profile_becomes_unverified_startup_template() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.devices[0].cooling_channels[0].curve_state = CurveState::Unverified;
        let key = (
            snapshot.devices[0].id.clone(),
            snapshot.devices[0].cooling_channels[0].id.clone(),
        );
        let config = AppConfig {
            confirm_apply: true,
            default_profiles: vec![crate::config::DefaultProfile {
                device_id: key.0.0.clone(),
                channel_id: key.1.0.clone(),
                profile: "Silent".into(),
            }],
        };

        let mut app = App::with_runtime_config(
            HardwareSnapshot {
                devices: Vec::new(),
                sequence: 0,
                host_control: Default::default(),
                kraken_display: Default::default(),
                monitoring: Default::default(),
            },
            "LIQUIDCTL",
            Vec::new(),
            config,
        );
        app.update_telemetry(snapshot);

        assert_eq!(app.active_profile_name(), "Silent");
        assert_eq!(app.selected_curve_point().unwrap().duty, 20);
        assert!(!app.active_curve_is_verified());
        assert!(!app.is_active_curve_dirty());
    }

    #[test]
    fn offline_device_keeps_edits_but_does_not_emit_apply_command() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.devices[0].online = false;
        let mut app = App::new(opted_in(snapshot), "LIQUIDCTL");
        app.handle_key_event(key(KeyCode::Up));

        let command = app.handle_key_event(key(KeyCode::Enter));

        assert!(command.is_none());
        assert!(app.is_active_curve_dirty());
        assert!(app.status.text.contains("offline"));
    }

    #[test]
    fn enter_creates_an_apply_command() {
        let mut app = app();
        let command = app.handle_key_event(key(KeyCode::Enter));
        assert!(matches!(command, Some(AppCommand::ApplyCurve { .. })));
    }

    #[test]
    fn apply_acknowledgement_does_not_clear_newer_edits() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::Up));
        let AppCommand::ApplyCurve {
            key: curve_key,
            points,
            ..
        } = app.handle_key_event(key(KeyCode::Enter)).unwrap()
        else {
            panic!("expected apply command");
        };
        app.handle_key_event(key(KeyCode::Up));

        app.apply_succeeded(curve_key, points);

        assert!(app.is_active_curve_dirty());
        assert_eq!(app.selected_curve_point().unwrap().duty, 47);
        app.handle_key_event(key(KeyCode::Char('r')));
        assert_eq!(app.selected_curve_point().unwrap().duty, 46);
    }

    #[test]
    fn uncertain_write_failure_marks_curve_unverified() {
        let mut app = app();
        let key = app.active_curve_key().unwrap();
        let points = app.active_channel().unwrap().points.clone();
        app.apply_succeeded(key.clone(), points);
        assert!(app.active_curve_is_verified());

        app.apply_outcome_unknown(&key, "write outcome unknown");

        assert!(!app.active_curve_is_verified());
        assert_eq!(app.status.kind, StatusKind::Error);
    }

    #[test]
    fn session_loss_invalidates_every_curve_without_changing_editor_state() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::Up));
        app.handle_key_event(key(KeyCode::Right));
        app.handle_key_event(key(KeyCode::Tab));
        app.handle_key_event(key(KeyCode::Char('c')));
        let points = app
            .snapshot
            .devices
            .iter()
            .flat_map(|device| &device.cooling_channels)
            .map(|channel| channel.points.clone())
            .collect::<Vec<_>>();
        let dirty_count = app.dirty_curve_count();
        let selection = (
            app.selected_cooling_device,
            app.selected_channel,
            app.selected_point,
        );
        let profile = app.active_profile_name().to_owned();

        app.invalidate_live_curve_claims();

        assert!(
            app.snapshot
                .devices
                .iter()
                .flat_map(|device| &device.cooling_channels)
                .all(|channel| channel.curve_state == CurveState::Unverified)
        );
        assert_eq!(
            app.snapshot
                .devices
                .iter()
                .flat_map(|device| &device.cooling_channels)
                .map(|channel| channel.points.clone())
                .collect::<Vec<_>>(),
            points
        );
        assert_eq!(app.dirty_curve_count(), dirty_count);
        assert_eq!(
            (
                app.selected_cooling_device,
                app.selected_channel,
                app.selected_point,
            ),
            selection
        );
        assert_eq!(app.active_profile_name(), profile);
    }

    #[test]
    fn telemetry_refresh_reconciles_topology_and_preserves_dirty_points() {
        let mut backend = DemoBackend::new();
        let initial = backend.refresh().unwrap();
        let mut app = App::new(initial, backend.name());
        app.handle_key_event(key(KeyCode::Up));
        let dirty_point = app.selected_curve_point().unwrap();

        let mut fresh = backend.refresh().unwrap();
        let kraken = fresh
            .devices
            .iter_mut()
            .find(|device| device.id.0 == "kraken-2023")
            .unwrap();
        let pump = kraken
            .cooling_channels
            .iter_mut()
            .find(|channel| channel.id.0 == "pump")
            .unwrap();
        pump.source = nzxt_cam_core::TemperatureSource::Gpu;
        pump.min_duty = 40;
        pump.max_duty = 80;
        for point in &mut pump.points {
            point.duty = 70;
        }
        let mut extra = kraken.cooling_channels[1].clone();
        extra.id = ChannelId::new("aux-fan");
        extra.name = "Auxiliary fan".into();
        kraken.cooling_channels.push(extra);
        fresh
            .devices
            .retain(|device| device.id.0 != "fan-controller-2022");

        app.update_telemetry(fresh);

        assert_eq!(app.cooling_device_indices().len(), 1);
        assert_eq!(app.active_device().unwrap().id.0, "kraken-2023");
        assert_eq!(app.active_device().unwrap().cooling_channels.len(), 3);
        assert_eq!(
            app.active_channel().unwrap().source,
            nzxt_cam_core::TemperatureSource::Gpu
        );
        assert_eq!(app.active_channel().unwrap().max_duty, 80);
        assert_eq!(app.selected_curve_point().unwrap(), dirty_point);
        assert!(app.is_active_curve_dirty());
    }

    #[test]
    fn telemetry_refresh_adopts_reported_curve_when_limit_change_erases_edit() {
        let mut backend = DemoBackend::new();
        let initial = backend.refresh().unwrap();
        let mut app = App::new(initial, backend.name());
        app.handle_key_event(key(KeyCode::Up));
        assert!(app.is_active_curve_dirty());

        let mut fresh = backend.refresh().unwrap();
        let pump = &mut fresh.devices[0].cooling_channels[0];
        pump.min_duty = 50;
        pump.max_duty = 80;
        for point in &mut pump.points {
            point.duty = 70;
        }

        app.update_telemetry(fresh);

        assert_eq!(app.selected_curve_point().unwrap().duty, 70);
        assert!(!app.is_active_curve_dirty());
    }

    #[test]
    fn telemetry_refresh_moves_selection_when_active_channel_disappears() {
        let mut backend = DemoBackend::new();
        let initial = backend.refresh().unwrap();
        let mut app = App::new(initial, backend.name());
        let mut fresh = backend.refresh().unwrap();
        fresh.devices[0]
            .cooling_channels
            .retain(|channel| channel.id.0 == "fan");

        app.update_telemetry(fresh);

        assert_eq!(app.active_channel().unwrap().id.0, "fan");
    }

    #[test]
    fn profile_picker_loads_and_applies_selected_preset() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::Char('p')));
        assert_eq!(app.modal, Some(Modal::Profiles));
        assert_eq!(
            app.profiles
                .iter()
                .map(|profile| profile.name.as_str())
                .collect::<Vec<_>>(),
            vec!["Silent", "Progressive", "Performance"]
        );
        app.handle_key_event(key(KeyCode::Home));

        let command = app.handle_key_event(key(KeyCode::Enter));

        assert!(matches!(
            command,
            Some(AppCommand::ApplyCurve {
                profile: Some(ref name),
                ..
            }) if *name == ProfileRef::BuiltIn(BuiltinProfile::Silent)
        ));
        assert_eq!(app.active_profile_name(), "Silent");
        assert_eq!(app.modal, None);
        assert_eq!(app.selected_curve_point().unwrap().duty, 20);
    }

    fn load_silent(app: &mut App) {
        assert!(app.handle_key_event(key(KeyCode::Char('p'))).is_none());
        app.handle_key_event(key(KeyCode::Home));
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::ApplyCurve { profile: Some(name), .. }) if name == ProfileRef::BuiltIn(BuiltinProfile::Silent)
        ));
    }

    #[test]
    fn firmware_profile_label_tracks_working_curve_without_losing_reference_or_apply_metadata() {
        let mut app = app();
        assert_eq!(app.active_profile_name(), "Custom");
        load_silent(&mut app);
        let curve_key = app.active_curve_key().unwrap();
        let original = app.active_channel().unwrap().points.clone();
        assert_eq!(app.active_profile_name(), "Silent");
        assert!(!app.active_profile_is_modified());

        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Down)); // locked failsafe is a no-op
        assert_eq!(app.active_profile_name(), "Silent");
        app.handle_key_event(key(KeyCode::Home));
        for _ in 0..100 {
            app.handle_key_event(key(KeyCode::Down)); // minimum is clamped
        }
        assert_eq!(app.active_profile_name(), "Silent");
        app.handle_key_event(key(KeyCode::Right));
        app.handle_key_event(key(KeyCode::Up));
        assert_eq!(app.active_profile_name(), "Custom");
        assert!(app.active_profile_is_modified());
        assert_eq!(
            app.active_profiles.get(&curve_key),
            Some(&ProfileRef::BuiltIn(BuiltinProfile::Silent))
        );
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::ApplyCurve { profile: None, .. })
        ));
        app.handle_key_event(key(KeyCode::Down));
        assert_eq!(app.active_channel().unwrap().points, original);
        assert_eq!(app.active_profile_name(), "Silent");
        assert!(!app.active_profile_is_modified());
        assert!(matches!(app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::ApplyCurve { profile: Some(name), .. }) if name == ProfileRef::BuiltIn(BuiltinProfile::Silent)));
        app.handle_key_event(key(KeyCode::Up));
        assert_eq!(app.active_profile_name(), "Custom");
        app.handle_key_event(key(KeyCode::Char('r')));
        assert_eq!(app.active_profile_name(), "Custom"); // reset is the actual applied curve
        assert!(!app.is_active_curve_dirty());
        load_silent(&mut app);
        assert_eq!(app.active_profile_name(), "Silent");
    }

    #[test]
    fn named_custom_profile_and_other_channel_keep_independent_labels_across_refresh() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot.clone(), "DEMO");
        load_silent(&mut app);
        let curve_key = app.active_curve_key().unwrap();
        app.handle_key_event(key(KeyCode::Up));
        let named =
            CurveProfile::custom("Night mode", app.active_channel().unwrap().points.clone())
                .unwrap();
        let mut library = app.profile_library.clone();
        let bindings = library
            .prepare(&[ProfileIntent {
                target: ProfileTarget::Firmware {
                    device_id: curve_key.0.clone(),
                    channel_id: curve_key.1.clone(),
                },
                curve: ProfileCurve::Firmware(named.points.clone()),
                preferred: None,
            }])
            .unwrap();
        let ProfileRef::Custom(id) = bindings[0].profile else {
            panic!("custom expected")
        };
        library.rename(id, "Night mode").unwrap();
        library.bind_success(&bindings).unwrap();
        app.set_profile_library(library);
        app.profile_bindings_applied(&bindings);
        assert_eq!(app.active_profile_name(), "Night mode");
        assert!(!app.active_profile_is_modified());
        app.handle_key_event(key(KeyCode::Char('c')));
        let other_name = app.active_profile_name().to_owned();
        app.handle_key_event(key(KeyCode::Char('[')));
        app.handle_key_event(key(KeyCode::Up));
        assert_eq!(app.active_profile_name(), "Custom");
        assert!(app.active_profile_is_modified());
        app.update_telemetry(snapshot.clone());
        assert_eq!(app.active_profile_name(), "Custom");
        assert!(app.active_profile_is_modified());
        app.handle_key_event(key(KeyCode::Char('c')));
        assert_eq!(app.active_profile_name(), other_name);
        app.handle_key_event(key(KeyCode::Char('[')));
        app.handle_key_event(key(KeyCode::Char('r')));
        assert_eq!(app.active_profile_name(), "Custom");
        // A reported exact profile can be selected without altering another channel.
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        let _ = app.handle_key_event(key(KeyCode::Enter));
        assert_eq!(app.active_profile_name(), "Night mode");
        app.update_telemetry(snapshot);
        assert_eq!(app.active_profile_name(), "Night mode");
    }

    #[test]
    fn clamped_firmware_custom_keeps_its_identity_when_applied_on_higher_minimum() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        let raw = built_in_profiles()[0].points.clone();
        snapshot.devices[0].cooling_channels[0].min_duty = 55;
        let custom = CurveProfile::custom("Pump Quiet", raw.clone()).unwrap();
        let mut app = App::with_custom_profiles(snapshot, "DEMO", vec![custom]);
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        let Some(AppCommand::ApplyCurve {
            key: target,
            points,
            profile: Some(reference),
        }) = app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("named apply expected")
        };
        assert_eq!(reference, ProfileRef::Custom(1));
        assert_eq!(app.active_profile_name(), "Pump Quiet");
        assert!(!app.active_profile_is_modified());
        assert!(
            points
                .iter()
                .zip(&raw)
                .all(|(actual, saved)| actual.temperature == saved.temperature
                    && actual.duty == saved.duty.max(55))
        );
        let intents = [ProfileIntent {
            target: ProfileTarget::Firmware {
                device_id: target.0,
                channel_id: target.1,
            },
            curve: ProfileCurve::Firmware(points),
            preferred: Some(reference),
        }];
        let bindings = app.profile_library.prepare(&intents).unwrap();
        assert_eq!(bindings[0].profile, reference);
        assert_eq!(app.profile_library.profiles.len(), 1);
    }

    #[test]
    fn persisted_custom_profile_is_available_in_picker() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let custom =
            CurveProfile::custom("Desk work", built_in_profiles()[0].points.clone()).unwrap();
        let mut app = App::with_custom_profiles(snapshot, backend.name(), vec![custom]);

        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        let command = app.handle_key_event(key(KeyCode::Enter));

        assert!(matches!(command, Some(AppCommand::ApplyCurve { .. })));
        assert_eq!(app.active_profile_name(), "Desk work");
        assert_eq!(app.profiles.len(), 4);
    }

    #[test]
    fn edited_preset_forks_only_after_successful_binding_and_preserves_newer_draft() {
        let mut app = app();
        load_silent(&mut app);
        app.handle_key_event(key(KeyCode::Up));
        let target_key = app.active_curve_key().unwrap();
        let accepted = app.active_channel().unwrap().points.clone();
        assert_eq!(app.active_profile_name(), "Custom");
        let command = app.handle_key_event(key(KeyCode::Enter));
        assert!(matches!(
            command,
            Some(AppCommand::ApplyCurve { profile: None, .. })
        ));
        let mut library = app.profile_library.clone();
        let bindings = library
            .prepare(&[ProfileIntent {
                target: ProfileTarget::Firmware {
                    device_id: target_key.0.clone(),
                    channel_id: target_key.1.clone(),
                },
                curve: ProfileCurve::Firmware(accepted.clone()),
                preferred: None,
            }])
            .unwrap();
        app.set_profile_library(library.clone());
        assert_eq!(app.active_profile_name(), "Custom");
        app.handle_key_event(key(KeyCode::Up));
        library.bind_success(&bindings).unwrap();
        app.set_profile_library(library);
        app.profile_bindings_applied(&bindings);
        assert_eq!(app.active_profile_name(), "Custom");
        assert!(app.is_active_curve_dirty());
        // A successful acknowledgement of the accepted curve changes only that identity.
        assert!(
            app.profile_library
                .bindings
                .iter()
                .any(|b| b.profile == bindings[0].profile)
        );
    }

    #[test]
    fn rename_highlighted_custom_without_apply_and_builtin_is_immutable() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let custom =
            CurveProfile::custom("Night mode", built_in_profiles()[0].points.clone()).unwrap();
        let mut app = App::with_custom_profiles(snapshot, backend.name(), vec![custom]);
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::Home));
        app.handle_key_event(key(KeyCode::Char('r')));
        assert_eq!(app.modal, Some(Modal::Profiles));
        assert!(app.status.text.contains("cannot be renamed"));
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Char('r')));
        assert_eq!(app.modal, Some(Modal::RenameProfile));
        assert_eq!(app.profile_name_input, "Night mode");
        app.handle_key_event(key(KeyCode::Esc));
        assert_eq!(app.modal, None);
        assert_eq!(app.profiles.last().unwrap().name, "Night mode");
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Char('r')));
        for _ in 0.."Night mode".chars().count() {
            app.handle_key_event(key(KeyCode::Backspace));
        }
        for c in "Desk Work".chars() {
            app.handle_key_event(key(KeyCode::Char(c)));
        }
        let Some(AppCommand::RenameProfile { id, name }) =
            app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("rename command expected")
        };
        assert_eq!(name, "Desk Work");
        app.apply_failed("simulated write failure");
        assert_eq!(app.modal, Some(Modal::RenameProfile));
        assert_eq!(app.profile_name_input, "Desk Work");
        let mut library = app.profile_library.clone();
        library.rename(id, &name).unwrap();
        app.set_profile_library(library);
        app.rename_succeeded(id);
        assert_eq!(app.modal, Some(Modal::Profiles));
        assert_eq!(app.selected_profile().unwrap().name, "Desk Work");
    }

    #[test]
    fn h_keys_do_not_navigate_or_command_but_tab_reaches_disabled_host() {
        let mut app = app();
        let firmware = app.active_curve_key();
        let points = app.active_channel().unwrap().points.clone();
        for code in [KeyCode::Char('h'), KeyCode::Char('H')] {
            assert!(app.handle_key_event(key(code)).is_none());
            assert_eq!(app.editor_mode, EditorMode::Firmware);
        }
        assert_eq!(app.active_curve_key(), firmware);
        assert_eq!(app.active_channel().unwrap().points, points);

        app.host_editor.channels.clear();
        app.host_control_state = HostControlState::Disabled;
        assert!(app.handle_key_event(key(KeyCode::BackTab)).is_none());
        assert_eq!(app.editor_mode, EditorMode::Host);
        assert!(app.status.text.contains("disabled"));
        for code in [KeyCode::Char('h'), KeyCode::Char('H')] {
            assert!(app.handle_key_event(key(code)).is_none());
            assert_eq!(app.editor_mode, EditorMode::Host);
        }
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        app.handle_key_event(key(KeyCode::Tab));
        assert_eq!(app.editor_mode, EditorMode::Firmware);
    }

    #[test]
    fn h_is_not_unknown_state_navigation_but_is_a_rename_character_when_allowed() {
        let mut app = app();
        app.mark_host_status_unknown();
        assert!(app.handle_key_event(key(KeyCode::Char('h'))).is_none());
        assert!(app.status.text.contains("STATUS UNKNOWN"));
        app.update_telemetry(app.snapshot.clone());
        let custom =
            CurveProfile::custom("Night mode", built_in_profiles()[0].points.clone()).unwrap();
        let snapshot = app.snapshot.clone();
        let mut app = App::with_custom_profiles(snapshot, "DEMO", vec![custom]);
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Char('r')));
        app.handle_key_event(key(KeyCode::Char('h')));
        app.handle_key_event(key(KeyCode::Char('H')));
        assert_eq!(app.profile_name_input, "Night modehH");
    }

    #[test]
    fn host_defaults_have_exact_domain_and_safe_shape_for_every_capability() {
        let app = app();
        assert_eq!(
            app.host_editor.channels.len(),
            app.snapshot.host_control.channels.len()
        );
        for channel in &app.host_editor.channels {
            assert_eq!(channel.curve.source, HostTemperatureSource::Cpu);
            assert_eq!(channel.curve.points.len(), 40);
            assert_eq!(
                channel
                    .curve
                    .points
                    .first()
                    .unwrap()
                    .temperature_millidegrees,
                22_000
            );
            assert_eq!(
                channel
                    .curve
                    .points
                    .last()
                    .unwrap()
                    .temperature_millidegrees,
                100_000
            );
            assert!(
                channel
                    .curve
                    .points
                    .iter()
                    .enumerate()
                    .all(|(index, point)| {
                        point.temperature_millidegrees == 22_000 + index as i32 * 2_000
                            && point.duty_percent >= channel.capability.minimum_duty_percent
                    })
            );
            assert!(
                channel
                    .curve
                    .points
                    .windows(2)
                    .all(|points| points[0].duty_percent <= points[1].duty_percent)
            );
            assert_eq!(channel.curve.points.last().unwrap().duty_percent, 100);
        }
        let policy = app.host_editor.policy();
        assert_eq!(policy.channels.len(), app.host_editor.channels.len());
    }

    #[test]
    fn host_edits_stay_monotonic_and_final_point_is_locked() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        app.host_editor.selected_point = 20;
        let original = app.selected_host_point().unwrap().duty_percent;
        for _ in 0..100 {
            app.handle_key_event(key(KeyCode::Down));
        }
        let channel = app.selected_host_channel().unwrap();
        assert_eq!(channel.curve.points[20].duty_percent, 30);
        assert!(channel.curve.points[19].duty_percent <= 30);
        assert_ne!(channel.curve.points[20].duty_percent, original);
        for _ in 0..100 {
            app.handle_key_event(key(KeyCode::Up));
        }
        let channel = app.selected_host_channel().unwrap();
        assert_eq!(channel.curve.points[20].duty_percent, 100);
        assert!(
            channel.curve.points[20..]
                .iter()
                .all(|point| point.duty_percent == 100)
        );

        let before = channel.curve.clone();
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Down));
        assert_eq!(app.selected_host_point().unwrap().duty_percent, 100);
        assert_eq!(app.selected_host_channel().unwrap().curve, before);
        assert!(app.status.text.contains("locked"));
    }

    #[test]
    fn host_raise_follows_only_later_points_below_new_duty() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        let channel = &mut app.host_editor.channels[0];
        for (index, point) in channel.curve.points.iter_mut().enumerate() {
            point.duty_percent = match index {
                0..=9 => 40,
                10..=12 => 50,
                13..=38 => 60,
                _ => 100,
            };
        }
        let original = channel.curve.clone();
        app.host_editor.selected_point = 10;
        assert!(
            app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::SHIFT))
                .is_none()
        );
        let curve = &app.selected_host_channel().unwrap().curve;
        assert_eq!(&curve.points[..10], &original.points[..10]);
        assert_eq!(
            curve.points[10..13]
                .iter()
                .map(|point| point.duty_percent)
                .collect::<Vec<_>>(),
            vec![55; 3]
        );
        assert_eq!(&curve.points[13..], &original.points[13..]);
        assert_eq!(curve.source, original.source);
        assert!(app.status.text.contains("2 hotter points raised"));
    }

    #[test]
    fn host_lower_follows_only_earlier_points_above_new_duty() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        let channel = &mut app.host_editor.channels[0];
        for (index, point) in channel.curve.points.iter_mut().enumerate() {
            point.duty_percent = match index {
                0..=9 => 40,
                10 => 50,
                11..=13 => 60,
                14..=38 => 80,
                _ => 100,
            };
        }
        let original = channel.curve.clone();
        app.host_editor.selected_point = 13;
        assert!(
            app.handle_key_event(KeyEvent::new(KeyCode::Down, KeyModifiers::SHIFT))
                .is_none()
        );
        let curve = &app.selected_host_channel().unwrap().curve;
        assert_eq!(&curve.points[..11], &original.points[..11]);
        assert_eq!(
            curve.points[11..14]
                .iter()
                .map(|point| point.duty_percent)
                .collect::<Vec<_>>(),
            vec![55; 3]
        );
        assert_eq!(&curve.points[14..], &original.points[14..]);
        assert_eq!(curve.source, original.source);
        assert!(app.status.text.contains("2 cooler points lowered"));
    }

    #[test]
    fn host_edits_clamp_to_capability_and_100_without_touching_other_groups_or_firmware() {
        let mut app = app();
        let firmware = app.snapshot.devices.clone();
        app.handle_key_event(key(KeyCode::BackTab));
        let other = app.host_editor.channels[1].clone();
        app.host_editor.channels[0].capability.minimum_duty_percent = 88;
        let channel = &mut app.host_editor.channels[0];
        channel.curve.source = HostTemperatureSource::Gpu;
        for point in &mut channel.curve.points[..39] {
            point.duty_percent = 88;
        }
        let temperatures = channel
            .curve
            .points
            .iter()
            .map(|point| point.temperature_millidegrees)
            .collect::<Vec<_>>();
        app.host_editor.selected_point = 20;
        for _ in 0..30 {
            assert!(
                app.handle_key_event(KeyEvent::new(KeyCode::Down, KeyModifiers::SHIFT))
                    .is_none()
            );
        }
        assert_eq!(app.selected_host_point().unwrap().duty_percent, 88);
        for _ in 0..30 {
            assert!(
                app.handle_key_event(KeyEvent::new(KeyCode::Up, KeyModifiers::SHIFT))
                    .is_none()
            );
        }
        let curve = &app.selected_host_channel().unwrap().curve;
        assert!(
            curve.points[20..]
                .iter()
                .all(|point| point.duty_percent == 100)
        );
        assert!(
            curve.points[..20]
                .iter()
                .all(|point| point.duty_percent == 88)
        );
        assert_eq!(
            curve
                .points
                .iter()
                .map(|point| point.temperature_millidegrees)
                .collect::<Vec<_>>(),
            temperatures
        );
        assert_eq!(curve.source, HostTemperatureSource::Gpu);
        assert_eq!(app.host_editor.selected_point, 20);
        assert_eq!(app.host_editor.channels[1], other);
        assert_eq!(app.snapshot.devices, firmware);
        let before = app.host_editor.policy();
        app.handle_key_event(key(KeyCode::Up)); // capped: no change to any point
        assert_eq!(app.host_editor.policy(), before);
    }

    #[test]
    fn host_running_variable_length_drafts_survive_refresh_and_apply_only_on_enter() {
        for count in [2_usize, 3, 64] {
            let mut backend = DemoBackend::new();
            let mut snapshot = backend.refresh().unwrap();
            snapshot.monitoring.opted_in = true;
            let mut app = App::new(snapshot.clone(), "LIVE");
            let mut running = app.host_editor.policy();
            for channel in &mut running.channels {
                channel.curve.points = (0..count)
                    .map(|index| HostCurvePoint {
                        temperature_millidegrees: 22_000 + index as i32 * 1_000,
                        duty_percent: if index + 1 == count { 100 } else { 50 },
                    })
                    .collect();
            }
            snapshot.host_control.state = HostControlState::Running;
            snapshot.host_control.active_policy = Some(running.clone());
            app.update_telemetry(snapshot.clone());
            app.handle_key_event(key(KeyCode::BackTab));
            let other = app.host_editor.channels[1].clone();
            app.host_editor.selected_point = 0;
            let delta = KeyEvent::new(KeyCode::Up, KeyModifiers::SHIFT);
            for _ in 0..11 {
                assert!(app.handle_key_event(delta).is_none());
            }
            let draft = app.selected_host_channel().unwrap().curve.clone();
            assert_eq!(draft.points[0].duty_percent, 100);
            assert!(draft.points.iter().all(|point| point.duty_percent == 100));
            assert!(app.host_editor.selected_is_dirty());
            assert_eq!(
                app.snapshot.host_control.active_policy,
                Some(running.clone())
            );
            app.update_telemetry(snapshot);
            assert_eq!(app.selected_host_channel().unwrap().curve, draft);
            assert_eq!(app.host_editor.channels[1], other);
            assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
            let Some(AppCommand::UpdateHostControl { channel_policies }) =
                app.handle_key_event(key(KeyCode::Enter))
            else {
                panic!("expected only selected draft to update");
            };
            assert_eq!(channel_policies.len(), 1);
            assert_eq!(channel_policies[0].curve, draft);
            assert_eq!(
                channel_policies[0].channel_id,
                running.channels[0].channel_id
            );
            app.host_update_succeeded(channel_policies);
            assert!(!app.host_editor.selected_is_dirty());
            assert_eq!(app.host_editor.channels[1], other);
            assert_eq!(app.host_editor.channels[0].curve.points.len(), count);
            assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
            assert_eq!(app.modal, Some(Modal::HostApplyScope));
        }
    }

    fn running_scope_app() -> App {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        app.host_start_succeeded(app.host_editor.policy());
        app.handle_key_event(key(KeyCode::BackTab));
        app
    }

    #[test]
    fn host_scope_resets_to_current_and_cancel_preserves_every_draft() {
        let mut app = running_scope_app();
        app.host_editor.channels[1].curve.source = HostTemperatureSource::Gpu;
        let drafts = app.host_editor.policy();
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(!app.host_apply_scope.as_ref().unwrap().all_groups);
        app.handle_key_event(key(KeyCode::End));
        assert!(app.host_apply_scope.as_ref().unwrap().all_groups);
        assert_eq!(app.handle_key_event(key(KeyCode::Esc)), None);
        assert_eq!(app.host_editor.policy(), drafts);
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(!app.host_apply_scope.as_ref().unwrap().all_groups);
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::RememberHostProfiles { .. })
        )); // explicit clean apply remembers
        assert_eq!(app.host_editor.policy(), drafts);
        assert_eq!(app.modal, None);
    }

    #[test]
    fn host_scope_all_clones_clean_selected_source_curve_and_minimum_not_other_drafts() {
        let mut app = running_scope_app();
        let selected = app.host_editor.channels[0].curve.clone();
        app.host_editor.channels[1].capability.minimum_duty_percent = 65;
        app.host_editor.channels[1].curve.source = HostTemperatureSource::Gpu;
        app.host_editor.channels[1].curve.points[0].duty_percent = 90;
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        app.handle_key_event(key(KeyCode::Down));
        let Some(AppCommand::UpdateHostControl { channel_policies }) =
            app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("expected atomic batch")
        };
        assert_eq!(channel_policies.len(), 2);
        assert_eq!(channel_policies[0].curve, selected);
        assert_eq!(channel_policies[1].curve.source, selected.source);
        assert_eq!(
            channel_policies[1].curve.points.len(),
            selected.points.len()
        );
        for (copied, point) in channel_policies[1]
            .curve
            .points
            .iter()
            .zip(&selected.points)
        {
            assert_eq!(
                copied.temperature_millidegrees,
                point.temperature_millidegrees
            );
            assert_eq!(copied.duty_percent, point.duty_percent.max(65));
        }
        assert_eq!(
            channel_policies[1]
                .curve
                .points
                .last()
                .unwrap()
                .duty_percent,
            100
        );
        assert_eq!(app.host_editor.channels[1].curve, channel_policies[1].curve);
        assert_eq!(
            app.snapshot
                .host_control
                .active_policy
                .as_ref()
                .unwrap()
                .channels[1]
                .curve
                .source,
            HostTemperatureSource::Cpu
        );
        // A newer working draft is not overwritten by the acknowledgement or snapshot.
        app.host_editor.channels[1].curve.source = HostTemperatureSource::CpuGpuMax;
        app.host_update_succeeded(channel_policies.clone());
        assert_eq!(app.status.text, "2 fan groups updated");
        assert_eq!(
            app.snapshot
                .host_control
                .active_policy
                .as_ref()
                .unwrap()
                .channels,
            channel_policies
        );
        assert_eq!(
            app.host_editor.channels[1].curve.source,
            HostTemperatureSource::CpuGpuMax
        );
        let fresh = app.snapshot.clone();
        app.update_telemetry(fresh);
        assert_eq!(
            app.host_editor.channels[1].curve.source,
            HostTemperatureSource::CpuGpuMax
        );
        assert!(app.host_policy_is_dirty());
    }

    #[test]
    fn host_scope_only_updates_selected_even_with_other_dirty_and_allows_clean_all() {
        let mut app = running_scope_app();
        app.host_editor.channels[1].curve.source = HostTemperatureSource::Gpu;
        let other = app.host_editor.channels[1].clone();
        app.handle_key_event(key(KeyCode::Char('s')));
        app.handle_key_event(key(KeyCode::Enter));
        let Some(AppCommand::UpdateHostControl { channel_policies }) =
            app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("selected apply")
        };
        assert_eq!(channel_policies.len(), 1);
        assert_eq!(app.host_editor.channels[1], other);
        app.host_update_succeeded(channel_policies);
        assert_eq!(app.status.text, "Fan group updated");
        app.handle_key_event(key(KeyCode::Enter)); // selected clean
        app.handle_key_event(key(KeyCode::End));
        assert!(matches!(app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::UpdateHostControl { channel_policies }) if channel_policies.len() == 2));
    }

    #[test]
    fn host_scope_copy_bounds_an_out_of_range_advertised_minimum() {
        let mut app = running_scope_app();
        app.host_editor.channels[1].capability.minimum_duty_percent = u8::MAX;
        app.handle_key_event(key(KeyCode::Enter));
        app.handle_key_event(key(KeyCode::End));
        let Some(AppCommand::UpdateHostControl { channel_policies }) =
            app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("expected a bounded batch proposal")
        };
        assert!(
            channel_policies[1]
                .curve
                .points
                .iter()
                .all(|point| point.duty_percent == 100)
        );
    }

    #[test]
    fn host_scope_rejects_stale_selection_curve_capabilities_targets_and_status() {
        for scenario in 0..6 {
            let mut app = running_scope_app();
            app.handle_key_event(key(KeyCode::Enter));
            app.handle_key_event(key(KeyCode::End));
            let before = app.host_editor.policy();
            match scenario {
                0 => app.host_editor.selected_channel = 1,
                1 => app.host_editor.channels[0].curve.source = HostTemperatureSource::Gpu,
                2 => app.host_editor.channels[1].capability.minimum_duty_percent = 50,
                3 => {
                    app.host_editor.channels.pop();
                }
                4 => app.mark_host_status_unknown(),
                _ => {
                    app.host_control_state = HostControlState::RestoreRequired;
                }
            }
            assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
            assert_eq!(app.modal, None);
            if scenario == 0 || scenario >= 4 {
                assert_eq!(app.host_editor.policy(), before);
            }
            assert_ne!(app.status.kind, StatusKind::Success);
        }
    }

    #[test]
    fn host_scope_copies_gpu_and_max_sources_with_variable_domain() {
        for source in [HostTemperatureSource::Gpu, HostTemperatureSource::CpuGpuMax] {
            let mut app = running_scope_app();
            let selected = &mut app.host_editor.channels[0].curve;
            selected.source = source;
            selected.points = vec![
                HostCurvePoint {
                    temperature_millidegrees: 15_000,
                    duty_percent: 35,
                },
                HostCurvePoint {
                    temperature_millidegrees: 66_000,
                    duty_percent: 80,
                },
                HostCurvePoint {
                    temperature_millidegrees: 100_000,
                    duty_percent: 100,
                },
            ];
            let selected = selected.clone();
            app.host_editor.channels[1].capability.minimum_duty_percent = 50;
            app.handle_key_event(key(KeyCode::Enter));
            app.handle_key_event(key(KeyCode::End));
            let Some(AppCommand::UpdateHostControl { channel_policies }) =
                app.handle_key_event(key(KeyCode::Enter))
            else {
                panic!("expected batch")
            };
            assert_eq!(channel_policies.len(), 2);
            assert_eq!(channel_policies[0].curve, selected);
            assert_eq!(channel_policies[1].curve.source, source);
            assert_eq!(
                channel_policies[1]
                    .curve
                    .points
                    .iter()
                    .map(|p| p.temperature_millidegrees)
                    .collect::<Vec<_>>(),
                vec![15_000, 66_000, 100_000]
            );
            assert_eq!(
                channel_policies[1]
                    .curve
                    .points
                    .iter()
                    .map(|p| p.duty_percent)
                    .collect::<Vec<_>>(),
                vec![50, 80, 100]
            );
        }
    }

    #[test]
    fn host_scope_rejects_refresh_policy_drift_and_pending_operation() {
        for busy in [false, true] {
            let mut app = running_scope_app();
            app.handle_key_event(key(KeyCode::Enter));
            app.handle_key_event(key(KeyCode::End));
            let drafts = app.host_editor.policy();
            if busy {
                app.host_operation_busy = true;
            } else {
                let mut fresh = app.snapshot.clone();
                fresh.host_control.active_policy.as_mut().unwrap().channels[1]
                    .curve
                    .source = HostTemperatureSource::Gpu;
                app.update_telemetry(fresh);
            }
            assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
            assert_eq!(app.modal, None);
            assert_eq!(app.host_editor.policy().channels[0], drafts.channels[0]);
            assert_eq!(app.status.kind, StatusKind::Error);
        }
    }

    #[test]
    fn host_scope_settings_stop_and_q_quit_without_applying() {
        let mut app = running_scope_app();
        app.handle_key_event(key(KeyCode::Enter));
        app.mark_host_status_unknown();
        assert_eq!(settings_stop(&mut app), Some(AppCommand::StopHostControl));
        assert_eq!(app.modal, None);
        app.host_status_trusted = true;
        app.handle_key_event(key(KeyCode::Enter));
        assert_eq!(app.handle_key_event(key(KeyCode::Char('q'))), None);
        assert!(app.exit);
        assert_eq!(app.modal, None);
    }

    #[test]
    fn host_target_is_curve_duty_not_applied_pwm_and_keeps_full_endpoint() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        let channel = app.selected_host_channel().unwrap();
        assert_eq!(channel.curve.points.last().unwrap().duty_percent, 100);
        let temperature = app.host_source_temperature_celsius().unwrap();
        assert_eq!(
            app.host_target_duty(),
            Some(evaluate_host_curve(
                &channel.curve,
                (temperature * 1_000.0).round() as i32
            ))
        );
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Down));
        assert_eq!(app.selected_host_point().unwrap().duty_percent, 100);
    }

    #[test]
    fn host_source_cycles_and_only_builtin_host_picker_opens() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        for expected in [
            HostTemperatureSource::Gpu,
            HostTemperatureSource::CpuGpuMax,
            HostTemperatureSource::Cpu,
        ] {
            app.handle_key_event(key(KeyCode::Char('s')));
            assert_eq!(app.selected_host_channel().unwrap().curve.source, expected);
        }
        app.handle_key_event(key(KeyCode::Char('p')));
        assert_eq!(app.modal, Some(Modal::HostProfiles));
        assert_eq!(app.handle_key_event(key(KeyCode::Char('s'))), None);
        assert_eq!(app.handle_key_event(key(KeyCode::Char('x'))), None);
        assert_eq!(app.modal, Some(Modal::HostProfiles));
        app.handle_key_event(key(KeyCode::Esc));
        app.handle_key_event(key(KeyCode::Char('x')));
        assert_eq!(app.modal, None);
    }

    #[test]
    fn host_presets_only_load_selected_drafts_and_keep_source_minimum_and_firmware() {
        for profile_index in 0..3 {
            let mut app = app();
            let firmware = app.snapshot.devices.clone();
            app.handle_key_event(key(KeyCode::BackTab));
            app.handle_key_event(key(KeyCode::Char('s')));
            app.host_editor.channels[0].capability.minimum_duty_percent = 55;
            let other = app.host_editor.channels[1].clone();
            let expected =
                built_in_host_profiles(HostTemperatureSource::Gpu, 55)[profile_index].clone();
            assert!(app.handle_key_event(key(KeyCode::Char('p'))).is_none());
            app.profile_cursor = profile_index;
            assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
            assert_eq!(app.modal, None);
            assert_eq!(app.host_control_state, HostControlState::Available);
            assert_eq!(app.selected_host_channel().unwrap().curve, expected.curve);
            assert_eq!(app.host_editor.channels[1], other);
            assert_eq!(app.snapshot.devices, firmware);
            assert_eq!(app.active_host_profile_name(), Some(expected.name));
            assert!(app.host_policy_is_dirty());
            // Only a separate Enter after the picker closes can issue Start.
            let Some(AppCommand::StartHostControl { policy }) =
                app.handle_key_event(key(KeyCode::Enter))
            else {
                panic!("expected separate explicit Start");
            };
            assert_eq!(policy.channels[0].curve, expected.curve);
            assert_eq!(policy.channels[1].curve, other.curve);
        }
    }

    #[test]
    fn host_preset_picker_cancel_and_unsafe_states_never_modify_or_start() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        let original = app.host_editor.policy();
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        assert!(app.handle_key_event(key(KeyCode::Esc)).is_none());
        assert_eq!(app.host_editor.policy(), original);
        for state in [
            HostControlState::Disabled,
            HostControlState::Restoring,
            HostControlState::RestoreRequired,
        ] {
            app.host_control_state = state;
            assert!(app.handle_key_event(key(KeyCode::Char('p'))).is_none());
            assert_eq!(app.modal, Some(Modal::HostProfiles));
            assert_eq!(app.host_editor.policy(), original);
            app.handle_key_event(key(KeyCode::Esc));
        }
        app.host_control_state = HostControlState::Available;
        app.handle_key_event(key(KeyCode::Char('p')));
        // Recheck edit permission if service state changes while the picker is open.
        app.host_control_state = HostControlState::Restoring;
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.host_editor.policy(), original);
        app.host_control_state = HostControlState::Available;
        app.mark_host_status_unknown();
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.host_editor.policy(), original);
        app.handle_key_event(key(KeyCode::Esc));
        assert_eq!(app.modal, None);
    }

    #[test]
    fn host_preset_picker_rejects_retargeting_and_rechecks_changed_minimum() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.host_editor.channels[0].capability.channel_id = ChannelId::new("replacement");
        let before = app.host_editor.policy();
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.host_editor.policy(), before);
        assert_eq!(app.modal, None);
        assert!(app.status.text.contains("selection changed"));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.host_editor.channels[0].capability.minimum_duty_percent = 90;
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(
            app.selected_host_channel()
                .unwrap()
                .curve
                .points
                .iter()
                .all(|point| point.duty_percent >= 90)
        );
    }

    #[test]
    fn loading_host_preset_replaces_a_larger_domain_and_clamps_selection() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        app.host_editor.channels[0].curve.points = (0..64)
            .map(|index| HostCurvePoint {
                temperature_millidegrees: index * 1_000,
                duty_percent: if index == 63 { 100 } else { 50 },
            })
            .collect();
        app.host_editor.selected_point = 63;
        app.handle_key_event(key(KeyCode::Char('p')));
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.selected_host_channel().unwrap().curve.points.len(), 40);
        assert_eq!(app.host_editor.selected_point, 39);
        assert_eq!(
            app.selected_host_point().unwrap().temperature_millidegrees,
            100_000
        );
    }

    #[test]
    fn host_preset_name_tracks_actual_curve_after_edits_and_reconnect() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        let curve = built_in_host_profiles(HostTemperatureSource::Cpu, 30)[2]
            .curve
            .clone();
        app.host_editor.channels[0].curve = curve.clone();
        assert_eq!(
            app.active_host_profile_name().as_deref(),
            Some("Performance")
        );
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Down));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
        let mut running = backend.refresh().unwrap();
        let mut actual = app.host_editor.policy();
        actual.channels[0].curve = curve;
        running.host_control.state = HostControlState::Running;
        running.host_control.active_policy = Some(actual);
        app.update_telemetry(running);
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
        assert!(app.host_editor.selected_is_dirty());
    }

    #[test]
    fn host_profile_labels_do_not_guess_when_floor_merges_presets() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.host_control.channels[0].minimum_duty_percent = 100;
        let mut app = App::new(snapshot.clone(), "DEMO");
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.profile_cursor = 0;
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.active_host_profile_name(), None);
        snapshot.host_control.state = HostControlState::Running;
        snapshot.host_control.active_policy = Some(app.host_editor.policy());
        app.update_telemetry(snapshot);
        assert_eq!(app.active_host_profile_name(), None);
    }

    #[test]
    fn reordered_snapshot_keeps_selected_id_and_applies_that_groups_draft() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot.clone(), "DEMO");
        let active = app.host_editor.policy();
        app.host_start_succeeded(active.clone());
        app.handle_key_event(key(KeyCode::BackTab));
        let selected = app
            .selected_host_channel()
            .unwrap()
            .capability
            .channel_id
            .clone();
        app.host_editor.channels[0].curve.points[0].duty_percent = 39;
        app.host_editor.channels[1].curve.points[0].duty_percent = 38;
        snapshot.host_control.state = HostControlState::Running;
        snapshot.host_control.active_policy = Some(active);
        snapshot.host_control.channels.reverse();
        app.update_telemetry(snapshot);
        assert_eq!(app.host_editor.selected_channel, 1);
        assert_eq!(
            app.selected_host_channel().unwrap().capability.channel_id,
            selected
        );
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        let Some(AppCommand::UpdateHostControl { channel_policies }) =
            app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("expected selected-group update");
        };
        assert_eq!(channel_policies.len(), 1);
        assert_eq!(channel_policies[0].channel_id, selected);
        assert_eq!(channel_policies[0].curve.points[0].duty_percent, 39);
        assert_eq!(app.host_editor.channels[0].curve.points[0].duty_percent, 38);
    }

    #[test]
    fn host_default_label_includes_the_default_temperature_source() {
        let mut app = app();
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Default"));
        app.host_editor.channels[0].curve.source = HostTemperatureSource::Gpu;
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
    }

    #[test]
    fn running_presets_apply_only_selected_group_and_preserve_other_drafts() {
        let mut backend = DemoBackend::new();
        let initial = backend.refresh().unwrap();
        let mut app = App::new(initial, "DEMO");
        let original = app.host_editor.policy();
        backend.start_host_control(&original).unwrap();
        app.update_telemetry(backend.refresh().unwrap());
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('c')));
        app.handle_key_event(key(KeyCode::Char('s')));
        let other_draft = app.selected_host_channel().unwrap().curve.clone();
        app.handle_key_event(key(KeyCode::Char('[')));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.profile_cursor = 2;
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.modal, Some(Modal::HostApplyScope));
        let Some(AppCommand::UpdateHostControl { channel_policies }) =
            app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("expected confirmed update")
        };
        assert_eq!(channel_policies.len(), 1);
        let channel_policy = &channel_policies[0];
        assert_eq!(channel_policy.channel_id, original.channels[0].channel_id);
        assert_eq!(channel_policy.curve.points.len(), 40);
        assert_eq!(
            channel_policy.curve.source,
            original.channels[0].curve.source
        );
        assert!(
            channel_policy
                .curve
                .points
                .iter()
                .all(|point| point.duty_percent >= 30)
        );
        assert_eq!(
            app.snapshot.host_control.active_policy,
            Some(original.clone())
        );
        backend.update_host_control(&channel_policies).unwrap();
        app.host_update_succeeded(channel_policies.clone());
        assert_eq!(app.host_editor.channels[1].curve, other_draft);
        assert_eq!(
            app.snapshot
                .host_control
                .active_policy
                .as_ref()
                .unwrap()
                .channels[1],
            original.channels[1]
        );
        app.update_telemetry(backend.refresh().unwrap());
        assert_eq!(app.host_editor.channels[1].curve, other_draft);
        assert!(app.host_policy_is_dirty());
        assert_eq!(app.handle_key_event(key(KeyCode::Enter)), None);
        assert_eq!(app.modal, Some(Modal::HostApplyScope));
        assert_eq!(settings_stop(&mut app), Some(AppCommand::StopHostControl));
    }

    #[test]
    fn running_draft_survives_snapshots_and_lost_ack_reconciles_only_accepted_channel() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), "DEMO");
        let original = app.host_editor.policy();
        backend.start_host_control(&original).unwrap();
        app.update_telemetry(backend.refresh().unwrap());
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('s')));
        let accepted = HostChannelPolicy {
            channel_id: original.channels[0].channel_id.clone(),
            curve: app.selected_host_channel().unwrap().curve.clone(),
        };
        app.handle_key_event(key(KeyCode::Char('c')));
        app.handle_key_event(key(KeyCode::Char('s')));
        let other = app.selected_host_channel().unwrap().curve.clone();
        for _ in 0..3 {
            app.update_telemetry(backend.refresh().unwrap());
        }
        assert_eq!(app.host_editor.channels[0].curve, accepted.curve);
        assert_eq!(app.host_editor.channels[1].curve, other);
        backend
            .update_host_control(std::slice::from_ref(&accepted))
            .unwrap();
        app.mark_host_status_unknown(); // reply lost; no retry or new Start
        app.update_telemetry(backend.refresh().unwrap());
        assert!(app.host_status_trusted);
        assert!(app.host_editor.selected_is_dirty());
        assert_eq!(app.host_editor.channels[0].curve, accepted.curve);
        assert_eq!(app.host_editor.channels[1].curve, other);
        assert_eq!(
            app.snapshot
                .host_control
                .active_policy
                .as_ref()
                .unwrap()
                .channels[1],
            original.channels[1]
        );
    }

    #[test]
    fn snapshot_diagnostic_displays_only_on_change_without_claiming_bios_restore() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.host_control.state = HostControlState::RestoreRequired;
        snapshot.host_control.last_error = Some("BIOS mode 2 did not verify".into());
        let mut app = App::new(opted_in(snapshot.clone()), "LIVE");
        assert!(app.status.text.contains("BIOS mode 2 did not verify"));
        assert_eq!(app.host_control_state, HostControlState::RestoreRequired);
        app.status = StatusMessage::info("unrelated");
        app.update_telemetry(snapshot.clone());
        assert_eq!(app.status.text, "unrelated");
        snapshot.host_control.last_error = Some("new actual failure".into());
        app.update_telemetry(snapshot);
        assert!(app.status.text.contains("new actual failure"));
        assert_eq!(app.host_control_state, HostControlState::RestoreRequired);
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(settings_stop(&mut app), Some(AppCommand::StopHostControl));
    }

    #[test]
    fn host_enter_starts_and_running_enter_updates_only_dirty_draft() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app =
            App::with_runtime_config(opted_in(snapshot), "LIVE", Vec::new(), AppConfig::default());
        app.handle_key_event(key(KeyCode::BackTab));

        let command = app.handle_key_event(key(KeyCode::Enter));
        assert!(matches!(command, Some(AppCommand::StartHostControl { .. })));
        assert_eq!(app.modal, None);
        let Some(AppCommand::StartHostControl { policy }) = command else {
            unreachable!()
        };
        app.host_start_succeeded(policy);
        assert_eq!(app.host_control_state, HostControlState::Running);
        assert!(!app.host_policy_is_dirty());
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.modal, Some(Modal::HostApplyScope));
        assert!(app.handle_key_event(key(KeyCode::Esc)).is_none());
        app.handle_key_event(key(KeyCode::Char('s')));
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::UpdateHostControl { .. })
        ));
        assert_eq!(settings_stop(&mut app), Some(AppCommand::StopHostControl));
        app.host_stop_succeeded();
        assert_eq!(app.host_control_state, HostControlState::Available);
        assert_eq!(app.status.kind, StatusKind::Success);
        assert_eq!(app.status.text, "Fan control stopped");
        assert_eq!(app.host_editor.channels.len(), 2);
    }

    #[test]
    fn host_stop_is_explicit_even_when_status_unknown_or_recovery_required() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(opted_in(backend.refresh().unwrap()), "LIVE");
        app.handle_key_event(key(KeyCode::BackTab));
        assert!(settings_stop(&mut app).is_none());
        app.host_start_succeeded(app.host_editor.policy());
        app.mark_host_status_unknown();
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(settings_stop(&mut app), Some(AppCommand::StopHostControl));
        app.mark_host_restore_required();
        assert_eq!(settings_stop(&mut app), Some(AppCommand::StopHostControl));
        app.host_editor.channels.clear();
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(settings_stop(&mut app), Some(AppCommand::StopHostControl));
    }

    #[test]
    fn host_enter_is_blocked_in_unsafe_states() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        for state in [
            HostControlState::Disabled,
            HostControlState::Restoring,
            HostControlState::RestoreRequired,
        ] {
            app.host_control_state = state;
            assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
            assert_eq!(app.status.kind, StatusKind::Error);
        }
    }

    #[test]
    fn service_policy_keeps_its_domain_and_selection_through_refresh_and_stop() {
        for count in [3, 64] {
            let mut backend = DemoBackend::new();
            let mut fresh = backend.refresh().unwrap();
            let mut app = App::new(fresh.clone(), backend.name());
            let mut actual = app.host_editor.policy();
            for channel in &mut actual.channels {
                channel.curve.points = (0..count)
                    .map(|index| HostCurvePoint {
                        temperature_millidegrees: index * 1_000,
                        duty_percent: if index + 1 == count { 100 } else { 50 },
                    })
                    .collect();
            }
            fresh.host_control.state = HostControlState::Running;
            fresh.host_control.active_policy = Some(actual.clone());
            app.update_telemetry(fresh.clone());
            app.host_editor.selected_point = usize::try_from(count - 1).unwrap();
            app.update_telemetry(fresh.clone());
            assert_eq!(
                app.host_editor.selected_point,
                usize::try_from(count - 1).unwrap()
            );
            assert_eq!(app.host_editor.policy(), actual);
            app.host_stop_succeeded();
            fresh.host_control.state = HostControlState::Available;
            fresh.host_control.active_policy = None;
            app.update_telemetry(fresh);
            assert_eq!(app.host_editor.policy(), actual);
            assert!(!app.host_policy_is_dirty());
        }
    }

    #[test]
    fn host_reconciliation_preserves_matching_edits_and_adds_defaults() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), backend.name());
        app.handle_key_event(key(KeyCode::BackTab));
        app.host_editor.selected_point = 20;
        app.handle_key_event(key(KeyCode::Up));
        let retained_id = app.host_editor.channels[0].capability.channel_id.clone();
        let retained_curve = app.host_editor.channels[0].curve.clone();

        let mut fresh = backend.refresh().unwrap();
        fresh.host_control.channels.remove(1);
        fresh.host_control.channels.push(HostChannelCapability {
            channel_id: ChannelId::new("new-candidate"),
            name: "New candidate".into(),
            minimum_duty_percent: 55,
        });
        app.update_telemetry(fresh);

        let retained = app
            .host_editor
            .channels
            .iter()
            .find(|channel| channel.capability.channel_id == retained_id)
            .unwrap();
        assert_eq!(retained.curve, retained_curve);
        let added = app
            .host_editor
            .channels
            .iter()
            .find(|channel| channel.capability.channel_id.0 == "new-candidate")
            .unwrap();
        assert_eq!(added.curve.source, HostTemperatureSource::Cpu);
        assert!(
            added
                .curve
                .points
                .iter()
                .all(|point| point.duty_percent >= 55)
        );
    }

    #[test]
    fn host_dirty_tracks_applied_policy_and_uncertain_state_until_snapshot() {
        let mut backend = DemoBackend::new();
        let mut app = App::new(backend.refresh().unwrap(), backend.name());
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('s')));
        assert!(app.host_policy_is_dirty());
        let policy = app.host_editor.policy();
        app.handle_key_event(key(KeyCode::Char('s')));
        assert_eq!(
            app.selected_host_channel().unwrap().curve.source,
            HostTemperatureSource::CpuGpuMax
        );
        app.host_start_succeeded(policy);
        assert!(app.host_policy_is_dirty());
        assert_eq!(
            app.selected_host_channel().unwrap().curve.source,
            HostTemperatureSource::CpuGpuMax
        );

        let error = crate::backend::BackendError::with_kind(
            crate::backend::BackendErrorKind::UnknownOutcome,
            "uncertain",
        );
        app.host_operation_failed(&error);
        assert!(!app.host_status_trusted);
        assert_eq!(app.status.kind, StatusKind::Error);
        assert!(!app.status.text.contains("verified"));
        let failure = crate::backend::BackendError::with_kind(
            crate::backend::BackendErrorKind::RestoreRequired,
            "BIOS mode 2 did not verify",
        );
        app.host_operation_failed(&failure);
        assert_eq!(app.host_control_state, HostControlState::RestoreRequired);
        assert_eq!(app.status.kind, StatusKind::Error);
        assert!(!app.status.text.contains("Fan control stopped"));
        let mut fresh = backend.refresh().unwrap();
        fresh.host_control.state = HostControlState::Available;
        app.update_telemetry(fresh);
        assert_eq!(app.host_control_state, HostControlState::Available);
        assert_eq!(
            app.selected_host_channel().unwrap().curve.source,
            HostTemperatureSource::CpuGpuMax
        );
    }

    #[test]
    fn reconnect_adopts_exact_service_curve_and_blocks_actions_while_unknown() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(opted_in(snapshot), "LIVE");
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('s')));
        assert!(app.host_policy_is_dirty());
        app.mark_host_status_unknown();
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(app.status.text.contains("STATUS UNKNOWN"));
        let original = app.selected_host_point().unwrap();
        app.handle_key_event(key(KeyCode::Up));
        assert_eq!(app.selected_host_point().unwrap(), original);

        let mut actual = app.host_editor.policy();
        actual.channels[0].curve.source = HostTemperatureSource::CpuGpuMax;
        actual.channels[0].curve.points = vec![
            HostCurvePoint {
                temperature_millidegrees: 30_500,
                duty_percent: 35,
            },
            HostCurvePoint {
                temperature_millidegrees: 80_000,
                duty_percent: 100,
            },
        ];
        let mut fresh = opted_in(backend.refresh().unwrap());
        fresh.host_control.state = HostControlState::Running;
        fresh.host_control.active_policy = Some(actual.clone());
        app.update_telemetry(fresh.clone());
        assert!(app.host_status_trusted);
        assert_ne!(app.host_editor.policy(), actual);
        assert!(app.host_policy_is_dirty());
        assert_eq!(app.selected_host_channel().unwrap().curve.points.len(), 40);
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert!(matches!(
            app.handle_key_event(key(KeyCode::Enter)),
            Some(AppCommand::UpdateHostControl { .. })
        ));
        let newcomer = App::new(opted_in(fresh), "LIVE");
        assert_eq!(newcomer.host_editor.policy(), actual);
        assert!(!newcomer.host_policy_is_dirty());
    }

    #[test]
    fn lost_connection_prevents_confirming_a_pending_firmware_apply() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.devices[0].cooling_channels[0].curve_state = CurveState::Unverified;
        let mut app =
            App::with_runtime_config(opted_in(snapshot), "LIVE", vec![], AppConfig::default());
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.modal, Some(Modal::ConfirmApply));
        app.mark_host_status_unknown();
        assert!(app.handle_key_event(key(KeyCode::Char('y'))).is_none());
        assert!(app.status.text.contains("STATUS UNKNOWN"));
        assert!(app.handle_key_event(key(KeyCode::Char('n'))).is_none());
        assert!(app.pending_confirmation.is_none());
    }

    #[test]
    fn explicit_restore_required_blocks_start_without_claiming_unknown_state() {
        let mut app = app();
        app.handle_key_event(key(KeyCode::BackTab));
        app.mark_host_restore_required();
        assert!(app.host_status_trusted);
        assert!(app.handle_key_event(key(KeyCode::Enter)).is_none());
        assert_eq!(app.status.text, "Recovery required; O for Settings");
    }

    #[test]
    fn host_s_cycles_source_in_both_cases_without_stopping() {
        for character in ['s', 'S'] {
            let mut app = app();
            app.handle_key_event(key(KeyCode::BackTab));
            app.host_start_succeeded(app.host_editor.policy());
            let original_points = app.selected_host_channel().unwrap().curve.points.clone();
            for expected in [
                HostTemperatureSource::Gpu,
                HostTemperatureSource::CpuGpuMax,
                HostTemperatureSource::Cpu,
            ] {
                assert_eq!(app.handle_key_event(key(KeyCode::Char(character))), None);
                assert_eq!(app.selected_host_channel().unwrap().curve.source, expected);
                assert_eq!(
                    app.selected_host_channel().unwrap().curve.points,
                    original_points
                );
                assert_eq!(app.host_control_state, HostControlState::Running);
            }
        }
    }

    #[test]
    fn host_stop_is_in_settings_not_an_x_shortcut() {
        for (state, trusted, busy, should_stop) in [
            (HostControlState::Available, true, false, false),
            (HostControlState::Disabled, true, false, false),
            (HostControlState::Running, true, false, true),
            (HostControlState::Restoring, true, false, true),
            (HostControlState::RestoreRequired, true, false, true),
            (HostControlState::Running, false, false, true),
            (HostControlState::Running, true, true, true),
        ] {
            let mut app = app();
            app.handle_key_event(key(KeyCode::BackTab));
            app.host_control_state = state;
            app.host_status_trusted = trusted;
            app.host_operation_busy = busy;
            let before = app.host_editor.policy();
            assert_eq!(app.handle_key_event(key(KeyCode::Char('x'))), None);
            assert_eq!(app.handle_key_event(key(KeyCode::Char('X'))), None);
            assert_eq!(
                settings_stop(&mut app),
                should_stop.then_some(AppCommand::StopHostControl)
            );
            assert_eq!(app.host_editor.policy(), before);
            if busy {
                assert!(app.status.text.contains("queued"));
            }
        }
    }

    #[test]
    fn host_s_respects_pending_unknown_recovery_and_disabled_guards() {
        for character in ['s', 'S'] {
            for (state, trusted, busy) in [
                (HostControlState::Running, true, true),
                (HostControlState::Running, false, false),
                (HostControlState::Restoring, true, false),
                (HostControlState::RestoreRequired, true, false),
                (HostControlState::Disabled, true, false),
            ] {
                let mut app = app();
                app.handle_key_event(key(KeyCode::BackTab));
                app.host_control_state = state;
                app.host_status_trusted = trusted;
                app.host_operation_busy = busy;
                let before = app.host_editor.policy();
                assert_eq!(app.handle_key_event(key(KeyCode::Char(character))), None);
                assert_eq!(app.host_editor.policy(), before);
            }
        }
    }

    #[test]
    fn firmware_s_does_not_save_and_x_does_not_stop_host() {
        let mut app = app();
        app.host_start_succeeded(app.host_editor.policy());
        let before = app.active_channel().unwrap().clone();
        for character in ['s', 'S', 'x', 'X'] {
            assert_eq!(app.handle_key_event(key(KeyCode::Char(character))), None);
            assert_eq!(app.modal, None);
        }
        assert_eq!(app.active_channel().unwrap(), &before);
        assert_eq!(app.host_control_state, HostControlState::Running);
    }

    fn fixture_library(snapshot: &HardwareSnapshot) -> ProfileLibrary {
        let mut library = ProfileLibrary::default();
        let key = (
            &snapshot.devices[0].id,
            &snapshot.devices[0].cooling_channels[0].id,
        );
        let firmware = snapshot.devices[0].cooling_channels[0].points.clone();
        let host = HostCurve {
            source: HostTemperatureSource::Gpu,
            points: vec![
                HostCurvePoint {
                    temperature_millidegrees: 25_000,
                    duty_percent: 45,
                },
                HostCurvePoint {
                    temperature_millidegrees: 65_000,
                    duty_percent: 100,
                },
            ],
        };
        library.profiles = vec![
            crate::profile::SavedProfile {
                id: 1,
                name: "Quiet Night".into(),
                curve: ProfileCurve::Firmware(firmware.clone()),
            },
            crate::profile::SavedProfile {
                id: 2,
                name: "GPU Night".into(),
                curve: ProfileCurve::Host(host.clone()),
            },
        ];
        library.next_index = 3;
        library.bindings = vec![
            ProfileBinding {
                target: ProfileTarget::Firmware {
                    device_id: key.0.clone(),
                    channel_id: key.1.clone(),
                },
                profile: ProfileRef::Custom(1),
                curve: ProfileCurve::Firmware(firmware),
            },
            ProfileBinding {
                target: ProfileTarget::Host {
                    channel_id: snapshot.host_control.channels[0].channel_id.clone(),
                },
                profile: ProfileRef::Custom(2),
                curve: ProfileCurve::Host(host),
            },
        ];
        library
    }

    #[test]
    fn restart_empty_snapshot_then_available_restores_firmware_and_host_preview_without_start() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let library = fixture_library(&snapshot);
        let mut empty = snapshot.clone();
        empty.devices.clear();
        empty.host_control.channels.clear();
        empty.host_control.active_policy = None;
        let mut app =
            App::with_profile_library(opted_in(empty), "LIVE", library, AppConfig::default());
        assert!(app.host_editor.channels.is_empty());
        let mut fresh = snapshot.clone();
        fresh.devices[0].cooling_channels[0].curve_state = CurveState::Unverified;
        app.update_telemetry(fresh.clone());
        assert_eq!(app.active_profile_name(), "Quiet Night");
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("GPU Night"));
        assert_eq!(
            app.selected_host_channel().unwrap().curve.source,
            HostTemperatureSource::Gpu
        );
        assert_eq!(app.selected_host_channel().unwrap().curve.points.len(), 2);
        assert!(!app.host_policy_is_dirty());
        assert_eq!(app.host_control_state, HostControlState::Available);
        assert!(app.snapshot.host_control.active_policy.is_none());
        let mut running = fresh;
        let actual = HostCurve {
            source: HostTemperatureSource::Cpu,
            points: vec![
                HostCurvePoint {
                    temperature_millidegrees: 20_000,
                    duty_percent: 40,
                },
                HostCurvePoint {
                    temperature_millidegrees: 75_000,
                    duty_percent: 100,
                },
            ],
        };
        running.host_control.state = HostControlState::Running;
        running.host_control.active_policy = Some(HostControlPolicy {
            channels: vec![HostChannelPolicy {
                channel_id: running.host_control.channels[0].channel_id.clone(),
                curve: actual.clone(),
            }],
        });
        app.update_telemetry(running);
        assert_eq!(app.selected_host_channel().unwrap().curve, actual);
        assert_ne!(app.active_host_profile_name().as_deref(), Some("GPU Night"));
    }

    #[test]
    fn changing_tracked_host_source_is_custom_until_successful_auto_profile_ack() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::new(snapshot.clone(), "DEMO");
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::Home));
        app.handle_key_event(key(KeyCode::Enter));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Silent"));
        app.handle_key_event(key(KeyCode::Char('s')));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
        let policy = app.host_editor.policy().channels[0].clone();
        let intents = app.host_profile_intents(&[policy]);
        assert_eq!(intents[0].preferred, None);
        let mut library = app.profile_library.clone();
        let bindings = library.prepare(&intents).unwrap();
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
        library.bind_success(&bindings).unwrap();
        app.set_profile_library(library);
        app.profile_bindings_applied(&bindings);
        assert_eq!(
            app.active_host_profile_name().as_deref(),
            Some("my-custom-profile-1")
        );

        let custom_curve = built_in_host_profiles(HostTemperatureSource::Cpu, 30)[0]
            .curve
            .clone();
        let mut custom_library = ProfileLibrary::default();
        custom_library.profiles.push(crate::profile::SavedProfile {
            id: 1,
            name: "Night Fan".into(),
            curve: ProfileCurve::Host(custom_curve),
        });
        custom_library.next_index = 2;
        let mut app =
            App::with_profile_library(snapshot, "DEMO", custom_library, AppConfig::default());
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Enter));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Night Fan"));
        app.handle_key_event(key(KeyCode::Char('s')));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
        assert_eq!(
            app.host_profile_intents(&[app.host_editor.policy().channels[0].clone()])[0].preferred,
            None
        );
    }

    #[test]
    fn host_custom_picker_preserves_source_domain_and_rejects_firmware_entries() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut app = App::with_profile_library(
            snapshot.clone(),
            "DEMO",
            fixture_library(&snapshot),
            AppConfig::default(),
        );
        app.handle_key_event(key(KeyCode::BackTab));
        assert_eq!(app.host_profiles().len(), 4);
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Enter));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("GPU Night"));
        assert_eq!(app.selected_host_channel().unwrap().curve.points.len(), 2);
        assert_eq!(
            app.selected_host_channel().unwrap().curve.source,
            HostTemperatureSource::Gpu
        );
        app.handle_key_event(key(KeyCode::Char('s')));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
        let intents = app.host_profile_intents(&app.host_editor.policy().channels);
        assert_eq!(intents[0].preferred, None);
        app.handle_key_event(key(KeyCode::Tab));
        assert_eq!(app.profiles.len(), 4);
        assert!(app.profiles.iter().all(|p| p.name != "GPU Night"));
    }

    #[test]
    fn inactive_highlighted_rename_updates_shared_binding_names_without_hardware() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let mut library = fixture_library(&snapshot);
        let second = snapshot.devices[0].cooling_channels[1].id.clone();
        let first = library.bindings[0].clone();
        library.bindings.push(ProfileBinding {
            target: ProfileTarget::Firmware {
                device_id: snapshot.devices[0].id.clone(),
                channel_id: second.clone(),
            },
            ..first.clone()
        });
        let mut app =
            App::with_profile_library(snapshot, "DEMO", library.clone(), AppConfig::default());
        app.handle_key_event(key(KeyCode::Char('c')));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Char('r')));
        assert_eq!(app.modal, Some(Modal::RenameProfile));
        library.rename(1, "Evening Mix").unwrap();
        let before = app.snapshot.devices.clone();
        app.set_profile_library(library);
        app.rename_succeeded(1);
        assert_eq!(app.snapshot.devices, before);
        assert_eq!(app.selected_profile().unwrap().name, "Evening Mix");
        app.handle_key_event(key(KeyCode::Esc));
        assert_eq!(app.active_profile_name(), "Custom"); // second channel is applied, not loaded
        app.handle_key_event(key(KeyCode::Char('[')));
        assert_eq!(app.active_profile_name(), "Evening Mix");
    }

    #[test]
    fn host_all_normalizes_each_minimum_and_keeps_unchanged_custom_identity() {
        let mut backend = DemoBackend::new();
        let mut snapshot = backend.refresh().unwrap();
        snapshot.host_control.channels[1].minimum_duty_percent = 65;
        let library = fixture_library(&snapshot);
        let mut app = App::with_profile_library(snapshot, "DEMO", library, AppConfig::default());
        app.host_start_succeeded(app.host_editor.policy());
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Char('p')));
        app.handle_key_event(key(KeyCode::End));
        app.handle_key_event(key(KeyCode::Enter)); // running selection opens scope
        assert_eq!(app.modal, Some(Modal::HostApplyScope));
        app.handle_key_event(key(KeyCode::End)); // All
        let Some(AppCommand::UpdateHostControl { channel_policies }) =
            app.handle_key_event(key(KeyCode::Enter))
        else {
            panic!("expected update")
        };
        assert_eq!(channel_policies.len(), 2);
        for (index, policy) in channel_policies.iter().enumerate() {
            assert_eq!(policy.curve.points.len(), 2);
            assert_eq!(policy.curve.source, HostTemperatureSource::Gpu);
            assert!(policy.curve.points.iter().all(|p| {
                p.duty_percent
                    >= app.host_editor.channels[index]
                        .capability
                        .minimum_duty_percent
            }));
        }
        let intents = app.host_profile_intents(&channel_policies);
        assert!(
            intents
                .iter()
                .all(|i| i.preferred == Some(ProfileRef::Custom(2)))
        );
        assert_ne!(intents[0].curve, intents[1].curve);
        app.handle_key_event(key(KeyCode::Char('s')));
        assert_eq!(app.active_host_profile_name().as_deref(), Some("Custom"));
        assert_eq!(
            app.host_profile_intents(&[app.host_editor.policy().channels[0].clone()])[0].preferred,
            None
        );
    }

    #[test]
    fn rename_is_available_even_when_host_control_is_unavailable_or_unknown() {
        let mut backend = DemoBackend::new();
        let snapshot = backend.refresh().unwrap();
        let library = fixture_library(&snapshot);
        let mut app = App::with_profile_library(snapshot, "DEMO", library, AppConfig::default());
        app.handle_key_event(key(KeyCode::BackTab));
        for (state, unknown) in [
            (HostControlState::Disabled, false),
            (HostControlState::Available, true),
        ] {
            app.host_control_state = state;
            app.host_status_trusted = !unknown;
            app.handle_key_event(key(KeyCode::Char('p')));
            assert_eq!(app.modal, Some(Modal::HostProfiles));
            app.handle_key_event(key(KeyCode::End));
            app.handle_key_event(key(KeyCode::Char('r')));
            assert_eq!(app.modal, Some(Modal::RenameProfile));
            assert_eq!(app.profile_name_input, "GPU Night");
            app.handle_key_event(key(KeyCode::Esc));
            assert_eq!(app.modal, None);
            assert_eq!(
                app.host_editor.channels[0].curve.source,
                HostTemperatureSource::Gpu
            );
        }
    }

    #[test]
    fn confirmed_noop_host_apply_remembers_without_update_rpc() {
        let mut app = app();
        let policy = app.host_editor.policy();
        app.host_start_succeeded(policy);
        app.handle_key_event(key(KeyCode::BackTab));
        app.handle_key_event(key(KeyCode::Enter));
        assert_eq!(app.modal, Some(Modal::HostApplyScope));
        let command = app.handle_key_event(key(KeyCode::Enter));
        assert!(
            matches!(command, Some(AppCommand::RememberHostProfiles { channel_policies }) if channel_policies.len() == 1)
        );
    }

    #[test]
    fn control_c_exits_instead_of_changing_channels() {
        let mut app = app();
        let event = KeyEvent::new(KeyCode::Char('c'), KeyModifiers::CONTROL);

        app.handle_key_event(event);

        assert!(app.exit);
        assert_eq!(app.selected_channel, 0);
    }
}
