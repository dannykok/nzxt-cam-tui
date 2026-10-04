use std::{
    env,
    error::Error,
    fmt, fs,
    fs::{File, OpenOptions},
    io::{self, Write},
    path::{Path, PathBuf},
    process,
    sync::atomic::{AtomicU64, Ordering},
};

use fs2::FileExt;
use serde::{Deserialize, Serialize};

use crate::config::DefaultProfile;
use crate::model::{
    ChannelId, CurvePoint, DeviceId, HostCurve, HostCurvePoint, HostTemperatureSource,
};

const PROFILE_FILE_VERSION: u64 = 2;
static TEMP_FILE_COUNTER: AtomicU64 = AtomicU64::new(0);
const CURVE_POINT_COUNT: usize = 40;
pub const MAX_PROFILE_NAME_CHARS: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileKind {
    BuiltIn,
    Custom,
}

impl ProfileKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::BuiltIn => "BUILT-IN",
            Self::Custom => "CUSTOM",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CurveProfile {
    pub name: String,
    pub kind: ProfileKind,
    pub points: Vec<CurvePoint>,
}

impl CurveProfile {
    fn built_in(name: &str, stops: &[(u8, u8)]) -> Self {
        Self {
            name: name.into(),
            kind: ProfileKind::BuiltIn,
            points: curve_from_stops(stops),
        }
    }

    pub fn custom(name: impl Into<String>, points: Vec<CurvePoint>) -> Result<Self, ProfileError> {
        let name = validate_name(name.into())?;
        validate_points(&points)?;
        Ok(Self {
            name,
            kind: ProfileKind::Custom,
            points,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HostCurveProfile {
    pub name: String,
    pub curve: HostCurve,
}

/// Rescale the built-in AIO shapes to host temperatures, flooring duties at the minimum.
pub fn built_in_host_profiles(
    source: HostTemperatureSource,
    minimum_duty_percent: u8,
) -> Vec<HostCurveProfile> {
    let minimum = minimum_duty_percent.min(100);
    built_in_profiles()
        .into_iter()
        .map(|profile| HostCurveProfile {
            name: profile.name,
            curve: HostCurve {
                source,
                points: profile
                    .points
                    .into_iter()
                    .enumerate()
                    .map(|(index, point)| HostCurvePoint {
                        temperature_millidegrees: 22_000 + 2_000 * index as i32,
                        duty_percent: point.duty.max(minimum),
                    })
                    .collect(),
            },
        })
        .collect()
}

pub fn built_in_profiles() -> Vec<CurveProfile> {
    vec![
        CurveProfile::built_in(
            "Silent",
            &[
                (20, 20),
                (30, 25),
                (35, 30),
                (40, 42),
                (45, 60),
                (50, 82),
                (59, 100),
            ],
        ),
        CurveProfile::built_in(
            "Progressive",
            &[
                (20, 30),
                (28, 35),
                (34, 45),
                (40, 58),
                (46, 73),
                (52, 90),
                (59, 100),
            ],
        ),
        CurveProfile::built_in(
            "Performance",
            &[
                (20, 50),
                (28, 55),
                (34, 65),
                (40, 75),
                (46, 85),
                (52, 95),
                (59, 100),
            ],
        ),
    ]
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BuiltinProfile {
    Silent,
    Progressive,
    Performance,
}
impl BuiltinProfile {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Silent => "Silent",
            Self::Progressive => "Progressive",
            Self::Performance => "Performance",
        }
    }
    const ALL: [Self; 3] = [Self::Silent, Self::Progressive, Self::Performance];
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProfileRef {
    BuiltIn(BuiltinProfile),
    Custom(u64),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProfileCurve {
    Firmware(Vec<CurvePoint>),
    Host(HostCurve),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProfileTarget {
    Firmware {
        device_id: DeviceId,
        channel_id: ChannelId,
    },
    Host {
        channel_id: ChannelId,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SavedProfile {
    pub id: u64,
    pub name: String,
    pub curve: ProfileCurve,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProfileBinding {
    pub target: ProfileTarget,
    pub profile: ProfileRef,
    pub curve: ProfileCurve,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProfileIntent {
    pub target: ProfileTarget,
    pub curve: ProfileCurve,
    pub preferred: Option<ProfileRef>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProfileLibrary {
    pub profiles: Vec<SavedProfile>,
    pub bindings: Vec<ProfileBinding>,
    pub proposals: Vec<ProfileBinding>,
    pub next_index: u64,
}
impl Default for ProfileLibrary {
    fn default() -> Self {
        Self {
            profiles: vec![],
            bindings: vec![],
            proposals: vec![],
            next_index: 1,
        }
    }
}
impl ProfileLibrary {
    pub fn from_legacy(
        profiles: &[CurveProfile],
        defaults: &[DefaultProfile],
    ) -> Result<Self, ProfileError> {
        let mut library = Self::default();
        for profile in profiles {
            if profile.kind != ProfileKind::Custom {
                return Err(invalid("legacy profile must be custom"));
            }
            let name = validate_name(profile.name.clone())?;
            validate_points(&profile.points)?;
            if library.reference_by_name(&name).is_some() {
                return Err(invalid("duplicate legacy profile name"));
            }
            let id = library.allocate_id()?;
            library.profiles.push(SavedProfile {
                id,
                name,
                curve: ProfileCurve::Firmware(profile.points.clone()),
            });
        }
        for saved in defaults {
            let target = ProfileTarget::Firmware {
                device_id: DeviceId::new(&saved.device_id),
                channel_id: ChannelId::new(&saved.channel_id),
            };
            validate_target(&target)?;
            let profile = library
                .reference_by_name(&saved.profile)
                .ok_or_else(|| invalid(format!("unknown legacy profile: {}", saved.profile)))?;
            let curve = match profile {
                ProfileRef::Custom(id) => library
                    .profiles
                    .iter()
                    .find(|p| p.id == id)
                    .unwrap()
                    .curve
                    .clone(),
                ProfileRef::BuiltIn(preset) => ProfileCurve::Firmware(
                    built_in_profiles()
                        .into_iter()
                        .find(|p| p.name == preset.name())
                        .unwrap()
                        .points,
                ),
            };
            library.bindings.retain(|b| b.target != target); // Legacy config's last matching entry wins.
            library.bindings.push(ProfileBinding {
                target,
                profile,
                curve,
            });
        }
        library.validate()?;
        Ok(library)
    }
    pub fn name(&self, reference: ProfileRef) -> Option<&str> {
        match reference {
            ProfileRef::BuiltIn(value) => Some(value.name()),
            ProfileRef::Custom(id) => self
                .profiles
                .iter()
                .find(|p| p.id == id)
                .map(|p| p.name.as_str()),
        }
    }
    pub fn reference_by_name(&self, name: &str) -> Option<ProfileRef> {
        BuiltinProfile::ALL
            .into_iter()
            .find(|p| p.name().eq_ignore_ascii_case(name))
            .map(ProfileRef::BuiltIn)
            .or_else(|| {
                self.profiles
                    .iter()
                    .find(|p| p.name.eq_ignore_ascii_case(name))
                    .map(|p| ProfileRef::Custom(p.id))
            })
    }
    pub fn firmware_profiles(&self) -> Vec<CurveProfile> {
        self.profiles
            .iter()
            .filter_map(|p| match &p.curve {
                ProfileCurve::Firmware(points) => Some(CurveProfile {
                    name: p.name.clone(),
                    kind: ProfileKind::Custom,
                    points: points.clone(),
                }),
                ProfileCurve::Host(_) => None,
            })
            .collect()
    }
    fn allocate_id(&mut self) -> Result<u64, ProfileError> {
        let id = self.next_index;
        if id == 0 {
            return Err(invalid("profile ID exhausted"));
        }
        self.next_index = id
            .checked_add(1)
            .ok_or_else(|| invalid("profile ID exhausted"))?;
        Ok(id)
    }
    pub fn prepare(
        &mut self,
        intents: &[ProfileIntent],
    ) -> Result<Vec<ProfileBinding>, ProfileError> {
        self.validate()?;
        let mut copy = self.clone();
        let mut result: Vec<ProfileBinding> = vec![];
        let mut created: Vec<(ProfileCurve, ProfileRef)> = vec![];
        for intent in intents {
            validate_target(&intent.target)?;
            validate_curve(&intent.curve)?;
            if !domain_matches(&intent.target, &intent.curve) {
                return Err(invalid("profile target and curve domains differ"));
            }
            if result.iter().any(|b| b.target == intent.target) {
                return Err(invalid("duplicate target in profile batch"));
            }
            let profile = if let Some(reference) = intent.preferred {
                let existing = match reference {
                    ProfileRef::BuiltIn(_) => true,
                    ProfileRef::Custom(id) => copy
                        .profiles
                        .iter()
                        .any(|p| p.id == id && same_domain(&p.curve, &intent.curve)),
                };
                if !existing {
                    return Err(invalid("preferred profile does not exist in this domain"));
                }
                reference
            } else if let Some((_, reference)) =
                created.iter().find(|(curve, _)| *curve == intent.curve)
            {
                *reference
            } else {
                let (id, name) = loop {
                    let id = copy.allocate_id()?;
                    let name = format!("my-custom-profile-{id}");
                    if name.chars().count() > MAX_PROFILE_NAME_CHARS {
                        return Err(invalid("automatic profile name index exhausted"));
                    }
                    // A user may rename an earlier entry to a future generated
                    // name. Skip that name without overwriting it or blocking
                    // all later applies; the allocation remains monotonic.
                    if copy.reference_by_name(&name).is_none() {
                        break (id, name);
                    }
                };
                copy.profiles.push(SavedProfile {
                    id,
                    name,
                    curve: intent.curve.clone(),
                });
                created.push((intent.curve.clone(), ProfileRef::Custom(id)));
                ProfileRef::Custom(id)
            };
            result.push(ProfileBinding {
                target: intent.target.clone(),
                profile,
                curve: intent.curve.clone(),
            });
        }
        for binding in &result {
            if !copy.proposals.contains(binding) {
                copy.proposals.push(binding.clone());
            }
        }
        copy.validate()?;
        *self = copy;
        Ok(result)
    }
    pub fn bind_success(&mut self, bindings: &[ProfileBinding]) -> Result<(), ProfileError> {
        self.validate()?;
        let mut copy = self.clone();
        for binding in bindings {
            if !copy.proposals.contains(binding) {
                return Err(invalid(
                    "successful binding has no matching prepared proposal",
                ));
            }
            if !copy.binding_valid(binding) {
                return Err(invalid("invalid successful profile binding"));
            }
            copy.proposals.retain(|b| b != binding);
            copy.bindings.retain(|b| b.target != binding.target);
            copy.bindings.push(binding.clone());
        }
        copy.validate()?;
        *self = copy;
        Ok(())
    }
    pub fn rename(&mut self, id: u64, name: &str) -> Result<(), ProfileError> {
        self.validate()?;
        let name = validate_name(name.to_owned())?;
        if self
            .profiles
            .iter()
            .any(|p| p.id != id && p.name.eq_ignore_ascii_case(&name))
        {
            return Err(invalid("profile name already exists"));
        }
        let profile = self
            .profiles
            .iter_mut()
            .find(|p| p.id == id)
            .ok_or_else(|| invalid("custom profile ID not found"))?;
        profile.name = name;
        Ok(())
    }
    fn binding_valid(&self, binding: &ProfileBinding) -> bool {
        if validate_target(&binding.target).is_err()
            || validate_curve(&binding.curve).is_err()
            || !domain_matches(&binding.target, &binding.curve)
        {
            return false;
        }
        match binding.profile {
            ProfileRef::BuiltIn(_) => true, // Clamped/source-specific accepted curves need not equal generated presets.
            ProfileRef::Custom(id) => self
                .profiles
                .iter()
                .any(|p| p.id == id && same_domain(&p.curve, &binding.curve)),
        }
    }
    fn validate(&self) -> Result<(), ProfileError> {
        if self.next_index == 0 {
            return Err(invalid("invalid next profile index"));
        }
        for (i, profile) in self.profiles.iter().enumerate() {
            if profile.id == 0
                || profile.id >= self.next_index
                || validate_name(profile.name.clone())? != profile.name
                || self.profiles[..i]
                    .iter()
                    .any(|p| p.id == profile.id || p.name.eq_ignore_ascii_case(&profile.name))
            {
                return Err(invalid("invalid or duplicate stored profile ID/name"));
            }
            validate_curve(&profile.curve)?;
        }
        for (i, binding) in self.bindings.iter().enumerate() {
            if !self.binding_valid(binding)
                || self.bindings[..i]
                    .iter()
                    .any(|b| b.target == binding.target)
            {
                return Err(invalid("invalid or duplicate successful binding"));
            }
        }
        for (i, proposal) in self.proposals.iter().enumerate() {
            if !self.binding_valid(proposal) || self.proposals[..i].contains(proposal) {
                return Err(invalid("invalid or duplicate prepared proposal"));
            }
        }
        Ok(())
    }
}
fn same_domain(a: &ProfileCurve, b: &ProfileCurve) -> bool {
    matches!(
        (a, b),
        (ProfileCurve::Firmware(_), ProfileCurve::Firmware(_))
            | (ProfileCurve::Host(_), ProfileCurve::Host(_))
    )
}
fn domain_matches(target: &ProfileTarget, curve: &ProfileCurve) -> bool {
    matches!(
        (target, curve),
        (ProfileTarget::Firmware { .. }, ProfileCurve::Firmware(_))
            | (ProfileTarget::Host { .. }, ProfileCurve::Host(_))
    )
}
fn validate_target(target: &ProfileTarget) -> Result<(), ProfileError> {
    let valid = match target {
        ProfileTarget::Firmware {
            device_id,
            channel_id,
        } => !device_id.0.trim().is_empty() && !channel_id.0.trim().is_empty(),
        ProfileTarget::Host { channel_id } => !channel_id.0.trim().is_empty(),
    };
    if !valid {
        return Err(invalid("profile target IDs cannot be blank"));
    }
    Ok(())
}
fn validate_curve(curve: &ProfileCurve) -> Result<(), ProfileError> {
    match curve {
        ProfileCurve::Firmware(points) => validate_points(points),
        ProfileCurve::Host(curve) => {
            if !(2..=64).contains(&curve.points.len()) {
                return Err(invalid("host curves require 2..=64 points"));
            }
            let mut previous = None;
            let mut previous_duty = 0;
            for point in &curve.points {
                if !(0..=120_000).contains(&point.temperature_millidegrees)
                    || previous.is_some_and(|p| point.temperature_millidegrees <= p)
                    || point.duty_percent > 100
                    || point.duty_percent < previous_duty
                {
                    return Err(invalid("invalid host curve temperatures or duties"));
                }
                previous = Some(point.temperature_millidegrees);
                previous_duty = point.duty_percent;
            }
            if previous_duty != 100 {
                return Err(invalid("host curve must end at 100%"));
            }
            Ok(())
        }
    }
}
fn invalid(message: impl Into<String>) -> ProfileError {
    ProfileError::Invalid(message.into())
}

#[derive(Clone, Debug)]
pub struct ProfileStore {
    path: PathBuf,
    legacy_defaults: Vec<DefaultProfile>,
    legacy_defaults_available: bool,
    demo: bool,
}
impl ProfileStore {
    pub fn at(path: impl Into<PathBuf>) -> Self {
        Self {
            path: path.into(),
            legacy_defaults: vec![],
            legacy_defaults_available: true,
            demo: false,
        }
    }
    pub fn from_environment() -> Result<Self, ProfileError> {
        let config_root = env::var_os("XDG_CONFIG_HOME")
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
            .or_else(|| {
                env::var_os("HOME")
                    .filter(|v| !v.is_empty())
                    .map(|v| PathBuf::from(v).join(".config"))
            })
            .ok_or_else(|| {
                invalid("neither XDG_CONFIG_HOME nor HOME is available for profile storage")
            })?;
        Ok(Self::at(
            config_root.join("nzxt-cam-tui").join("profiles.json"),
        ))
    }
    pub fn for_demo() -> Result<Self, ProfileError> {
        let mut store = Self::from_environment()?;
        store.path.set_file_name("demo-profiles.json");
        store.demo = true;
        Ok(store)
    }
    pub fn with_legacy_defaults(mut self, defaults: Vec<DefaultProfile>) -> Self {
        if !self.demo {
            self.legacy_defaults = defaults;
            self.legacy_defaults_available = true;
        }
        self
    }
    pub fn with_unavailable_legacy_defaults(mut self) -> Self {
        if !self.demo {
            self.legacy_defaults_available = false;
        }
        self
    }
    fn migrate_legacy(&self, profiles: &[CurveProfile]) -> Result<ProfileLibrary, ProfileError> {
        if !self.legacy_defaults_available {
            return Err(invalid(
                "cannot migrate profile selections while configuration is unreadable",
            ));
        }
        ProfileLibrary::from_legacy(profiles, &self.legacy_defaults)
    }
    pub fn path(&self) -> &Path {
        &self.path
    }
    pub fn load(&self) -> Result<ProfileLibrary, ProfileError> {
        self.read_latest().map(|(library, _)| library)
    }
    // v1 bytes are retained verbatim until the first committed v2 mutation.
    fn read_latest(&self) -> Result<(ProfileLibrary, Option<Vec<u8>>), ProfileError> {
        let raw = match fs::read(&self.path) {
            Ok(raw) => raw,
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                return Ok((self.migrate_legacy(&[])?, None));
            }
            Err(error) => return Err(error.into()),
        };
        let value: serde_json::Value = serde_json::from_slice(&raw)?;
        let version = value
            .get("version")
            .and_then(serde_json::Value::as_u64)
            .ok_or_else(|| invalid("missing or invalid profile file version"))?;
        match version {
            PROFILE_FILE_VERSION => {
                let stored: StoredV2 = serde_json::from_slice(&raw)?;
                stored.library.validate()?;
                Ok((stored.library, None))
            }
            1 => {
                let stored: StoredV1 = serde_json::from_slice(&raw)?;
                if stored.version != 1 {
                    return Err(invalid("unsupported legacy profile version"));
                }
                let legacy = stored
                    .profiles
                    .into_iter()
                    .map(StoredProfile::into_profile)
                    .collect::<Result<Vec<_>, _>>()?;
                Ok((self.migrate_legacy(&legacy)?, Some(raw)))
            }
            _ => Err(invalid(format!(
                "unsupported profile file version {version}"
            ))),
        }
    }
    fn transaction<T>(
        &self,
        operation: impl FnOnce(&mut ProfileLibrary) -> Result<T, ProfileError>,
    ) -> Result<(ProfileLibrary, T), ProfileError> {
        if let Some(parent) = self.path.parent() {
            create_directory_durable(parent)?;
        }
        let lock = OpenOptions::new()
            .create(true)
            .truncate(false)
            .read(true)
            .write(true)
            .open(self.path.with_extension("lock"))?;
        lock.lock_exclusive()?;
        let outcome = (|| {
            let (mut library, old_v1) = self.read_latest()?;
            let value = operation(&mut library)?;
            library.validate()?;
            if let Some(raw) = old_v1 {
                self.backup_v1(&raw)?;
            }
            self.write_v2(&library)?;
            Ok((library, value))
        })();
        let unlock = FileExt::unlock(&lock);
        match (outcome, unlock) {
            (Err(e), _) => Err(e),
            (Ok(_), Err(e)) => Err(e.into()),
            (Ok(v), Ok(())) => Ok(v),
        }
    }
    fn backup_v1(&self, raw: &[u8]) -> Result<(), ProfileError> {
        let backup = self.path.with_extension("json.v1.bak");
        match OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&backup)
        {
            Ok(mut file) => {
                // On failure, leave the original v1 untouched and never claim success.
                if let Err(e) = file
                    .write_all(raw)
                    .and_then(|_| file.sync_all())
                    .and_then(|_| sync_parent(&backup))
                {
                    let _ = fs::remove_file(&backup);
                    return Err(e.into());
                }
            }
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                if fs::read(&backup)? != raw {
                    return Err(invalid("existing v1 backup differs from the original"));
                }
            }
            Err(e) => return Err(e.into()),
        }
        Ok(())
    }
    fn write_v2(&self, library: &ProfileLibrary) -> Result<(), ProfileError> {
        let bytes = serde_json::to_vec_pretty(&StoredV2 {
            version: PROFILE_FILE_VERSION,
            library: library.clone(),
        })?;
        let filename = self
            .path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("profiles.json");
        let nonce = TEMP_FILE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let temporary = self
            .path
            .with_file_name(format!(".{filename}.{}-{nonce}.tmp", process::id()));
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temporary)?;
        let outcome = (|| {
            file.write_all(&bytes)?;
            file.write_all(b"\n")?;
            file.sync_all()?;
            fs::rename(&temporary, &self.path)?;
            sync_parent(&self.path)?;
            Ok(())
        })();
        if outcome.is_err() {
            let _ = fs::remove_file(&temporary);
        }
        outcome
    }
    pub fn prepare(
        &self,
        intents: &[ProfileIntent],
    ) -> Result<(ProfileLibrary, Vec<ProfileBinding>), ProfileError> {
        self.transaction(|library| library.prepare(intents))
    }
    pub fn bind_success(
        &self,
        bindings: &[ProfileBinding],
    ) -> Result<ProfileLibrary, ProfileError> {
        self.transaction(|library| library.bind_success(bindings))
            .map(|(library, _)| library)
    }
    pub fn rename(&self, id: u64, name: &str) -> Result<ProfileLibrary, ProfileError> {
        self.transaction(|library| library.rename(id, name))
            .map(|(library, _)| library)
    }
}
fn create_directory_durable(path: &Path) -> io::Result<()> {
    create_directory_with_sync(path, &mut |directory| File::open(directory)?.sync_all())
}

fn create_directory_with_sync(
    path: &Path,
    sync: &mut impl FnMut(&Path) -> io::Result<()>,
) -> io::Result<()> {
    if path.as_os_str().is_empty() {
        return Ok(());
    }
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    if path.is_dir() {
        // The directory may have just been created by a concurrent first save.
        sync(path)?;
        return sync(parent);
    }
    create_directory_with_sync(parent, sync)?;
    match fs::create_dir(path) {
        Ok(()) => {
            if let Err(error) = sync(path).and_then(|_| sync(parent)) {
                // Only remove the empty directory just created by this call.
                // A retry must not mistake an unsynced creation for durable data.
                let _ = fs::remove_dir(path);
                return Err(error);
            }
        }
        Err(error) if error.kind() == io::ErrorKind::AlreadyExists && path.is_dir() => {
            // Another creator may still be between mkdir and its parent sync.
            sync(path)?;
            sync(parent)?;
        }
        Err(error) => return Err(error),
    }
    Ok(())
}

fn sync_parent(path: &Path) -> io::Result<()> {
    File::open(
        path.parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new(".")),
    )?
    .sync_all()
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct StoredV2 {
    version: u64,
    #[serde(flatten)]
    library: ProfileLibrary,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct StoredV1 {
    version: u64,
    profiles: Vec<StoredProfile>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct StoredProfile {
    name: String,
    duties: Vec<u8>,
}
impl StoredProfile {
    fn into_profile(self) -> Result<CurveProfile, ProfileError> {
        if self.duties.len() != CURVE_POINT_COUNT {
            return Err(invalid("legacy profile must contain exactly 40 duties"));
        }
        CurveProfile::custom(
            self.name,
            (20_u8..60)
                .zip(self.duties)
                .map(|(temperature, duty)| CurvePoint { temperature, duty })
                .collect(),
        )
    }
}

#[derive(Debug)]
pub enum ProfileError {
    Io(io::Error),
    Json(serde_json::Error),
    Invalid(String),
}
impl fmt::Display for ProfileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "profile file error: {e}"),
            Self::Json(e) => write!(f, "invalid profile JSON: {e}"),
            Self::Invalid(e) => f.write_str(e),
        }
    }
}
impl Error for ProfileError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            Self::Json(e) => Some(e),
            Self::Invalid(_) => None,
        }
    }
}
impl From<io::Error> for ProfileError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}
impl From<serde_json::Error> for ProfileError {
    fn from(e: serde_json::Error) -> Self {
        Self::Json(e)
    }
}

fn validate_name(name: String) -> Result<String, ProfileError> {
    let name = name.trim().to_owned();
    if name.is_empty()
        || name.chars().count() > MAX_PROFILE_NAME_CHARS
        || name.chars().any(char::is_control)
    {
        return Err(invalid(
            "profile name must contain 1..=32 printable characters",
        ));
    }
    if BuiltinProfile::ALL
        .into_iter()
        .any(|p| p.name().eq_ignore_ascii_case(&name))
    {
        return Err(invalid("built-in profile names are reserved"));
    }
    Ok(name)
}
fn validate_points(points: &[CurvePoint]) -> Result<(), ProfileError> {
    if points.len() != CURVE_POINT_COUNT
        || points
            .iter()
            .enumerate()
            .any(|(i, p)| p.temperature != 20 + i as u8 || p.duty > 100)
    {
        return Err(invalid(
            "firmware curves require 40 duties at 20..59°C, each <=100%",
        ));
    }
    Ok(())
}

fn curve_from_stops(stops: &[(u8, u8)]) -> Vec<CurvePoint> {
    (20_u8..60)
        .map(|temperature| CurvePoint {
            temperature,
            duty: interpolate_stops(stops, temperature),
        })
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
    use std::{
        sync::{Arc, Barrier},
        thread,
        time::SystemTime,
    };

    fn temp() -> (PathBuf, ProfileStore) {
        let nonce = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = env::temp_dir().join(format!("nzxt-profile-{}-{nonce}", process::id()));
        (dir.clone(), ProfileStore::at(dir.join("profiles.json")))
    }
    fn fw() -> ProfileCurve {
        ProfileCurve::Firmware(built_in_profiles()[0].points.clone())
    }
    fn host(source: HostTemperatureSource) -> ProfileCurve {
        ProfileCurve::Host(HostCurve {
            source,
            points: vec![
                HostCurvePoint {
                    temperature_millidegrees: 30_000,
                    duty_percent: 20,
                },
                HostCurvePoint {
                    temperature_millidegrees: 80_000,
                    duty_percent: 100,
                },
            ],
        })
    }
    fn target(name: &str) -> ProfileTarget {
        ProfileTarget::Firmware {
            device_id: DeviceId::new("kraken"),
            channel_id: ChannelId::new(name),
        }
    }
    fn intent(target: ProfileTarget, curve: ProfileCurve) -> ProfileIntent {
        ProfileIntent {
            target,
            curve,
            preferred: None,
        }
    }
    #[test]
    fn newly_created_profile_directories_sync_their_parent_entries() {
        let (dir, _) = temp();
        let nested = dir.join("config").join("nzxt-cam-tui");
        let mut synced = Vec::new();
        create_directory_with_sync(&nested, &mut |p| {
            synced.push(p.to_path_buf());
            Ok(())
        })
        .unwrap();
        for created in [&dir, &dir.join("config"), &nested] {
            assert!(synced.contains(created));
            assert!(synced.contains(&created.parent().unwrap().to_path_buf()));
        }
        assert!(nested.is_dir());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn failed_parent_sync_removes_only_the_new_empty_directory_and_retry_syncs() {
        let (dir, _) = temp();
        fs::create_dir(&dir).unwrap();
        fs::write(dir.join("keep"), b"existing data").unwrap();
        let child = dir.join("profiles");
        assert!(
            create_directory_with_sync(&child, &mut |p| {
                if p == dir && child.is_dir() {
                    Err(io::Error::other("injected parent fsync failure"))
                } else {
                    Ok(())
                }
            })
            .is_err()
        );
        assert!(!child.exists());
        let mut synced = Vec::new();
        create_directory_with_sync(&child, &mut |p| {
            synced.push(p.to_path_buf());
            Ok(())
        })
        .unwrap();
        assert!(synced.contains(&dir));
        assert_eq!(fs::read(dir.join("keep")).unwrap(), b"existing data");
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn unreadable_config_blocks_legacy_migration_but_not_v2_or_demo_profiles() {
        let (dir, store) = temp();
        let unavailable = store.clone().with_unavailable_legacy_defaults();
        assert!(
            unavailable
                .prepare(&[intent(target("pump"), fw())])
                .is_err()
        );
        assert!(!store.path().exists());
        fs::write(store.path(), br#"{"version":1,"profiles":[]}"#).unwrap();
        assert!(
            unavailable
                .prepare(&[intent(target("pump"), fw())])
                .is_err()
        );
        assert_eq!(
            fs::read(store.path()).unwrap(),
            br#"{"version":1,"profiles":[]}"#
        );
        store.prepare(&[intent(target("pump"), fw())]).unwrap();
        assert!(unavailable.load().is_ok());
        unavailable.rename(1, "Independent profile").unwrap();
        assert_eq!(
            store.load().unwrap().name(ProfileRef::Custom(1)),
            Some("Independent profile")
        );
        let mut demo = ProfileStore::at(dir.join("demo-profiles.json"));
        demo.demo = true;
        demo.with_unavailable_legacy_defaults()
            .prepare(&[intent(target("pump"), fw())])
            .unwrap();
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn built_in_profiles_remain_distinct() {
        assert_eq!(built_in_profiles().len(), 3);
        assert_ne!(built_in_profiles()[0].points, built_in_profiles()[1].points);
        assert_eq!(
            built_in_host_profiles(HostTemperatureSource::Gpu, 40)[0]
                .curve
                .points[0]
                .duty_percent,
            40
        );
    }
    #[test]
    fn all_batch_deduplicates_exact_curves_not_historical_or_other_domains() {
        let mut library = ProfileLibrary::default();
        let bindings = library
            .prepare(&[
                intent(target("pump"), fw()),
                intent(target("fan"), fw()),
                intent(
                    ProfileTarget::Host {
                        channel_id: ChannelId::new("host-a"),
                    },
                    host(HostTemperatureSource::Cpu),
                ),
                intent(
                    ProfileTarget::Host {
                        channel_id: ChannelId::new("host-b"),
                    },
                    host(HostTemperatureSource::Gpu),
                ),
            ])
            .unwrap();
        assert_eq!(bindings[0].profile, bindings[1].profile);
        assert_ne!(bindings[2].profile, bindings[3].profile);
        assert_eq!(library.next_index, 4);
        assert_eq!(library.proposals, bindings);
        assert!(library.bindings.is_empty());
        let new = library.prepare(&[intent(target("third"), fw())]).unwrap();
        assert_ne!(bindings[0].profile, new[0].profile);
        library.bind_success(&bindings[0..1]).unwrap();
        assert_eq!(library.bindings, bindings[0..1]);
        assert_eq!(library.proposals.len(), 4);
    }
    #[test]
    fn chosen_custom_does_not_seed_dedup_of_new_edits() {
        let mut library = ProfileLibrary::default();
        let old = library
            .prepare(&[intent(target("pump"), fw())])
            .unwrap()
            .remove(0);
        let batch = library
            .prepare(&[
                ProfileIntent {
                    target: target("fan"),
                    curve: fw(),
                    preferred: Some(old.profile),
                },
                intent(target("extra"), fw()),
                intent(target("other"), fw()),
            ])
            .unwrap();
        assert_eq!(batch[0].profile, old.profile);
        assert_ne!(batch[1].profile, old.profile);
        assert_eq!(batch[1].profile, batch[2].profile);
    }
    #[test]
    fn builtin_host_source_and_clamped_curve_are_durable_before_ack() {
        let (dir, store) = temp();
        let curve = host(HostTemperatureSource::CpuGpuMax);
        let (prepared, proposals) = store
            .prepare(&[ProfileIntent {
                target: ProfileTarget::Host {
                    channel_id: ChannelId::new("case"),
                },
                curve: curve.clone(),
                preferred: Some(ProfileRef::BuiltIn(BuiltinProfile::Silent)),
            }])
            .unwrap();
        assert!(prepared.profiles.is_empty());
        assert!(prepared.bindings.is_empty());
        assert_eq!(prepared.proposals[0].curve, curve);
        assert_eq!(store.load().unwrap().proposals, proposals);
        assert_eq!(store.bind_success(&proposals).unwrap().bindings, proposals);
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn automatic_names_skip_a_user_renamed_future_index() {
        let (dir, store) = temp();
        let (_, first) = store.prepare(&[intent(target("pump"), fw())]).unwrap();
        store.bind_success(&first).unwrap();
        store.rename(1, "MY-CUSTOM-PROFILE-2").unwrap();
        let (library, second) = store.prepare(&[intent(target("fan"), fw())]).unwrap();
        assert_eq!(second[0].profile, ProfileRef::Custom(3));
        assert_eq!(library.name(second[0].profile), Some("my-custom-profile-3"));
        assert_eq!(library.name(first[0].profile), Some("MY-CUSTOM-PROFILE-2"));
        assert_eq!(library.next_index, 4);
        assert_eq!(library.bindings, first);
        assert_eq!(store.load().unwrap(), library);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn invalid_batch_and_overflow_roll_back_all() {
        let mut library = ProfileLibrary::default();
        let original = library.clone();
        assert!(
            library
                .prepare(&[
                    intent(target("pump"), fw()),
                    intent(
                        ProfileTarget::Host {
                            channel_id: ChannelId::new("fan")
                        },
                        fw()
                    )
                ])
                .is_err()
        );
        assert_eq!(library, original);
        library.next_index = u64::MAX;
        let before = library.clone();
        assert!(library.prepare(&[intent(target("pump"), fw())]).is_err());
        assert_eq!(library, before);
        assert!(
            library
                .prepare(&[intent(target("pump"), fw()), intent(target("pump"), fw())])
                .is_err()
        );
    }
    #[test]
    fn preferred_domain_and_rename_keep_stable_binding_identity() {
        let mut library = ProfileLibrary::default();
        let binding = library
            .prepare(&[intent(target("pump"), fw())])
            .unwrap()
            .remove(0);
        let ProfileRef::Custom(id) = binding.profile else {
            panic!()
        };
        assert!(
            library
                .prepare(&[ProfileIntent {
                    target: ProfileTarget::Host {
                        channel_id: ChannelId::new("fan")
                    },
                    curve: host(HostTemperatureSource::Gpu),
                    preferred: Some(binding.profile)
                }])
                .is_err()
        );
        library.bind_success(&[binding]).unwrap();
        library.rename(id, "QUIET Night").unwrap();
        assert_eq!(library.name(ProfileRef::Custom(id)), Some("QUIET Night"));
        assert_eq!(
            library.reference_by_name("quiet night"),
            Some(ProfileRef::Custom(id))
        );
        assert_eq!(library.bindings[0].profile, ProfileRef::Custom(id));
        assert_eq!(library.next_index, 2);
        for bad in [
            "silent",
            "",
            "quiet night",
            "b\tad",
            "x".repeat(33).as_str(),
        ] {
            if bad == "quiet night" {
                continue;
            } // renaming to one's own case-folded name is valid
            assert!(library.rename(id, bad).is_err(), "{bad}");
        }
        assert!(library.rename(999, "unknown").is_err());
        assert_eq!(library.name(ProfileRef::Custom(id)), Some("QUIET Night"));
    }
    #[test]
    fn load_is_read_only_and_v1_backup_is_exact_before_first_mutation() {
        let (dir, store) = temp();
        fs::create_dir_all(&dir).unwrap();
        let duties = vec![25; 40];
        let raw = format!(
            "{{\n \"version\":1,\"profiles\":[{{\"name\":\"Old\",\"duties\":{}}}]\n}}\n",
            serde_json::to_string(&duties).unwrap()
        );
        fs::write(store.path(), &raw).unwrap();
        let store = store.with_legacy_defaults(vec![DefaultProfile {
            device_id: "kraken".into(),
            channel_id: "pump".into(),
            profile: "oLd".into(),
        }]);
        let loaded = store.load().unwrap();
        assert_eq!(loaded.bindings.len(), 1);
        assert_eq!(loaded.next_index, 2);
        assert_eq!(fs::read_to_string(store.path()).unwrap(), raw);
        let backup = store.path().with_extension("json.v1.bak");
        assert!(!backup.exists());
        store.rename(1, "New").unwrap();
        assert_eq!(fs::read(&backup).unwrap(), raw.as_bytes());
        assert_eq!(
            store.load().unwrap().name(ProfileRef::Custom(1)),
            Some("New")
        );
        assert_eq!(
            store.load().unwrap().bindings[0].profile,
            ProfileRef::Custom(1)
        );
        store.rename(1, "Newer").unwrap();
        assert_eq!(fs::read(&backup).unwrap(), raw.as_bytes());
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn existing_mismatched_backup_blocks_v1_mutation() {
        let (dir, store) = temp();
        fs::create_dir_all(&dir).unwrap();
        let original = r#"{"version":1,"profiles":[]}"#;
        fs::write(store.path(), original).unwrap();
        let backup = store.path().with_extension("json.v1.bak");
        fs::write(&backup, b"different backup").unwrap();
        assert!(store.prepare(&[intent(target("pump"), fw())]).is_err());
        assert_eq!(fs::read_to_string(store.path()).unwrap(), original);
        assert_eq!(fs::read(backup).unwrap(), b"different backup");
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn corrupt_future_and_unresolved_legacy_are_not_overwritten() {
        for raw in [
            "not json".to_owned(),
            r#"{"version":99,"profiles":[]}"#.into(),
            r#"{"version":1,"profiles":[]}"#.into(),
            r#"{"version":2,"profiles":[],"bindings":[],"proposals":[],"next_index":0}"#.into(),
        ] {
            let (dir, store) = temp();
            fs::create_dir_all(&dir).unwrap();
            fs::write(store.path(), &raw).unwrap();
            let defaults = vec![DefaultProfile {
                device_id: "kraken".into(),
                channel_id: "pump".into(),
                profile: "unresolved".into(),
            }];
            let store = store.with_legacy_defaults(defaults);
            assert!(store.load().is_err());
            assert!(store.prepare(&[intent(target("pump"), fw())]).is_err());
            assert_eq!(fs::read_to_string(store.path()).unwrap(), raw);
            assert!(!store.path().with_extension("json.v1.bak").exists());
            fs::remove_dir_all(dir).unwrap();
        }
    }
    #[test]
    fn rejected_batch_never_persists_partial_profiles() {
        let (dir, store) = temp();
        assert!(
            store
                .prepare(&[
                    intent(target("pump"), fw()),
                    intent(
                        ProfileTarget::Host {
                            channel_id: ChannelId::new("case")
                        },
                        fw()
                    ),
                ])
                .is_err()
        );
        assert!(store.load().unwrap().profiles.is_empty());
        assert!(!store.path().exists());
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn durable_prepare_then_binding_only_after_ack_and_failed_save_blocks() {
        let (dir, store) = temp();
        assert!(!dir.exists());
        let (library, proposals) = store.prepare(&[intent(target("pump"), fw())]).unwrap();
        assert_eq!(library.proposals, proposals);
        assert!(library.bindings.is_empty());
        assert!(store.load().unwrap().bindings.is_empty());
        let now = store.bind_success(&proposals).unwrap();
        assert_eq!(now.bindings, proposals);
        assert!(now.proposals.is_empty());
        let bytes = fs::read(store.path()).unwrap();
        assert_eq!(
            serde_json::from_slice::<serde_json::Value>(&bytes).unwrap()["version"],
            2
        );
        fs::remove_dir_all(dir).unwrap();
        let (dir, store) = temp();
        fs::create_dir_all(store.path()).unwrap(); // an invalid file path prevents prepare
        assert!(store.prepare(&[intent(target("pump"), fw())]).is_err());
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn concurrent_transactions_use_latest_snapshot() {
        let (dir, store) = temp();
        let barrier = Arc::new(Barrier::new(4));
        let handles: Vec<_> = (0..4)
            .map(|i| {
                let (store, barrier) = (store.clone(), Arc::clone(&barrier));
                thread::spawn(move || {
                    barrier.wait();
                    let (_, p) = store
                        .prepare(&[intent(target(&format!("pump-{i}")), fw())])
                        .unwrap();
                    store.bind_success(&p).unwrap();
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
        let state = store.load().unwrap();
        assert_eq!(state.profiles.len(), 4);
        assert_eq!(state.bindings.len(), 4);
        assert_eq!(state.next_index, 5);
        let barrier = Arc::new(Barrier::new(2));
        let ids: Vec<_> = state.profiles.iter().take(2).map(|p| p.id).collect();
        let handles: Vec<_> = ids
            .into_iter()
            .enumerate()
            .map(|(i, id)| {
                let (store, barrier) = (store.clone(), Arc::clone(&barrier));
                thread::spawn(move || {
                    barrier.wait();
                    store.rename(id, &format!("new-{i}")).unwrap();
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
        assert_eq!(
            store
                .load()
                .unwrap()
                .profiles
                .iter()
                .filter(|p| p.name.starts_with("new-"))
                .count(),
            2
        );
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn demo_uses_separate_filename_and_never_imports_defaults() {
        let (dir, store) = temp();
        let mut demo = store.clone();
        demo.path.set_file_name("demo-profiles.json");
        demo.demo = true;
        demo = demo.with_legacy_defaults(vec![DefaultProfile {
            device_id: "x".into(),
            channel_id: "y".into(),
            profile: "unknown".into(),
        }]);
        assert!(demo.load().unwrap().bindings.is_empty());
        demo.prepare(&[intent(target("pump"), fw())]).unwrap();
        assert!(!store.path().exists());
        assert_eq!(demo.load().unwrap().profiles.len(), 1);
        fs::remove_dir_all(dir).unwrap();
    }
}
