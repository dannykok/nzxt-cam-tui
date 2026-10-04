use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Reading {
    pub label: String,
    pub value: f64,
    pub unit: String,
    pub kind: ReadingKind,
}

impl Reading {
    pub fn new(
        label: impl Into<String>,
        value: f64,
        unit: impl Into<String>,
        kind: ReadingKind,
    ) -> Self {
        Self {
            label: label.into(),
            value,
            unit: unit.into(),
            kind,
        }
    }

    pub fn formatted_value(&self) -> String {
        match self.kind {
            ReadingKind::Temperature => format!("{:.1}", self.value),
            ReadingKind::Duty | ReadingKind::ChannelCount => format!("{:.0}", self.value),
            ReadingKind::Speed => format!("{:.0}", self.value),
            ReadingKind::Mode => format_mode(self.value),
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ReadingKind {
    Temperature,
    Speed,
    Duty,
    ChannelCount,
    Mode,
}

fn format_mode(value: f64) -> String {
    if !value.is_finite() || value.fract() != 0.0 {
        return "UNKNOWN".into();
    }
    match value {
        0.0 => "FULL SPEED".into(),
        1.0 => "MANUAL".into(),
        2.0 => "AUTO".into(),
        value => format!("MODE {value:.0}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn readings_keep_their_existing_precision() {
        assert_eq!(
            Reading::new("Liquid", 31.54, "°C", ReadingKind::Temperature).formatted_value(),
            "31.5"
        );
        assert_eq!(
            Reading::new("Pump", 2261.6, "rpm", ReadingKind::Speed).formatted_value(),
            "2262"
        );
        assert_eq!(
            Reading::new("Duty", 49.6, "%", ReadingKind::Duty).formatted_value(),
            "50"
        );
        assert_eq!(
            Reading::new("Channels", 3.0, "", ReadingKind::ChannelCount).formatted_value(),
            "3"
        );
    }

    #[test]
    fn modes_have_conservative_integer_only_formatting() {
        for (value, expected) in [
            (0.0, "FULL SPEED"),
            (1.0, "MANUAL"),
            (2.0, "AUTO"),
            (7.0, "MODE 7"),
            (-1.0, "MODE -1"),
        ] {
            assert_eq!(
                Reading::new("Mode", value, "", ReadingKind::Mode).formatted_value(),
                expected
            );
        }
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1.5] {
            assert_eq!(
                Reading::new("Mode", value, "", ReadingKind::Mode).formatted_value(),
                "UNKNOWN"
            );
        }
    }
}
