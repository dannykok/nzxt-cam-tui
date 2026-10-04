//! Minimal fixed-version handshake messages.
//!
//! The TUI and service are shipped together, so protocol v6 uses an exact
//! version match rather than version-list negotiation. Rejections contain only
//! a finite code and never carry peer-controlled diagnostic text.

use serde::{Deserialize, Serialize};

/// The fixed protocol version spoken by this release.
pub const PROTOCOL_VERSION_V6: u16 = 6;

/// The first frame sent by a client.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ClientHello {
    pub protocol_version: u16,
}

impl ClientHello {
    /// Constructs the hello used by protocol-v6 clients.
    #[must_use]
    pub const fn v6() -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION_V6,
        }
    }

    /// Returns whether this hello is the exact version supported by the
    /// service.
    #[must_use]
    pub const fn is_supported(self) -> bool {
        self.protocol_version == PROTOCOL_VERSION_V6
    }
}

/// Finite rejection reasons sent during the handshake or while no operation
/// protocol exists.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RejectionCode {
    ServiceBusy,
    UnsupportedVersion,
    InvalidMessage,
    UnexpectedMessage,
}

/// The service's response to a connection.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(tag = "status", rename_all = "snake_case", deny_unknown_fields)]
pub enum ServerResponse {
    Ready { protocol_version: u16 },
    Rejected { code: RejectionCode },
}

impl ServerResponse {
    /// Constructs the only successful response supported by this release.
    #[must_use]
    pub const fn ready_v6() -> Self {
        Self::Ready {
            protocol_version: PROTOCOL_VERSION_V6,
        }
    }

    #[must_use]
    pub const fn rejected(code: RejectionCode) -> Self {
        Self::Rejected { code }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_v6_messages_have_compact_round_trips() {
        let hello = ClientHello::v6();
        let hello_json = serde_json::to_string(&hello).unwrap();
        assert_eq!(hello_json, r#"{"protocol_version":6}"#);
        assert_eq!(
            serde_json::from_str::<ClientHello>(&hello_json).unwrap(),
            hello
        );

        assert_eq!(
            serde_json::to_string(&ServerResponse::ready_v6()).unwrap(),
            r#"{"status":"ready","protocol_version":6}"#
        );
        for response in [
            ServerResponse::ready_v6(),
            ServerResponse::rejected(RejectionCode::ServiceBusy),
            ServerResponse::rejected(RejectionCode::UnsupportedVersion),
            ServerResponse::rejected(RejectionCode::InvalidMessage),
            ServerResponse::rejected(RejectionCode::UnexpectedMessage),
        ] {
            let json = serde_json::to_string(&response).unwrap();
            assert_eq!(
                serde_json::from_str::<ServerResponse>(&json).unwrap(),
                response
            );
        }
    }

    #[test]
    fn only_the_exact_v6_number_is_supported_at_u16_boundaries() {
        assert!(ClientHello::v6().is_supported());
        for protocol_version in [0, 1, 2, 3, 4, 5, PROTOCOL_VERSION_V6 + 1, u16::MAX] {
            assert!(!ClientHello { protocol_version }.is_supported());
        }
    }

    #[test]
    fn unknown_fields_and_rejection_codes_are_rejected() {
        assert!(
            serde_json::from_str::<ClientHello>(r#"{"protocol_version":6,"name":"peer"}"#).is_err()
        );
        assert!(
            serde_json::from_str::<ServerResponse>(
                r#"{"status":"ready","protocol_version":6,"instance":"peer"}"#
            )
            .is_err()
        );
        assert!(
            serde_json::from_str::<ServerResponse>(
                r#"{"status":"rejected","code":"peer_diagnostic"}"#
            )
            .is_err()
        );
    }
}
