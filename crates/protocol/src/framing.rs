use std::{fmt, io, io::Write};

use bytes::{Bytes, BytesMut};
use serde::{Serialize, de::DeserializeOwned};
use serde_json::Value;
use tokio_util::codec::{Decoder, Encoder, LengthDelimitedCodec};

/// Maximum JSON payload accepted from or emitted to a peer.
pub const MAX_FRAME_LENGTH: usize = 256 * 1024;

/// A four-byte, big-endian length-delimited JSON codec.
///
/// The length prefix is not included in [`MAX_FRAME_LENGTH`]. Empty payloads
/// are rejected. Decoding first enforces the byte limit and then applies
/// `serde_json`'s parser and nesting limits.
#[derive(Debug)]
pub struct JsonFrameCodec {
    inner: LengthDelimitedCodec,
}

impl JsonFrameCodec {
    #[must_use]
    pub fn new() -> Self {
        let inner = LengthDelimitedCodec::builder()
            .big_endian()
            .length_field_length(4)
            .max_frame_length(MAX_FRAME_LENGTH)
            .new_codec();
        Self { inner }
    }
}

impl Default for JsonFrameCodec {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder for JsonFrameCodec {
    type Item = Value;
    type Error = FrameError;

    fn decode(&mut self, source: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let Some(payload) = self.inner.decode(source).map_err(FrameError::Io)? else {
            return Ok(None);
        };
        if payload.is_empty() {
            return Err(FrameError::EmptyPayload);
        }

        serde_json::from_slice(&payload)
            .map(Some)
            .map_err(FrameError::Json)
    }
}

impl<T> Encoder<T> for JsonFrameCodec
where
    T: Serialize,
{
    type Error = FrameError;

    fn encode(&mut self, item: T, destination: &mut BytesMut) -> Result<(), Self::Error> {
        let payload = serialize_bounded(&item)?;
        self.inner
            .encode(Bytes::from(payload), destination)
            .map_err(FrameError::Io)
    }
}

/// Verifies that a serializable value fits in one protocol frame.
///
/// Validation uses the codec's bounded writer, so an oversized value never
/// grows an intermediate allocation beyond [`MAX_FRAME_LENGTH`].
pub fn validate_frame_payload<T>(value: &T) -> Result<(), FrameError>
where
    T: Serialize,
{
    serialize_bounded(value).map(drop)
}

fn serialize_bounded<T>(value: &T) -> Result<Vec<u8>, FrameError>
where
    T: Serialize,
{
    let mut payload = BoundedBuffer::new(MAX_FRAME_LENGTH);
    if let Err(error) = serde_json::to_writer(&mut payload, value) {
        return if payload.limit_exceeded {
            Err(FrameError::PayloadTooLarge {
                maximum: MAX_FRAME_LENGTH,
            })
        } else {
            Err(FrameError::Json(error))
        };
    }
    if payload.bytes.is_empty() {
        return Err(FrameError::EmptyPayload);
    }
    Ok(payload.bytes)
}

#[derive(Debug)]
struct BoundedBuffer {
    bytes: Vec<u8>,
    maximum: usize,
    limit_exceeded: bool,
}

impl BoundedBuffer {
    fn new(maximum: usize) -> Self {
        Self {
            bytes: Vec::new(),
            maximum,
            limit_exceeded: false,
        }
    }
}

impl Write for BoundedBuffer {
    fn write(&mut self, buffer: &[u8]) -> io::Result<usize> {
        if buffer.len() > self.maximum.saturating_sub(self.bytes.len()) {
            self.limit_exceeded = true;
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "serialized frame exceeds its configured byte limit",
            ));
        }

        self.bytes.extend_from_slice(buffer);
        Ok(buffer.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Converts a validated JSON frame into a concrete protocol message.
pub fn from_value<T>(value: Value) -> Result<T, FrameError>
where
    T: DeserializeOwned,
{
    serde_json::from_value(value).map_err(FrameError::Json)
}

#[derive(Debug)]
pub enum FrameError {
    Io(io::Error),
    Json(serde_json::Error),
    EmptyPayload,
    PayloadTooLarge { maximum: usize },
}

impl fmt::Display for FrameError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "frame I/O failed: {error}"),
            Self::Json(error) => write!(formatter, "frame contains invalid JSON: {error}"),
            Self::EmptyPayload => formatter.write_str("frame payload must not be empty"),
            Self::PayloadTooLarge { maximum } => {
                write!(formatter, "frame payload exceeds the {maximum}-byte limit")
            }
        }
    }
}

impl std::error::Error for FrameError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Json(error) => Some(error),
            Self::EmptyPayload | Self::PayloadTooLarge { .. } => None,
        }
    }
}

impl From<io::Error> for FrameError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

#[cfg(test)]
mod tests {
    use bytes::BufMut;
    use serde::{Deserialize, Serialize};

    use super::*;

    #[derive(Debug, Deserialize, PartialEq, Serialize)]
    struct TestMessage {
        name: String,
        sequence: u64,
    }

    fn message() -> TestMessage {
        TestMessage {
            name: "telemetry".into(),
            sequence: 42,
        }
    }

    #[test]
    fn codec_uses_a_four_byte_big_endian_length_and_round_trips_json() {
        let mut codec = JsonFrameCodec::new();
        let mut encoded = BytesMut::new();

        codec.encode(message(), &mut encoded).unwrap();

        let declared_length = u32::from_be_bytes(encoded[..4].try_into().unwrap()) as usize;
        assert_eq!(declared_length, encoded.len() - 4);
        let decoded = codec.decode(&mut encoded).unwrap().unwrap();
        assert_eq!(from_value::<TestMessage>(decoded).unwrap(), message());
        assert!(encoded.is_empty());
    }

    #[test]
    fn fragmented_and_combined_frames_decode_without_losing_boundaries() {
        let mut encoder = JsonFrameCodec::new();
        let mut first = BytesMut::new();
        let mut second = BytesMut::new();
        encoder.encode(message(), &mut first).unwrap();
        encoder
            .encode(
                TestMessage {
                    name: "event".into(),
                    sequence: 43,
                },
                &mut second,
            )
            .unwrap();

        let split_at = first.len() - 1;
        let mut input = first.split_to(split_at);
        let mut decoder = JsonFrameCodec::new();
        assert!(decoder.decode(&mut input).unwrap().is_none());

        input.extend_from_slice(&first);
        input.extend_from_slice(&second);
        let one = from_value::<TestMessage>(decoder.decode(&mut input).unwrap().unwrap()).unwrap();
        let two = from_value::<TestMessage>(decoder.decode(&mut input).unwrap().unwrap()).unwrap();
        assert_eq!(one.sequence, 42);
        assert_eq!(two.sequence, 43);
        assert!(decoder.decode(&mut input).unwrap().is_none());
    }

    #[test]
    fn zero_length_and_invalid_json_are_rejected() {
        let mut decoder = JsonFrameCodec::new();
        let mut empty = BytesMut::from(&0_u32.to_be_bytes()[..]);
        assert!(matches!(
            decoder.decode(&mut empty),
            Err(FrameError::EmptyPayload)
        ));

        let mut invalid = BytesMut::new();
        invalid.put_u32(1);
        invalid.extend_from_slice(b"{");
        assert!(matches!(
            JsonFrameCodec::new().decode(&mut invalid),
            Err(FrameError::Json(_))
        ));
    }

    #[test]
    fn oversized_declared_length_is_rejected_before_receiving_a_payload() {
        let mut input = BytesMut::new();
        input.put_u32(u32::try_from(MAX_FRAME_LENGTH + 1).unwrap());

        let error = JsonFrameCodec::new().decode(&mut input).unwrap_err();
        assert!(matches!(error, FrameError::Io(_)));
    }

    #[test]
    fn oversized_encoded_payload_is_rejected_without_writing_a_partial_frame() {
        let oversized = "x".repeat(MAX_FRAME_LENGTH);
        let mut destination = BytesMut::new();

        let error = JsonFrameCodec::new()
            .encode(&oversized, &mut destination)
            .unwrap_err();

        assert!(matches!(error, FrameError::PayloadTooLarge { .. }));
        assert!(matches!(
            validate_frame_payload(&oversized),
            Err(FrameError::PayloadTooLarge { .. })
        ));
        assert!(destination.is_empty());
    }

    #[test]
    fn encoder_buffer_never_retains_bytes_beyond_its_limit() {
        let mut buffer = BoundedBuffer::new(4);
        buffer.write_all(b"1234").unwrap();

        assert!(buffer.write_all(b"5").is_err());
        assert_eq!(buffer.bytes, b"1234");
        assert!(buffer.limit_exceeded);
    }

    #[test]
    fn deeply_nested_json_is_rejected_by_the_parser_limit() {
        let nesting = 200;
        let json = format!("{}null{}", "[".repeat(nesting), "]".repeat(nesting));
        let mut input = BytesMut::new();
        input.put_u32(u32::try_from(json.len()).unwrap());
        input.extend_from_slice(json.as_bytes());

        assert!(matches!(
            JsonFrameCodec::new().decode(&mut input),
            Err(FrameError::Json(_))
        ));
    }
}
