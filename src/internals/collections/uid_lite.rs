//! Lite UID generator to create unique names and IDs.
//!
//! It makes use of the /dev/urandom file to get a entropic sequence of 16 u8 and convert them to a
//! string.
//!
//! This implementation **DO NOT GUARANTEE** monotonic epoch UID generation. If your use case
//! requires that all UUIDs follow a contiguous sequence of reclamation stamps, consider using more
//! robust implementations of UUID.

use std::fs::File;
use std::io::Read;

/// Generate a new random UUID as a string.
///
/// It extract 16 numbers from /dev/urandom, applies the version 4 mask, the RFC 4122 mask at byte
/// 8, runs the bytes into a decoder to generate a readable String, and returns it to the caller.
///
/// ### Returns:
/// The UUID decoded string.
pub fn generate_uuid() -> String {
    let mut bytes = [0u8; 16];
    File::open("/dev/urandom").unwrap().read_exact(&mut bytes).unwrap();

    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;

    let uuid = uuid_to_string(bytes);

    uuid
}

/// Deoces the bytes into ASCII readable characters and formats it into a String that can be used.
///
/// ### Params:
/// @uuid: The 16 bytes array containing the raw UUID.
///
/// ### Returns:
/// The decoded byte string.
fn uuid_to_string(uuid: [u8; 16]) -> String {
    format!(
        "{:02x}{:02x}{:02x}{:02x}-\
        {:02x}{:02x}-\
        {:02x}{:02x}-\
        {:02x}{:02x}-\
        {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        uuid[0],
        uuid[1],
        uuid[2],
        uuid[3],
        uuid[4],
        uuid[5],
        uuid[6],
        uuid[7],
        uuid[8],
        uuid[9],
        uuid[10],
        uuid[11],
        uuid[12],
        uuid[13],
        uuid[14],
        uuid[15]
    )
}
