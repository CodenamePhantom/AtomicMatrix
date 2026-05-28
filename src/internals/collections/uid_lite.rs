use std::fs::File;
use std::io::Read;

pub fn generate_uuid() -> String {
    let mut bytes = [0u8; 16];
    File::open("/dev/urandom")
        .unwrap()
        .read_exact(&mut bytes)
        .unwrap();

    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;

    let uuid = uuid_to_string(bytes);

    uuid
}

fn uuid_to_string(uuid: [u8; 16]) -> String {
    format!(
        "{:02x}{:02x}{:02x}{:02x}-\
        {:02x}{:02x}-\
        {:02x}{:02x}-\
        {:02x}{:02x}-\
        {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        uuid[0],uuid[1],uuid[2],uuid[3],
        uuid[4],uuid[5],
        uuid[6],uuid[7],
        uuid[8],uuid[9],
        uuid[10],uuid[11],uuid[12],uuid[13],uuid[14],uuid[15],
    )
}
