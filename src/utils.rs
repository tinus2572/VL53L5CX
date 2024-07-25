pub(crate) fn from_u8_to_i16(src: &[u8], dst: &mut[i16]) {
    for (i, chunk) in src.chunks(2).enumerate() {
        dst[i] = ((chunk[0] as u16) | (chunk[1] as u16) << 8) as i16;
    }
}

pub(crate) fn from_u8_to_i32(src: &[u8], dst: &mut[i32]) {
    for (i, chunk) in src.chunks(4).enumerate() {
        dst[i] = ((chunk[0] as u32) | (chunk[1] as u32) << 8 | (chunk[2] as u32) << 16 | (chunk[3] as u32) << 24) as i32;
    }
}

pub(crate) fn from_u8_to_u16(src: &[u8], dst: &mut[u16]) {
    for (i, chunk) in src.chunks(2).enumerate() {
        dst[i] = (chunk[0] as u16) | (chunk[1] as u16) << 8;
    }
}

pub(crate) fn from_u8_to_u32(src: &[u8], dst: &mut[u32]) {
    for (i, chunk) in src.chunks(4).enumerate() {
        dst[i] = (chunk[0] as u32) | (chunk[1] as u32) << 8 | (chunk[2] as u32) << 16 | (chunk[3] as u32) << 24;
    }
}

pub(crate) fn from_i16_to_u8(src: &[i16], dst: &mut[u8]) {
    for (i, &num) in src.iter().enumerate() {
        dst[i*2..(i+1)*2].copy_from_slice(&num.to_le_bytes()); 
    }
}

pub(crate) fn from_i32_to_u8(src: &[i32], dst: &mut[u8]) {
    for (i, &num) in src.iter().enumerate() {
        dst[i*4..(i+1)*4].copy_from_slice(&num.to_le_bytes()); 
    }
}

#[allow(dead_code)]
pub(crate) fn from_u16_to_u8(src: &[u16], dst: &mut[u8]) {
    for (i, &num) in src.iter().enumerate() {
        dst[i*2..(i+1)*2].copy_from_slice(&num.to_le_bytes()); 
    }
}

pub(crate) fn from_u32_to_u8(src: &[u32], dst: &mut[u8]) {
    for (i, &num) in src.iter().enumerate() {
        dst[i*4..(i+1)*4].copy_from_slice(&num.to_le_bytes()); 
    }
}

pub(crate) fn swap_buffer(buffer: &mut [u8], size: usize) {
    for chunk in buffer[..size].chunks_exact_mut(4) {
        let tmp: u32 = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        chunk.copy_from_slice(&tmp.to_le_bytes());
    }
}
