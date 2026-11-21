//! Cursor images as PNG, sent once per viewer and then referred to by ID.

use crate::hub::{Client, Hub, Out};
use bytes::Bytes;

impl Hub {
    /// Replays the current cursor to a newly connected client and remembers its ID.
    pub fn send_current_cursor(&self, client: &Client) {
        if let Some((id, msg)) = self.cursor.lock().clone() {
            let mut known = client.lb.known_cursors.lock();
            if !known.contains(&id) {
                if known.len() >= 64 {
                    known.pop_front();
                }
                known.push_back(id);
            }
            client.send(Out::Bin(msg));
        }
    }

    /// Publishes a cursor image to all clients: PNG-compressed on first send per client
    /// (flags=0x01), or a 6-byte cache hit (flags=0x02) when the client already has it.
    pub fn publish_cursor(&self, w: u16, h: u16, hx: u16, hy: u16, rgba: &[u8]) {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        w.hash(&mut hasher);
        h.hash(&mut hasher);
        hx.hash(&mut hasher);
        hy.hash(&mut hasher);
        rgba.hash(&mut hasher);
        let h64 = hasher.finish();
        let id = ((h64 ^ (h64 >> 32)) as u32).max(1);

        // Encode cursor RGBA as PNG (typically ~200-400 B vs 4-16 KB raw RGBA).
        let png_bytes = if w > 0 && h > 0 && rgba.len() == w as usize * h as usize * 4 {
            let mut out = Vec::new();
            let mut enc = png::Encoder::new(&mut out, w as u32, h as u32);
            enc.set_color(png::ColorType::Rgba);
            enc.set_depth(png::BitDepth::Eight);
            enc.set_compression(png::Compression::Fast);
            match enc
                .write_header()
                .and_then(|mut wr| wr.write_image_data(rgba))
            {
                Ok(()) if out.len() < rgba.len() => Some(out),
                _ => None,
            }
        } else {
            None
        };

        let (flags, payload): (u8, &[u8]) = match &png_bytes {
            Some(p) => (0x01, p.as_slice()),
            None => (0x00, rgba),
        };
        let mut full = Vec::with_capacity(14 + payload.len());
        full.push(0x03);
        full.push(flags);
        full.extend_from_slice(&w.to_be_bytes());
        full.extend_from_slice(&h.to_be_bytes());
        full.extend_from_slice(&hx.to_be_bytes());
        full.extend_from_slice(&hy.to_be_bytes());
        full.extend_from_slice(&id.to_be_bytes());
        full.extend_from_slice(payload);
        let full = Bytes::from(full);

        let mut hit = [0u8; 6];
        hit[0] = 0x03;
        hit[1] = 0x02;
        hit[2..6].copy_from_slice(&id.to_be_bytes());
        let hit = Bytes::copy_from_slice(&hit);

        *self.cursor.lock() = Some((id, full.clone()));
        for c in self.clients.lock().iter() {
            let mut known = c.lb.known_cursors.lock();
            if known.contains(&id) {
                drop(known);
                c.send(Out::Bin(hit.clone()));
            } else {
                if known.len() >= 64 {
                    known.pop_front();
                }
                known.push_back(id);
                drop(known);
                c.send(Out::Bin(full.clone()));
            }
        }
    }
}
