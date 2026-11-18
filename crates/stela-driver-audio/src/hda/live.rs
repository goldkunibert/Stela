// SPDX-License-Identifier: GPL-2.0-only
//! Refilling a running analog playback ring. [`Controller::write_pcm`] needs a
//! stopped stream, and the HDMI owner has its own completed-period refill;
//! a bridge that plays a guest's sound keeps one cyclic ring ahead of the
//! controller's read position instead. The controller reads the ring on its
//! own clock, so a write is accepted only where the hardware's link position
//! (LPIB) shows it cannot be reading, by a margin, before and after the copy.
use super::*;
use crate::Direction;
use core::sync::atomic::{Ordering, fence};

/// Bytes ahead of the read position a write must start: the controller's
/// FIFO prefetches ahead of LPIB, so the ring is not safe to change right in
/// front of it. 64 frames of 16-bit stereo, beside the position policy's own
/// FIFO depth.
const GUARD_BYTES: usize = 256;

impl<R: Registers, T: Timer> Controller<R, T> {
    /// Writes `pcm` at `offset` of the running analog playback stream `id`'s
    /// ring. `offset` and `pcm.len()` are whole 32-bit words, and the range
    /// lies inside the ring. The range must also lie wholly ahead of the
    /// read position: it starts at least a guard (the FIFO's depth and 64
    /// frames) in front of LPIB and ends before the ring wraps back to it.
    /// Otherwise [`Error::PlaybackUnderrun`], with the ring unchanged. The
    /// position is read again after the copy; if the controller reached the
    /// range meanwhile the same error is returned, though the bytes are
    /// written: a caller that gets it has fallen behind the controller and
    /// should refill from the position on.
    pub fn refill_playback(
        &mut self,
        id: StreamId,
        offset: usize,
        pcm: &[u8],
    ) -> Result<(), Error> {
        self.minimum_guard()?;
        if self.hdmi_stream_port(id).is_some() {
            return Err(Error::InvalidStream);
        }
        let (running, ring, playback) = {
            let stream = self.stream(id)?;
            (
                stream.running,
                stream.buffer.len(),
                stream.direction == Direction::Playback,
            )
        };
        if !running {
            return Err(Error::StreamStopped);
        }
        if !playback
            || pcm.is_empty()
            || !offset.is_multiple_of(4)
            || !pcm.len().is_multiple_of(4)
            || offset.checked_add(pcm.len()).is_none_or(|end| end > ring)
        {
            return Err(Error::InvalidBuffer);
        }
        let guard = GUARD_BYTES + self.position_info(id)?.fifo_delay_bytes as usize;
        let before = self.position(id)? as usize;
        let ahead = (offset + ring - before) % ring;
        if ahead < guard || ahead + pcm.len() > ring {
            return Err(Error::PlaybackUnderrun);
        }
        let index = usize::from(id.descriptor_index());
        for (word, bytes) in pcm.as_chunks::<4>().0.iter().enumerate() {
            // Checked aligned volatile words, not a mutable slice over RAM
            // the controller is reading.
            self.streams[index]
                .as_mut()
                .ok_or(Error::InvalidStream)?
                .buffer
                .write(offset + word * 4, u32::from_le_bytes(*bytes).to_le());
        }
        fence(Ordering::SeqCst);
        let after = self.position(id)? as usize;
        let moved = (after + ring - before) % ring;
        if moved > ahead - guard {
            return Err(Error::PlaybackUnderrun);
        }
        Ok(())
    }
}
