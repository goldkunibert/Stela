//! The pump that carries the sound of the guest's USB speaker to a cyclic
//! playback ring on the host's audio controller.
//!
//! The guest sends 48 kHz stereo at its own clock, the controller reads its
//! ring at the codec's, and neither is told about the other: the guest's
//! queue grows or drains by the clocks' difference. The pump keeps the
//! host's ring a fixed distance ahead of the controller's read position
//! (`target_fill`), takes the guest's frames to fill it, pads with silence
//! when the guest has sent fewer (so the controller never replays old
//! audio), and drops the guest's oldest frames when its queue has grown past
//! `max_queue`, which bounds the delay the drift can add.
//!
//! The pump holds no clock: how far the controller has read is the change in
//! its position since the last [`Pump::service`]. That is exact only while a
//! ring is not read through twice between two services, so the caller
//! services at least every `ring_frames / 4` of audio (10 ms of an 80 ms
//! ring); a longer stall is counted as an underrun and the stream resumes
//! from the controller's position.
//!
//! Owning the controller, its route and its timing is the caller's, through
//! [`Sink`]; the guest's speaker is its [`Source`].
use alloc::vec::Vec;

/// 16-bit stereo, the format of the host ring.
pub const FRAME_BYTES: usize = 4;
/// Frames converted per step, which bounds the pump's stack use.
const CHUNK: usize = 256;

/// The guest's speaker, as the engine's machine presents it.
pub trait Source {
    /// Whether the guest has selected a streaming setting.
    fn streaming(&self) -> bool;
    /// Frames waiting.
    fn queued(&self) -> usize;
    /// Moves up to `out.len()` frames, 32-bit MSB-aligned with the guest's
    /// mute and volume applied; the number moved.
    fn take(&mut self, out: &mut [[i32; 2]]) -> usize;
}

/// Why a [`Sink`] refused a write.
#[derive(Debug, PartialEq, Eq)]
pub enum Refusal<E> {
    /// The range is not ahead of the controller's read position by enough:
    /// the pump has fallen behind it.
    TooClose,
    Sink(E),
}

/// A running cyclic playback ring of 16-bit stereo frames.
pub trait Sink {
    type Error;
    /// The ring's frames.
    fn ring_frames(&self) -> usize;
    /// The frames at the read position's front that a write must keep clear.
    fn min_ahead_frames(&self) -> usize;
    /// The controller's read position, in frames from the ring's start.
    fn position_frames(&mut self) -> Result<usize, Self::Error>;
    /// Writes `prime` at the ring's start while stopped, then starts the
    /// stream from there.
    fn start(&mut self, prime: &[u8]) -> Result<(), Self::Error>;
    /// Writes `pcm` at frame `frame_offset` while running. The range never
    /// wraps past the ring's end. The ring is silent where the pump has not
    /// written, from the start: `start` is given a ring it has cleared.
    fn write(&mut self, frame_offset: usize, pcm: &[u8]) -> Result<(), Refusal<Self::Error>>;
    /// Stops the stream and leaves its ring silent.
    fn stop(&mut self) -> Result<(), Self::Error>;
}

/// The pump's tuning, in frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// How far ahead of the read position the host ring is kept: 20 ms.
    pub target_fill: usize,
    /// The guest's queue beyond which its oldest frames are dropped: 50 ms.
    pub max_queue: usize,
    /// What a drop leaves queued: 10 ms.
    pub target_queue: usize,
    /// Silence written, with the guest not streaming and nothing queued,
    /// before the stream stops: 200 ms.
    pub stop_after: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            target_fill: 960,
            max_queue: 2400,
            target_queue: 480,
            stop_after: 9600,
        }
    }
}

/// What the pump counted, for the engine's report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Frames the controller has read since the stream last started.
    pub played: u64,
    /// Frames of silence written where the guest had sent none.
    pub silence: u64,
    /// The guest's frames dropped because its queue grew past the limit.
    pub dropped: u64,
    /// Times the controller read past what the pump had written.
    pub underruns: u32,
    /// Times the stream started.
    pub starts: u32,
}

#[derive(Debug, PartialEq, Eq)]
enum State {
    Stopped,
    Running {
        /// Frames read and written since the start; the ring offset of the
        /// write position is `written % ring`.
        played: u64,
        written: u64,
        position: usize,
        /// Silence written in a row while the guest was not streaming.
        quiet: usize,
    },
}

pub struct Pump {
    config: Config,
    state: State,
    stats: Stats,
}

/// Converts one 32-bit MSB-aligned sample to 16-bit little-endian.
fn sample(value: i32) -> [u8; 2] {
    ((value >> 16) as i16).to_le_bytes()
}

impl Pump {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            state: State::Stopped,
            stats: Stats::default(),
        }
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    pub fn running(&self) -> bool {
        matches!(self.state, State::Running { .. })
    }

    /// Frames of `frames` taken from `source` as 16-bit PCM into `out`,
    /// padded with silence, and how many of them were silence.
    fn convert<S: Source>(source: &mut S, frames: usize, out: &mut Vec<u8>) -> usize {
        let mut taken = [[0i32; 2]; CHUNK];
        let mut silence = 0;
        let mut left = frames;
        while left > 0 {
            let n = left.min(CHUNK);
            let got = source.take(&mut taken[..n]);
            for frame in &taken[..got] {
                out.extend_from_slice(&sample(frame[0]));
                out.extend_from_slice(&sample(frame[1]));
            }
            out.resize(out.len() + (n - got) * FRAME_BYTES, 0);
            silence += n - got;
            left -= n;
        }
        silence
    }

    /// Serves the stream once: starts it, tops its ring up, or stops it.
    pub fn service<S: Source, K: Sink>(
        &mut self,
        source: &mut S,
        sink: &mut K,
    ) -> Result<(), K::Error> {
        match self.state {
            State::Stopped => self.start(source, sink),
            State::Running { .. } => self.top_up(source, sink),
        }
    }

    fn start<S: Source, K: Sink>(&mut self, source: &mut S, sink: &mut K) -> Result<(), K::Error> {
        let frames = self.config.target_fill.min(sink.ring_frames());
        // Waits for a full prime, so the start has no gap of silence in it.
        if !source.streaming() || source.queued() < frames {
            return Ok(());
        }
        let mut prime = Vec::with_capacity(frames * FRAME_BYTES);
        self.stats.silence += Self::convert(source, frames, &mut prime) as u64;
        sink.start(&prime)?;
        // A controller that does not start reading at the ring's first
        // frame is followed from where it is: the frames before it count as
        // read, and a position past the prime is an underrun at the first
        // service.
        let ring = sink.ring_frames();
        let position = sink.position_frames()? % ring;
        self.stats.starts += 1;
        self.stats.played = position as u64;
        self.state = State::Running {
            played: position as u64,
            written: frames as u64,
            position,
            quiet: 0,
        };
        Ok(())
    }

    fn top_up<S: Source, K: Sink>(&mut self, source: &mut S, sink: &mut K) -> Result<(), K::Error> {
        let State::Running {
            mut played,
            mut written,
            mut position,
            mut quiet,
        } = self.state
        else {
            return Ok(());
        };
        let ring = sink.ring_frames();
        let ahead = sink.min_ahead_frames();
        let now = sink.position_frames()? % ring;
        let moved = (now + ring - position) % ring;
        played += moved as u64;
        // The frames the controller has passed are overwritten with silence,
        // so a ring it reads through again, the pump behind it, plays
        // nothing old. A write refused as too close is not worth a retry:
        // the frames are behind the controller.
        let mut scrub = position;
        let mut left = moved;
        while left > 0 {
            let frames = left.min(ring - scrub);
            match sink.write(scrub, &alloc::vec![0; frames * FRAME_BYTES]) {
                Ok(()) | Err(Refusal::TooClose) => {}
                Err(Refusal::Sink(error)) => return Err(error),
            }
            scrub = (scrub + frames) % ring;
            left -= frames;
        }
        position = now;
        // The controller read past what was written, or into the frames a
        // write must keep clear: write on from where it can still be
        // reached, and the frames between are what the ring held.
        if written < played + ahead as u64 {
            self.stats.underruns += 1;
            written = played + ahead as u64;
        }
        let fill = (written - played) as usize;

        // A queue the drift has grown too long loses its oldest frames.
        let queued = source.queued();
        if queued > self.config.max_queue {
            let mut waste = [[0i32; 2]; CHUNK];
            let mut drop = queued - self.config.target_queue;
            while drop > 0 {
                let got = source.take(&mut waste[..drop.min(CHUNK)]);
                if got == 0 {
                    break;
                }
                self.stats.dropped += got as u64;
                drop -= got;
            }
        }

        let want = self
            .config
            .target_fill
            .saturating_sub(fill)
            .min(ring.saturating_sub(fill + ahead));
        let mut refused = false;
        let mut left = want;
        while left > 0 && !refused {
            // Never across the ring's end, which the sink does not wrap.
            let offset = (written % ring as u64) as usize;
            let frames = left.min(ring - offset);
            let mut pcm = Vec::with_capacity(frames * FRAME_BYTES);
            let silence = Self::convert(source, frames, &mut pcm);
            match sink.write(offset, &pcm) {
                Ok(()) => {
                    written += frames as u64;
                    left -= frames;
                    self.stats.silence += silence as u64;
                    if source.streaming() || silence < frames {
                        quiet = 0;
                    } else {
                        quiet += frames;
                    }
                }
                Err(Refusal::TooClose) => {
                    // Fallen behind during the copy: the next service finds
                    // the controller past what was written, counts the
                    // underrun and starts from its position.
                    written = played;
                    refused = true;
                }
                Err(Refusal::Sink(error)) => return Err(error),
            }
        }
        self.stats.played = played;
        if quiet >= self.config.stop_after && source.queued() == 0 {
            sink.stop()?;
            self.state = State::Stopped;
            return Ok(());
        }
        self.state = State::Running {
            played,
            written,
            position,
            quiet,
        };
        Ok(())
    }
}

#[cfg(test)]
mod tests;
