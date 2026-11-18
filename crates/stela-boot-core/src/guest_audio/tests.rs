//! The pump against a simulated controller and a simulated guest, each on
//! its own clock. Every guest frame carries its own number (a 32-bit count
//! from 1 in the two channels), so what the controller read shows whether
//! frames came in order, were repeated, or were lost.
use super::*;
use alloc::collections::VecDeque;
use alloc::vec;

const RING: usize = 3840;
const AHEAD: usize = 64;
/// The guest's own ring, which drops its oldest frames when full.
const GUEST_RING: usize = 9600;

fn left(count: u32) -> i32 {
    i32::from(count as u16 as i16) << 16
}
fn right(count: u32) -> i32 {
    i32::from((count >> 16) as u16 as i16) << 16
}

#[derive(Default)]
struct Guest {
    streaming: bool,
    queue: VecDeque<[i32; 2]>,
    next: u32,
    /// Fraction of a frame carried between ticks, in millionths.
    carry: i64,
}

impl Guest {
    fn new() -> Self {
        Self {
            streaming: true,
            next: 1,
            ..Self::default()
        }
    }

    fn send(&mut self, frames: usize) {
        for _ in 0..frames {
            if self.queue.len() == GUEST_RING {
                self.queue.pop_front();
            }
            self.queue.push_back([left(self.next), right(self.next)]);
            self.next += 1;
        }
    }

    /// `frames` at the nominal rate scaled by `1 + ppm / 1e6`.
    fn send_scaled(&mut self, frames: usize, ppm: i64) {
        let exact = frames as i64 * 1_000_000 + frames as i64 * ppm + self.carry;
        self.carry = exact % 1_000_000;
        self.send((exact / 1_000_000) as usize);
    }
}

impl Source for Guest {
    fn streaming(&self) -> bool {
        self.streaming
    }
    fn queued(&self) -> usize {
        self.queue.len()
    }
    fn take(&mut self, out: &mut [[i32; 2]]) -> usize {
        let n = out.len().min(self.queue.len());
        for (slot, frame) in out.iter_mut().zip(self.queue.drain(..n)) {
            *slot = frame;
        }
        n
    }
}

struct Controller {
    ring: Vec<u8>,
    position: usize,
    running: bool,
    /// The frames it has read, as 32-bit counts (0 for silence).
    heard: Vec<u32>,
    starts: u32,
    stops: u32,
    /// Where a start leaves the read position.
    start_at: usize,
    /// Writes to refuse as too close, counted down.
    refuse: u32,
}

impl Controller {
    fn new() -> Self {
        Self {
            ring: vec![0; RING * FRAME_BYTES],
            position: 0,
            running: false,
            heard: Vec::new(),
            starts: 0,
            stops: 0,
            start_at: 0,
            refuse: 0,
        }
    }

    /// Reads `frames` frames from the ring, as the hardware does on its own.
    fn advance(&mut self, frames: usize) {
        if !self.running {
            return;
        }
        for _ in 0..frames {
            let at = self.position * FRAME_BYTES;
            let l = i16::from_le_bytes([self.ring[at], self.ring[at + 1]]) as u16;
            let r = i16::from_le_bytes([self.ring[at + 2], self.ring[at + 3]]) as u16;
            self.heard.push(u32::from(r) << 16 | u32::from(l));
            self.position = (self.position + 1) % RING;
        }
    }

    /// The counts of the non-silent frames heard, in order.
    fn counts(&self) -> Vec<u32> {
        self.heard.iter().copied().filter(|&c| c != 0).collect()
    }
}

impl Sink for Controller {
    type Error = &'static str;
    fn ring_frames(&self) -> usize {
        RING
    }
    fn min_ahead_frames(&self) -> usize {
        AHEAD
    }
    fn position_frames(&mut self) -> Result<usize, Self::Error> {
        Ok(self.position)
    }
    fn start(&mut self, prime: &[u8]) -> Result<(), Self::Error> {
        assert!(!self.running);
        self.ring[..prime.len()].copy_from_slice(prime);
        self.running = true;
        self.position = self.start_at;
        self.starts += 1;
        Ok(())
    }
    fn write(&mut self, frame_offset: usize, pcm: &[u8]) -> Result<(), Refusal<Self::Error>> {
        assert!(self.running);
        let frames = pcm.len() / FRAME_BYTES;
        assert_eq!(pcm.len() % FRAME_BYTES, 0);
        assert!(frame_offset + frames <= RING, "a write wraps the ring");
        let ahead = (frame_offset + RING - self.position) % RING;
        if self.refuse > 0 && pcm.iter().any(|&byte| byte != 0) {
            self.refuse -= 1;
            return Err(Refusal::TooClose);
        }
        // What the driver enforces: clear of the read position by a
        // guard, and ending before the ring wraps back to it.
        if ahead < AHEAD || ahead + frames > RING {
            return Err(Refusal::TooClose);
        }
        self.ring[frame_offset * FRAME_BYTES..][..pcm.len()].copy_from_slice(pcm);
        Ok(())
    }
    fn stop(&mut self) -> Result<(), Self::Error> {
        self.ring.fill(0);
        self.running = false;
        self.stops += 1;
        Ok(())
    }
}

/// 5 ms of audio per tick, the controller reading first, then the guest's
/// packets arriving, then one service.
const TICK: usize = 240;

fn tick(
    pump: &mut Pump,
    guest: &mut Guest,
    controller: &mut Controller,
    ppm: i64,
) -> Result<(), &'static str> {
    controller.advance(TICK);
    guest.send_scaled(TICK, ppm);
    pump.service(guest, controller)
}

fn run(ticks: usize, ppm: i64) -> (Pump, Guest, Controller) {
    let mut pump = Pump::new(Config::default());
    let mut guest = Guest::new();
    let mut controller = Controller::new();
    for _ in 0..ticks {
        tick(&mut pump, &mut guest, &mut controller, ppm).unwrap();
    }
    (pump, guest, controller)
}

fn in_order(counts: &[u32]) {
    assert!(
        counts.windows(2).all(|pair| pair[0] < pair[1]),
        "frames repeated or out of order"
    );
}

#[test]
fn a_matched_clock_plays_every_frame_once_in_order() {
    let (pump, _guest, controller) = run(2000, 0);
    let counts = controller.counts();
    assert!(counts.len() > 400_000, "{}", counts.len());
    // Consecutive: nothing lost, nothing repeated, from the first frame.
    assert!(counts.windows(2).all(|pair| pair[1] == pair[0] + 1));
    assert_eq!(counts[0], 1);
    let stats = pump.stats();
    assert_eq!((stats.underruns, stats.dropped, stats.starts), (0, 0, 1));
    assert_eq!(stats.silence, 0);
    assert!(controller.running);
}

#[test]
fn a_fast_guest_has_its_oldest_frames_dropped_and_the_delay_stays_bounded() {
    // The guest's clock runs 0.5 % fast: 240 frames a second more.
    let (pump, guest, controller) = run(4000, 5_000);
    in_order(&controller.counts());
    let stats = pump.stats();
    assert!(stats.dropped > 3000, "{}", stats.dropped);
    assert_eq!(stats.underruns, 0);
    // What the guest has sent less what was heard is what is in flight:
    // the queue, the ring, and what a drop has not yet removed.
    let config = Config::default();
    let last = *controller.counts().last().unwrap();
    let delay = (guest.next - 1 - last) as usize;
    assert!(
        delay <= config.max_queue + config.target_fill + TICK + AHEAD,
        "{delay}"
    );
}

#[test]
fn a_slow_guest_gets_silence_not_old_audio() {
    // 0.5 % slow: the guest is short 240 frames a second.
    let (pump, _guest, controller) = run(4000, -5_000);
    // Never a repeat or a step back, even where the ring ran short.
    in_order(&controller.counts());
    assert!(pump.stats().silence > 3000, "{}", pump.stats().silence);
    assert_eq!(pump.stats().underruns, 0);
}

#[test]
fn a_stall_is_an_underrun_that_plays_silence_and_then_resumes_cleanly() {
    let mut pump = Pump::new(Config::default());
    let mut guest = Guest::new();
    let mut controller = Controller::new();
    for _ in 0..200 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    // 60 ms without a service: the controller reads past the 20 ms the pump
    // had written, and on into frames the pump has scrubbed.
    for _ in 0..12 {
        controller.advance(TICK);
        guest.send(TICK);
    }
    pump.service(&mut guest, &mut controller).unwrap();
    assert_eq!(
        pump.stats().underruns,
        1,
        "the fill ran out within the stall"
    );
    for _ in 0..200 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    in_order(&controller.counts());
    // After it the stream is steady again, with no further underrun.
    let underruns = pump.stats().underruns;
    for _ in 0..400 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    assert_eq!(pump.stats().underruns, underruns);
}

#[test]
fn a_write_the_controller_refuses_as_too_close_is_an_underrun_that_recovers() {
    let mut pump = Pump::new(Config::default());
    let mut guest = Guest::new();
    let mut controller = Controller::new();
    for _ in 0..100 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    controller.refuse = 1;
    for _ in 0..300 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    in_order(&controller.counts());
    assert_eq!(pump.stats().underruns, 1);
}

#[test]
fn a_guest_that_stops_streaming_stops_the_stream_after_a_silence_and_a_new_one_starts_it() {
    let mut pump = Pump::new(Config::default());
    let mut guest = Guest::new();
    let mut controller = Controller::new();
    for _ in 0..100 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    assert_eq!(controller.starts, 1);
    guest.streaming = false;
    // What was queued still plays; then 200 ms of silence, then the stop.
    let mut ticks = 0;
    while controller.running {
        controller.advance(TICK);
        pump.service(&mut guest, &mut controller).unwrap();
        ticks += 1;
        assert!(ticks < 400, "the stream never stopped");
    }
    assert_eq!(controller.stops, 1);
    // Idle ticks do nothing, and nothing is started for a silent guest.
    for _ in 0..50 {
        controller.advance(TICK);
        pump.service(&mut guest, &mut controller).unwrap();
    }
    assert_eq!((controller.starts, controller.running), (1, false));
    // A guest that streams again, with enough queued, starts it again.
    guest.streaming = true;
    guest.send(3 * TICK);
    pump.service(&mut guest, &mut controller).unwrap();
    assert!(!controller.running, "15 ms are not a full prime yet");
    guest.send(TICK);
    pump.service(&mut guest, &mut controller).unwrap();
    assert_eq!((controller.starts, controller.running), (2, true));
    for _ in 0..100 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    in_order(&controller.counts());
}

#[test]
fn a_controller_that_starts_reading_elsewhere_is_followed_from_there() {
    let mut pump = Pump::new(Config::default());
    let mut guest = Guest::new();
    let mut controller = Controller::new();
    controller.start_at = 500;
    for _ in 0..300 {
        tick(&mut pump, &mut guest, &mut controller, 0).unwrap();
    }
    in_order(&controller.counts());
    // The prime is 960 frames at the ring's start; the controller began
    // inside it, so it read some of it and then nothing stale.
    assert!(pump.stats().underruns <= 1);
}

#[test]
fn the_guest_sample_is_the_high_sixteen_bits() {
    assert_eq!(sample(0x1234_5678), [0x34, 0x12]);
    assert_eq!(sample(-1), [0xff, 0xff]);
    assert_eq!(sample(i32::MIN), [0x00, 0x80]);
    assert_eq!(sample(i32::MAX), [0xff, 0x7f]);
}
