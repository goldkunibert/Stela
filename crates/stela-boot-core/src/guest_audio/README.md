# Guest audio (`--guest-audio`)

Status: the guest side, the bridge logic and the driver primitive are built
and tested on the host. The engine glue that owns the host's audio
controller is not written, so no sound plays yet.

## How it is meant to work

```
macOS ──USB Audio Class 2──▶ stela-soc usb::audio::Audio (200 ms ring, 48 kHz stereo)
                                   │  AppleDevices::audio_take
                                   ▼
                      stela-boot-core guest_audio::Pump        (clock-free)
                                   │  Sink::write / start / stop
                                   ▼
        stela-driver-audio hda::Controller::refill_playback    (guarded live write)
                                   │
                              HDA ring ──▶ codec ──▶ analog output
```

macOS binds its own USB audio class driver to the speaker, so no Apple audio
hardware is modelled.

## What exists

| Piece | Where | Tests |
|---|---|---|
| UAC2 speaker function (already existed, now attached) | `stela-soc/src/usb/audio.rs` | its own, plus `machine::tests::the_hub_gives_the_audio_speaker_its_port_and_the_disks_lose_one` |
| Port on the guest's hub, `AppleDevices::audio_*` | `stela-soc/src/machine.rs`, `machine/usb.rs` | the test above |
| `--guest-audio` option | `stela-boot-core/src/options.rs`, `options/parse.rs` | `options::tests::the_guest_audio_option_round_trips_and_is_off_by_default` |
| `Pump`, `Source`, `Sink` | `stela-boot-core/src/guest_audio.rs` | `guest_audio/tests.rs`: matched clock, ±0.5 % drift, stalls, refused writes, stop/restart |
| `Controller::refill_playback` | `stela-driver-audio/src/hda/live.rs` | `hda/tests/live_refill.rs` |

With `--guest-audio` set, macOS lists a "USB audio" output and sends it
sound; today that sound is dropped at the speaker's ring.

### The pump's guarantees (tested against a simulated controller)

- Frames reach the controller in order, never repeated.
- A guest clock that runs fast loses its oldest frames; the added delay stays
  under `max_queue + target_fill` (about 70 ms).
- A slow guest, a stall or a refused write plays silence. The pump writes
  silence over every frame the controller has passed, so a ring read through
  again plays nothing old.
- It must be serviced at least every `ring_frames / 4` of audio (10 ms for an
  80 ms ring).

### The driver primitive

`write_pcm` needs a stopped stream and the HDMI path has its own refill, so
`refill_playback` is new: it writes into a running analog ring only where the
link position shows the controller cannot be reading (a guard of 256 bytes
plus the FIFO depth ahead of LPIB), and re-reads the position after the copy.

## What is left: the engine glue

None of this can be built or checked off the target hardware; `stela-boot`
does not compile without `cargo xtask gpu fetch`, and the ownership rules
below are the project's, not this note's.

1. **Own the HDA controller after boot services end, for the whole boot.**
   `stela_drivers::probe::native::hda::attach` says what is missing: there is
   "no native original-owner IRQ/CPU/EOI/service/work completion producer", so
   it "retains/refuses before bind". The diagnostic (`audio_check.rs`,
   `start.rs::check_audio`) works around that with `--audio-own-hpet`
   (exclusive HPET and an exclusive legacy controller). A boot that plays
   sound needs either that same exclusive ownership or the missing producer,
   and a timer that is not the HPET the diagnostic takes.
2. **Pick the output.** `audio_check::route` refuses a codec with more than
   one standard analog route, and a Realtek ALC897 board has several jacks.
   The glue needs an explicit choice (for example a pin node on the option
   line) and `Controller::configure_realtek_route` for the Realtek codec.
3. **A `Sink` over `Controller` and one `StreamId`.**
   - `ring_frames`: the ring's bytes / 4.
   - `min_ahead_frames`: (256 + FIFO delay bytes) / 4.
   - `position_frames`: `position(id)` / 4.
   - `start(prime)`: `write_pcm(id, 0, prime)`, then `start(id)`.
   - `write`: `refill_playback`, mapping `Error::PlaybackUnderrun` to
     `Refusal::TooClose`.
   - `stop`: `stop(id)`, then `write_pcm` of zeros, because the trait promises
     a silent ring.
4. **A `Source` over `AppleDevices`**: `audio_streaming`, `audio_queued`,
   `audio_take`. `audio_take_due` says when 5 ms have arrived.
5. **Call `Pump::service`** from the engine's service loop, where the guest
   network is serviced (`engine/service.rs`, next to `network_status`), at
   least every 10 ms and when `audio_take_due` is set.
6. **Report `Pump::stats`** in the boot log: starts, underruns, silence frames
   and dropped frames are what a hardware run needs to show.

## Before any of that: run the diagnostic on the board

`--audio-check playback` (with `--audio-own-hpet`, in a `--features
diagnostics` build) plays a test tone through the board's codec after boot.
Its log shows whether the codec, the route and the timing work on this
machine, which decides how much of step 1 and 2 is real work.

The GPU's HDMI/DP audio function is parked by the probe and driven by no
driver here; only the motherboard's analog codec is in scope.
