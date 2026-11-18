// SPDX-License-Identifier: GPL-2.0-only
//! Refilling a running analog playback ring.
use super::*;

/// A started 4096-byte stereo playback ring and its position register.
fn running() -> (Host, Rc<RefCell<Model>>, Fixture, StreamId, usize) {
    let mut fixture = Fixture::new();
    let model = Model::new(256);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let id = controller
        .allocate_stream(
            Direction::Playback,
            stereo(),
            1024,
            4,
            &mut fixture.allocator,
        )
        .unwrap();
    controller.write_pcm(id, 0, &[0; 4096]).unwrap();
    controller.start(id).unwrap();
    let lpib = stream_base(id.descriptor_index()) + SD_LPIB;
    (controller, model, fixture, id, lpib)
}

fn ring(controller: &Host, id: StreamId) -> Vec<u8> {
    controller.stream(id).unwrap().buffer.bytes().to_vec()
}

#[test]
fn a_write_ahead_of_the_read_position_lands_in_the_ring() {
    let (mut controller, model, _fixture, id, lpib) = running();
    model.borrow_mut().set(lpib, 2048, 4);
    // Ahead by exactly the guard, and ending where the ring meets LPIB.
    controller.refill_playback(id, 2304, &[0x11; 1024]).unwrap();
    controller.refill_playback(id, 3328, &[0x22; 768]).unwrap();
    controller.refill_playback(id, 0, &[0x33; 2048]).unwrap();
    let ring = ring(&controller, id);
    assert!(ring[2304..3328].iter().all(|&byte| byte == 0x11));
    assert!(ring[3328..4096].iter().all(|&byte| byte == 0x22));
    assert!(ring[0..2048].iter().all(|&byte| byte == 0x33));
}

#[test]
fn a_write_where_the_controller_reads_is_refused_and_leaves_the_ring_alone() {
    let (mut controller, model, _fixture, id, lpib) = running();
    model.borrow_mut().set(lpib, 2048, 4);
    // At LPIB, inside the guard, and one word short of it.
    for offset in [2048, 2052, 2300] {
        assert_eq!(
            controller.refill_playback(id, offset, &[0xff; 4]),
            Err(Error::PlaybackUnderrun),
            "{offset}"
        );
    }
    // Reaching LPIB from behind wraps onto the controller's read position.
    assert_eq!(
        controller.refill_playback(id, 1024, &[0xff; 1028]),
        Err(Error::PlaybackUnderrun)
    );
    assert!(ring(&controller, id).iter().all(|&byte| byte == 0));
}

#[test]
fn the_guard_follows_the_position_when_the_ring_wraps() {
    let (mut controller, model, _fixture, id, lpib) = running();
    // LPIB at the end of the ring reads as the start.
    model.borrow_mut().set(lpib, 4096, 4);
    assert_eq!(
        controller.refill_playback(id, 0, &[1; 256]),
        Err(Error::PlaybackUnderrun)
    );
    controller.refill_playback(id, 256, &[1; 256]).unwrap();
    model.borrow_mut().set(lpib, 3900, 4);
    assert_eq!(
        controller.refill_playback(id, 3900, &[1; 4]),
        Err(Error::PlaybackUnderrun)
    );
    // Four bytes before LPIB is the farthest point from it.
    controller.refill_playback(id, 3896, &[2; 4]).unwrap();
}

#[test]
fn malformed_or_misdirected_writes_are_invalid_buffers() {
    let (mut controller, model, _fixture, id, lpib) = running();
    model.borrow_mut().set(lpib, 0, 4);
    for (offset, length) in [(0, 0), (1024, 3), (1026, 4), (4092, 8), (usize::MAX - 3, 8)] {
        assert_eq!(
            controller.refill_playback(id, offset, &alloc::vec![0; length]),
            Err(Error::InvalidBuffer),
            "{offset} {length}"
        );
    }
}

#[test]
fn a_stopped_stream_and_a_capture_stream_take_no_live_writes() {
    let (mut controller, _model, mut fixture, id, _lpib) = running();
    let capture = controller
        .allocate_stream(
            Direction::Capture,
            stereo(),
            1024,
            4,
            &mut fixture.allocator,
        )
        .unwrap();
    controller.start(capture).unwrap();
    assert_eq!(
        controller.refill_playback(capture, 1024, &[0; 4]),
        Err(Error::InvalidBuffer)
    );
    controller.stop(id).unwrap();
    assert_eq!(
        controller.refill_playback(id, 1024, &[0; 4]),
        Err(Error::StreamStopped)
    );
}
