from coordination_node.offboard_sequence import offboard_request

TIMING = dict(first_tick=20, retry_ticks=30, arm_delay=2)


def _requests(confirmed_at=None, ticks=100):
    sent = {}
    for count in range(ticks):
        request = offboard_request(
            count, confirmed_at is not None and count >= confirmed_at,
            **TIMING)
        if request:
            sent[count] = request
    return sent


def test_nothing_sent_before_setpoints_have_been_streaming():
    assert all(count >= 20 for count in _requests())


def test_mode_then_arm_repeat_until_confirmed():
    # A first ARM that PX4 denies must not strand the drone: keep asking.
    assert _requests() == {20: 'mode', 22: 'arm', 50: 'mode', 52: 'arm',
                           80: 'mode', 82: 'arm'}


def test_stops_for_good_once_px4_confirms():
    assert _requests(confirmed_at=40) == {20: 'mode', 22: 'arm'}
