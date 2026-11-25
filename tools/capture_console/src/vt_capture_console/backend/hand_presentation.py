"""Hide expired optical geometry on display copies of a live scene."""


def hide_hand(hand, state, reason):
    if hand['valid']:
        hand.update(valid=False, state=state, reason=reason,
                    position_m=None, quaternion_xyzw=None, joint_positions_m=[])


def expire_hands(scene, elapsed_ns):
    generated = scene.get('generated_realtime_ns')
    for hand in scene.get('hands', ()):
        if not hand['valid']:
            continue
        source = hand['source_realtime_ns']
        if (not isinstance(generated, int) or not isinstance(source, int)
                or not 0 <= generated - source + elapsed_ns <= 250_000_000):
            hide_hand(hand, 'offline', '手部观测已过期，等待新数据')
