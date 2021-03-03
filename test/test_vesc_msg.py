from ros2_vesc_drv.vesc_msg import MessageFactory, PACK_LEN_MAX

MSG_SHALL_NOT_PASS = [
    b"0",  # no start byte
    b"a",  # too short still
]


def test_raw_packet_handling():
    messages = []
    msg_factory = MessageFactory(messages.append)
    list(
        map(
            msg_factory.handle_packet,
            [
                b"0",  # no start byte
                b"\x02",  # has start byte but is too short
                b"\x041234",  # isnt long enough
                b"5678",  # is long enough but bad terminator
                b"\x02\x02\x00\x00\x01\x01\x02\x01\x02\x02\x03\x03\x03",  # bad termbyte
                b"\x02\x01012\x03\x02\x01012\x03",  # bad CRC
                b"\2\1\36\363\377\3",  # one good msg
                b"\2\1\36\363\377\3\2\1\36\363\377\3",  # 2 good msgs in one buff
                b"0" * PACK_LEN_MAX * 4,  # how about buffer longer than max allowed?
            ],
        )
    )
    msgs_decoded = len(messages)
    assert msgs_decoded == 3
