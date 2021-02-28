# content of test_sample.py
from test.samples.strace_log_1 import STRACE
from ros2_vesc_drv.vesc_msg import MessageFactory


def test_find_valid_message_sunny_day():
    messages = []
    msg_factory = MessageFactory(messages.append)
    for packet in STRACE:
        msg_factory.handle_packet(packet)
    msgs_decoded = len(messages)
    assert msgs_decoded == 4


def test_message_looping():
    assert True
