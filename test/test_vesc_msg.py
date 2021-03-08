# Copyright 2021 Viktor Kravchenko (viktor@vik.works)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ros2_vesc_drv.vesc_msg import MessageUnpacker, PACK_LEN_MAX, MC_VALS_DECODER

MSG_SHALL_NOT_PASS = [
    b"0",  # no start byte
    b"a",  # too short still
]


def test_raw_packet_handling():
    messages = []
    msg_factory = MessageUnpacker(messages.append)
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


def test_decode_mc_values():
    CAPTURED_MC_VALUES_SAMPLE = [
        b"\2I\4\0011\0\350\0\0\0\23\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\3\1g\0\0\0?\0\0",
        b"\0\0\0\0\10\353\0\0\0\0\0\0\2\377\0\0\3043\0\0\0\0\0\v\0\0\0\0\0\0\0\0\0\0",
        b"\0\0\0\0 \351\3",  # read
    ]
    expects = MC_VALS_DECODER.blueprint(
        command=4,
        temp_mos=30.5,
        temp_motor=23.2,
        current_motor=0.19,
        current_in=0.0,
        id=0.0,
        iq=0.0,
        duty_now=0.0,
        rpm=3,
        v_in=35.9,
        amp_hours=0.0063,
        amp_hours_charged=0.0,
        watt_hours=0.2283,
        watt_hours_charged=0.0,
        tachometer=767,
        tachometer_abs=50227,
        fault_code=0,
        position=0.0,
        vesc_id=11,
        temp_mos_1=0.0,
        temp_mos_2=0.0,
        temp_mos_3=0.0,
        vd=0.0,
        vq=0.0,
    )
    messages = []
    msg_factory = MessageUnpacker(messages.append)
    list(map(msg_factory.handle_packet, CAPTURED_MC_VALUES_SAMPLE))
    assert len(messages) == 1
    mc_values = MC_VALS_DECODER.decode(messages[0])
    assert isinstance(mc_values, MC_VALS_DECODER.blueprint)
    assert mc_values == expects
