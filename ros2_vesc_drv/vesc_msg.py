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

import binascii
import struct
from dataclasses import dataclass, make_dataclass, asdict
from typing import Callable, Final, List

PAYLOAD_LEN_MAX: Final = 255  # maximum size (expectation limit) of a useful packet
PACK_LEN_MAX: Final = int(PAYLOAD_LEN_MAX * 3)  # maximum size of the buffered packet
BUF_LEN_MIN: Final = 6  # minimum buffer fill for a valid message + overhead
START_BYTE: Final = b"\x02"
END_BYTE: Final = 3
OVERHEAD_LEN: Final = 4
OFFSET_MSGLEN: Final = 1
OFFSET_PAYLOAD_START: Final = 2


@dataclass
class VescAttribute:
    name: str
    ctype: str
    divider: int = 1


class VescMessageFactory:
    def __init__(self, cls_name, attr_def: List[VescAttribute]):
        self._attrs = attr_def
        self._decode_str = "!" + "".join(map(lambda x: x.ctype, self._attrs))
        self.blueprint = make_dataclass(
            cls_name,
            list(map(lambda x: x.name, self._attrs)),
            namespace={"asdict": asdict},
        )
        self._dividers = list(map(lambda x: x.divider, self._attrs))

    @staticmethod
    def _prep_value(vpkg: List[int]):
        raw_val, divider = vpkg
        return raw_val if divider < 2 else raw_val / divider

    def decode(self, payload):
        raw = struct.unpack(self._decode_str, payload)
        vals = list(map(self._prep_value, zip(raw, self._dividers)))
        return self.blueprint(*vals)


# the below sequence is constructed following commands.cpp#L157 of vesc_tool
MC_VALS_DECODER = VescMessageFactory(
    "MCValuesMsg",
    [
        VescAttribute(name="command", ctype="B"),
        VescAttribute(name="temp_mos", ctype="h", divider=10),
        VescAttribute(name="temp_motor", ctype="h", divider=10),
        VescAttribute(name="current_motor", ctype="i", divider=100),
        VescAttribute(name="current_in", ctype="i", divider=100),
        VescAttribute(name="id", ctype="i", divider=100),
        VescAttribute(name="iq", ctype="i", divider=100),
        VescAttribute(name="duty_now", ctype="h", divider=1000),
        VescAttribute(name="rpm", ctype="i"),
        VescAttribute(name="v_in", ctype="h", divider=10),
        VescAttribute(name="amp_hours", ctype="i", divider=10000),
        VescAttribute(name="amp_hours_charged", ctype="i", divider=10000),
        VescAttribute(name="watt_hours", ctype="i", divider=10000),
        VescAttribute(name="watt_hours_charged", ctype="i", divider=10000),
        VescAttribute(name="tachometer", ctype="i"),
        VescAttribute(name="tachometer_abs", ctype="i"),
        VescAttribute(name="fault_code", ctype="B"),
        VescAttribute(name="position", ctype="i", divider=1000000),
        VescAttribute(name="vesc_id", ctype="B"),
        VescAttribute(name="temp_mos_1", ctype="h", divider=10),
        VescAttribute(name="temp_mos_2", ctype="h", divider=10),
        VescAttribute(name="temp_mos_3", ctype="h", divider=10),
        VescAttribute(name="vd", ctype="i", divider=100),
        VescAttribute(name="vq", ctype="i", divider=100),
    ],
)


class MessageUnpacker:
    """
    Given serial packets produces valid VESC payloads (when found).

    A valid VESC message is expected to have the following structure:

    * START_BYTE (x02),
    * msg_len - message length byte
    * payload, bytes array of length matching the value of msg_len
    * crc16, 2 bytes, ">H"
    * END_BYTE (x03)
    """

    def __init__(self, handle_message: Callable) -> None:
        self.handle_message: Callable = handle_message
        self.buffer: bytes = b""

    def handle_packet(self, packet: bytes):
        self.buffer += packet if len(packet) <= PACK_LEN_MAX else packet[:-PACK_LEN_MAX]
        self._evaluate_buffer()

    def _evaluate_buffer(self) -> bool:
        msg_start = self.buffer.find(START_BYTE)
        if msg_start < 0:  # check if buffer has a start byte
            return False
        self.buffer = self.buffer[msg_start:]
        buf_len = len(self.buffer)
        if buf_len < BUF_LEN_MIN:
            return False
        msg_len = self.buffer[OFFSET_MSGLEN]
        if msg_len + OVERHEAD_LEN >= buf_len:
            return False
        if self.buffer[msg_len + OVERHEAD_LEN] != END_BYTE:
            self.buffer = self.buffer[1:]
            # not a message, start over from the next start byte
            while self._evaluate_buffer():
                pass
            return True
        payload = self.buffer[OFFSET_PAYLOAD_START : OFFSET_PAYLOAD_START + msg_len]
        msg_end = msg_len + OVERHEAD_LEN
        crc_should = struct.unpack(">H", self.buffer[msg_end - 2 : msg_end])[0]
        crc_is = binascii.crc_hqx(payload, 0)
        if crc_should == crc_is:
            self.handle_message(payload)
            self.buffer = self.buffer[1:]
            while self._evaluate_buffer():
                pass
            return True
        else:
            # not a valid message, start over from the next start byte
            self.buffer = self.buffer[1:]
            while self._evaluate_buffer():
                pass
            return True
