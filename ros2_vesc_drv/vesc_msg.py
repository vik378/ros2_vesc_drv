from typing import Callable, Final
import struct
import binascii

PACKET_LEN_MAX: Final = 255  # maximum size (expectation limit) of a useful packet
BUF_LEN_MAX: Final = int(PACKET_LEN_MAX * 3)  # maximum size of the buffer
BUF_LEN_MIN: Final = 6
START_BYTE: Final = b"\x02"
END_BYTE: Final = 3
OVERHEAD_LEN: Final = 4
OFFSET_MSGLEN: Final = 1
OFFSET_PAYLOAD_START: Final = 2


class MessageFactory:
    """
    Given serial packets produces valid VESC messages.
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
        self.buffer += packet
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
