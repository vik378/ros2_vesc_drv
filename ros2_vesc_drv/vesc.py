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


import rclpy
import struct
import binascii
import serial
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from typing import Final, Union, TypeVar
from std_msgs.msg import Float32

MsgType = TypeVar("MsgType")


CAN_ADDR_R: Final = 1
CAN_ADDR_L: Final = -1
UPDATE_RATE: Final = 10  # Hz
NODE_NAME: Final = "vesc_diff_drv"
TOPIC_LEFT: Final = "/vesc_L/duty"  # Float32
TOPIC_RIGHT: Final = "/vesc_R/duty"  # Float32
DEFAULT_SERIAL_ADDR: Final = "/dev/ttyACM0"
SERIAL_BAUDRATE: Final = 115200
SERIAL_TIMEOUT: Final = 1  # second


COMM_SET_DUTY: Final = b"\x06"
COMM_FORWARD_CAN: Final = b"\x22"


class SetDutyMsg:
    def __init__(self, val: float, can_addr: int = -1):
        self.val = val
        self.can_addr = can_addr

    @property
    def as_bytes(self):
        payload = COMM_SET_DUTY + struct.pack(">i", int(self.val * 10000))
        if self.can_addr > -1:
            if self.can_addr > 127:
                raise ValueError("CAN ID > 127 is not supported at the moment")
            payload = COMM_FORWARD_CAN + self.can_addr.to_bytes(1, "big") + payload
        crc = binascii.crc_hqx(payload, 0).to_bytes(2, "big")
        return b"\x02" + len(payload).to_bytes(1, "big") + payload + crc + b"\x03"


class VESCDiffDriver(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.duty_left = 0.0
        self.duty_right = 0.0
        self.declare_parameter("serial_port", DEFAULT_SERIAL_ADDR)
        self.serial_port = self.get_parameter("serial_port").value
        self.last_stamp_R = self.get_clock().now()
        self.last_stamp_L = self.get_clock().now()
        self.subs_duty_left = self.create_subscription(
            Float32, TOPIC_LEFT, self.set_duty_left, UPDATE_RATE
        )
        self.subs_duty_right = self.create_subscription(
            Float32, TOPIC_RIGHT, self.set_duty_right, UPDATE_RATE
        )
        self.tmr = self.create_timer(0.025, self.update_vesc_demands)
        self.subs_duty_left
        self.subs_duty_right
        self.ser = None
        self.bind_serial()
        self.add_on_set_parameters_callback(self.handle_parameters_change)

    def handle_parameters_change(self, params):
        for param in params:
            if param.name == "serial_port":
                self.serial_port = param.value
                self.bind_serial()
        return SetParametersResult(successful=True)

    def bind_serial(self):
        if not self.serial_port:
            raise AttributeError("Serial port is not defined, nothing to do")
        if hasattr(self, "ser") and isinstance(self.ser, serial.Serial):
            self.ser.close()
        self.get_logger().info(f"Attaching VESC driver to {self.serial_port}")
        try:
            self.ser = serial.Serial(
                self.serial_port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT
            )
        except Exception as e:
            self.get_logger().warning(f"failed to open port: {e}")
            self.ser = None
        else:
            self.get_logger().warning(f"Successfully attached to {self.serial_port}")

    def check_last_update(self):
        """
        Reset demands to 0 if no update has arrived before a deadline.

        Rationale: in case of upper level component failure (to produce a command
        before a deadline) this will protect the system from uncontrolled rotation
        of the managed motor.
        """
        now = self.get_clock().now()
        diff_L = (now - self.last_stamp_L).nanoseconds * 1e-9
        diff_R = (now - self.last_stamp_R).nanoseconds * 1e-9
        if diff_L > 0.1:
            self.duty_left = 0.0
        if diff_R > 0.1:
            self.duty_right = 0.0

    def set_duty_left(self, msg: Union[MsgType, bytes]):
        self.duty_left = msg.data
        self.last_stamp_L = self.get_clock().now()

    def set_duty_right(self, msg: Union[MsgType, bytes]):
        self.duty_right = msg.data
        self.last_stamp_R = self.get_clock().now()

    def update_vesc_demands(self):
        self.check_last_update()
        demand_L = SetDutyMsg(self.duty_left, can_addr=CAN_ADDR_L)
        demand_R = SetDutyMsg(self.duty_right, can_addr=CAN_ADDR_R)
        if self.ser:
            self.ser.write(demand_L.as_bytes)
            self.ser.write(demand_R.as_bytes)

    def __del__(self):
        if hasattr(self, "ser") and isinstance(self.ser, serial.Serial):
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        vesc_diff_drv = VESCDiffDriver()
        executor.add_node(vesc_diff_drv)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            vesc_diff_drv.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
