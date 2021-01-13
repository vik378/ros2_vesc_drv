# Copyright 2021 Viktor Kravchenko (viktor@vik.works)

# Licensed under the Apache License, Version 2.0 (the "License")
#  you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at

#      http: // www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.


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
CONTROL_MODES: Final = frozenset(["duty", "speed", "position"])
DEFAULT_CONTROL_MODE: Final = "duty"


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
        self.ser = None
        self.declare_parameter("serial_port", DEFAULT_SERIAL_ADDR)
        self.declare_parameter("control_mode", DEFAULT_CONTROL_MODE)
        self.bind_serial_port(self.get_parameter("serial_port").value)
        self.set_control_mode(self.get_parameter("control_mode").value)
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
        self.add_on_set_parameters_callback(self.handle_parameters_change)

    def handle_parameters_change(self, params):
        for param in params:
            if param.name == "serial_port":
                self.bind_serial_port(param.value)
            if param.name == "control_mode":
                self.set_control_mode(param.value)
        return SetParametersResult(successful=True)

    def set_control_mode(self, control_mode: str):
        if control_mode not in CONTROL_MODES:
            self.get_logger().warn(
                f"Requested control mode is not supported, pick from {CONTROL_MODES}"
            )

    def bind_serial_port(self, path_to_port: str):
        if hasattr(self, "ser") and isinstance(self.ser, serial.Serial):
            self.ser.close()
        self.get_logger().info(f"Attaching VESC driver to {path_to_port}")
        try:
            self.ser = serial.Serial(
                path_to_port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT
            )
        except Exception as e:
            self.get_logger().warning(f"failed to open port: {e}")
            self.ser = None
        else:
            self.get_logger().info(f"Successfully attached to {path_to_port}")

    def check_last_update(self):
        """Checks if there is a recent message from demands publisher and if there was
        no recent demand update recets demands to zero."""
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
