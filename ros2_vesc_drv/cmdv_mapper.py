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


from typing import Final
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


TOPIC_CMD_VEL: Final = "/cmd_vel"
TOPIC_VESC_LEFT: Final = "/vesc_L/duty"
TOPIC_VESC_RIGHT: Final = "/vesc_R/duty"


class BasicRemapper(Node):
    def __init__(self):
        super().__init__("cmd_vel_remapper")
        self.subs_cmdv = self.create_subscription(
            Twist, TOPIC_CMD_VEL, self.remap_cmd_vel, 10
        )
        self.pub_vesc_l = self.create_publisher(Float32, TOPIC_VESC_LEFT, 10)
        self.pub_vesc_r = self.create_publisher(Float32, TOPIC_VESC_RIGHT, 10)
        self.subs_cmdv

    def remap_cmd_vel(self, msg: Twist):
        duty_l = msg.linear.x
        duty_r = msg.angular.z
        msg = Float32()
        msg.data = duty_l
        self.pub_vesc_l.publish(msg)
        msg.data = duty_r
        self.pub_vesc_r.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    remapper = BasicRemapper()
    rclpy.spin(remapper)
    remapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
