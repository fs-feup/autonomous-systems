# Copyright 2016 Open Source Robotics Foundation, Inc.
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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pacsim.msg import Wheels


class WheelsSubscriber(Node):

    def __init__(self):
        super().__init__('wheels_subscriber')
        self.subscription = self.create_subscription(
            Wheels,
            'pacsim/wheelspeeds',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        timestamp = msg.stamp
        fl_rot = msg.fl
        fr_rot = msg.fr
        rl_rot = msg.rl
        rr_rot = msg.rr

        self.get_logger().info(f"Velocity = {convertion(fl_rot, fr_rot, rl_rot, rr_rot)}")

def main(args=None):
    rclpy.init(args=args)

    wheels_subscriber = WheelsSubscriber()

    rclpy.spin(wheels_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wheels_subscriber.destroy_node()
    rclpy.shutdown()

def convertion(fl, fr, rl, rr):
    r = 0.206
    speed = (fl + fr + rl + rr)*r/4
    return speed


if __name__ == '__main__':
    main()
