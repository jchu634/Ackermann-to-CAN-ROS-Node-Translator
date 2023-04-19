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
import random
from rclpy.node import Node

from ackermann_can_interfaces.msg import AckermannDrive
from ackermann_can_interfaces.msg import AckermannDriveStamped



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.drive.steering_angle = random.uniform(-3.14,3.14)
        msg.drive.steering_angle_velocity = random.uniform(-5,5)
        msg.drive.speed = random.uniform(0,100)
        msg.drive.acceleration = random.uniform(0,20)
        msg.drive.jerk = random.uniform(0,20)
        
        # msg.drive.steering_angle = 2.43861236989
        # msg.drive.steering_angle_velocity = 1.453125000612314
        # msg.drive.speed = 103.000013131231
        # msg.drive.acceleration = 14.312543750131231
        # msg.drive.jerk = 13.40625131231
        


        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.header.frame_id)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
