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

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            'LPPX/turtle1/pose',
            self.listener_callback,
            1000)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        import os
        os.system("clear")
        self.get_logger().info('X: "%s"' % msg.x)
        self.get_logger().info('Y: "%s"' % msg.y)
        self.get_logger().info('Theta: "%s"' % msg.theta)
        self.get_logger().info('Linear: "%s"' % msg.linear_velocity)
        self.get_logger().info('Angular: "%s"' % msg.angular_velocity)
        #self.get_logger().info('Y: "%s"' % msg.y)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
