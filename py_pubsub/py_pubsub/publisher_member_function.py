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

import numpy as np


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.pose_goal = Pose()
        self.pose = Pose()
        self.init_  = self.init_variables()
        self.subs = self.init_subscriber()
        self.pubs = self.init_publisher()


    def init_variables(self):
        self.pose_goal.x = 3.0
        self.pose_goal.y = 9.0
        self.pose_goal.theta = 0.0

    def goal_callback(self,msg):
        self.pose_goal.x = msg.x
        self.pose_goal.y = msg.y
        self.pose_goal.theta = msg.theta
        self.get_logger().info('Goal X: "%s"' % self.pose_goal.x)
        self.get_logger().info('Goal Y: "%s"' % self.pose_goal.y)

    def init_publisher(self):
        self.publisher_ = self.create_publisher(Twist, '/LPPX/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.i = 0
        return self.publisher_

    def pub_callback(self):
        msg = Twist()
        # posicao desejada
        #self.xd = 2
        #self.yd = 9
        pos_desejada = np.array([self.pose_goal.x,self.pose_goal.y])
        # posicao atual
        pos_atual = np.array([self.pose.x,self.pose.y])
        # conversao em coordenadas polares
        erro = pos_desejada - pos_atual
        ro = np.linalg.norm(erro)
        alpha = np.arctan2(erro[1],erro[0]) - self.pose.theta

        if abs(self.pose_goal.x-self.pose.x) > 0.2 or abs(self.pose_goal.y-self.pose.y) > 0.2:
            # regra de controle
            vmax = 0.7
            vel_lin = vmax*np.tanh(ro)
            kw = 0.6
            vel_ang = kw*alpha

            msg.linear.x = vel_lin
            #msg.linear.y = vel_lin
            msg.angular.z = vel_ang
        else:
            msg.linear.x = 0.0
            #msg.linear.y = 0.0
            msg.angular.z = 0.0
        
        #msg.x=1.0
        #msg.y=2.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing vel_lin: "%s"' % msg.linear.x)
        self.get_logger().info('Publishing vel_ang: "%s"' % msg.angular.z)
        self.get_logger().info('Distancia Euclidiana: "%s"' % str(ro))
        self.get_logger().info('Error X: "%s"' % str(self.pose_goal.x-self.pose.x))
        self.get_logger().info('Error Y: "%s"' % str(self.pose_goal.y-self.pose.y))
        self.i += 1

    def init_subscriber(self):
        self.subscription_0 = self.create_subscription(
            Pose,
            '/LPPX/goal',
            self.goal_callback,
            100)
        self.subscription_0

        self.subscription_ = self.create_subscription(
            Pose,
            '/LPPX/turtle1/pose',
            self.pose_callback,
            1000)
        self.subscription_  # prevent unused variable warning

        return self.subscription_

    def pose_callback(self, msg):
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = msg.theta

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
