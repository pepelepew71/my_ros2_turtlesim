#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from turtlesim import srv

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill


class Turtle:

    def __init__(self, master: "Tracking"):
        self.master = master
        self.num = 1
        self.x = None
        self.y = None
        self.t = None

    def set_random_state(self):
        self.x = random.uniform(0.0, 10.0)
        self.y = random.uniform(0.0, 10.0)
        self.t = random.uniform(-math.pi, math.pi)

    def get_distance_to(self, x: float, y: float):
        return ((self.x - x)**2.0 + (self.y - y)**2.0)**0.5

    def spawn(self):
        self.num += 1
        self.set_random_state()
        req = Spawn.Request()
        req.x = self.x
        req.y = self.y
        req.theta = self.t
        req.name = "turtle"+str(self.num)
        self.master.srv_client_spawn.call_async(request=req)
        self.master.get_logger().info(f'spawn turtle{self.num} at ({self.x:.3f}, {self.y:.3f})')


class Tracking(Node):

    def __init__(self):
        super().__init__(node_name="target_generator")
        self.get_logger().info(message="target_generator start")

        self.srv_client_spawn = self.create_client(srv_type=Spawn, srv_name="/spawn")
        while not self.srv_client_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn")

        self.srv_client_kill = self.create_client(srv_type=Kill, srv_name="/kill")
        while not self.srv_client_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /kill")

        self.turtle = Turtle(master=self)
        self.turtle.spawn()

        self.create_subscription(msg_type=Pose, topic="/turtle1/pose", callback=self._cb_turtle1_pose, qos_profile=10)
        self.pub_turtle1_cmdvel = self.create_publisher(msg_type=Twist, topic="/turtle1/cmd_vel", qos_profile=10)

    def _cb_turtle1_pose(self, msg):
        dist = self.turtle.get_distance_to(x=msg.x, y=msg.y)
        if dist < 0.5:
            req = Kill.Request()
            req.name = "turtle"+str(self.turtle.num)
            self.srv_client_kill.call_async(request=req)
            self.get_logger().info("Kill turtle"+str(self.turtle.num))
            self.turtle.spawn()
        else:
            twist = Twist()
            twist.linear.x = 2.0 if dist > 2.0 else dist
            twist.angular.z = - ((msg.theta - math.atan2(self.turtle.y - msg.y, self.turtle.x - msg.x)))
            self.get_logger().info(f'{twist.linear.x}, {twist.angular.z}')
            self.pub_turtle1_cmdvel.publish(msg=twist)


def main():
    rclpy.init(args=None)
    node = Tracking()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
