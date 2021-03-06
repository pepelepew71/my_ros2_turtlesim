#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill


class Target:

    def __init__(self, master: "Tracking"):
        """
        Due to kill and spawn the new turtle with the same name can't be shown on
        the turtlesim. So, use a serial number (num) to create new target turtle.
        """
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
        self.set_random_state()
        self.num += 1
        req = Spawn.Request()
        req.x = self.x
        req.y = self.y
        req.theta = self.t
        req.name = "target"+str(self.num)
        self.master.srv_client_spawn.call_async(request=req)
        self.master.get_logger().info(f'spawn target{self.num} at ({self.x:.3f}, {self.y:.3f})')


class Tracking(Node):

    SPAWN_DISTANCE = 0.25

    def __init__(self):
        """
        2 service clients to re-create target turtle. Subscribe turtle1's position,
        using P-control to minimize the position between target and turtle1, then
        publish command to turtle1.

        Attributes:
            srv_client_spawn
            srv_client_kill
            target
            pub_turtle1_cmdvel
        """
        super().__init__(node_name="tracking")
        self.get_logger().info(message="tracking start")

        self.srv_client_spawn = self.create_client(srv_type=Spawn, srv_name="/spawn")
        while not self.srv_client_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn")

        self.srv_client_kill = self.create_client(srv_type=Kill, srv_name="/kill")
        while not self.srv_client_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /kill")

        self.target = Target(master=self)
        self.target.spawn()

        self.create_subscription(msg_type=Pose, topic="/turtle1/pose", callback=self._cb_turtle1_pose, qos_profile=10)
        self.pub_turtle1_cmdvel = self.create_publisher(msg_type=Twist, topic="/turtle1/cmd_vel", qos_profile=10)

    def _cb_turtle1_pose(self, msg):
        """
        Callback for subscriber /turtle1/pose.
        """
        dist = self.target.get_distance_to(x=msg.x, y=msg.y)
        if dist < __class__.SPAWN_DISTANCE:
            req = Kill.Request()
            req.name = "target"+str(self.target.num)
            self.srv_client_kill.call_async(request=req)
            self.get_logger().info("Kill target"+str(self.target.num))
            self.target.spawn()
        else:
            twist = Twist()
            twist.linear.x = 1.0 if dist < 2.0 else dist / 2.0  # speed up
            twist.angular.z = (math.atan2(self.target.y - msg.y, self.target.x - msg.x) - msg.theta)
            if twist.angular.z > math.pi:
                twist.angular.z -= 2.0*math.pi
            elif twist.angular.z < -math.pi:
                twist.angular.z += 2.0*math.pi
            else:
                pass
            twist.angular.z *= 2.0  # speed up
            self.pub_turtle1_cmdvel.publish(msg=twist)


def main():
    rclpy.init(args=None)
    node = Tracking()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
