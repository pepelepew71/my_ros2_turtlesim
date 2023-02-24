#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill


class Target:

    def __init__(self, master: "Tracker"):
        """
        Kill and spawn a turtle with the same name is not working for turtlesim.
        """
        self._master = master
        self._serial_number = 1
        self.x = None
        self.y = None
        self.t = None
        self.spawn()  # spawn first target turtle

    def _set_random_state(self):
        self.x = random.uniform(0.0, 10.0)
        self.y = random.uniform(0.0, 10.0)
        self.t = random.uniform(-math.pi, math.pi)

    def get_distance_to(self, x: float, y: float):
        return ((self.x - x)**2.0 + (self.y - y)**2.0)**0.5

    def kill(self):
        req = Kill.Request()
        req.name = "target"+str(self._serial_number)
        self._master.srv_client_kill.call_async(request=req)
        self._master.get_logger().info("kill target"+str(self._serial_number))

    def spawn(self):
        self._set_random_state()
        self._serial_number += 1
        req = Spawn.Request()
        req.x = self.x
        req.y = self.y
        req.theta = self.t
        req.name = "target"+str(self._serial_number)
        self._master.srv_client_spawn.call_async(request=req)
        self._master.get_logger().info(f'spawn target{self._serial_number} at ({self.x:.3f}, {self.y:.3f})')


class Tracker(Node):

    DISTANCE_TOL = 0.25

    def __init__(self):
        """
        Use services /turtlesim/kill and /turtlesim/spawn to keep creating a
        new turtle<number>. Then use P-control to control turtle1's location.
        """
        super().__init__(node_name="tracking")

        # -- service client
        self.srv_client_spawn = self.create_client(srv_type=Spawn, srv_name="/spawn")
        while not self.srv_client_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn")

        self.srv_client_kill = self.create_client(srv_type=Kill, srv_name="/kill")
        while not self.srv_client_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /kill")

        # -- subscriber
        self.create_subscription(msg_type=Pose, topic="/turtle1/pose", callback=self._callback_turtle1_pose, qos_profile=10)

        # -- publisher
        self.pub_turtle1_cmdvel = self.create_publisher(msg_type=Twist, topic="/turtle1/cmd_vel", qos_profile=10)

        # -- target object
        self.target = Target(master=self)

    def _callback_turtle1_pose(self, msg):
        """
        Callback for subscriber /turtle1/pose.
        """
        dist = self.target.get_distance_to(x=msg.x, y=msg.y)
        if dist < __class__.DISTANCE_TOL:
            self.target.kill()
            self.target.spawn()
        else:
            twist = Twist()
            twist.linear.x = 1.0 if dist < 2.0 else dist / 2.0
            twist.angular.z = (math.atan2(self.target.y - msg.y, self.target.x - msg.x) - msg.theta)
            if twist.angular.z > math.pi:
                twist.angular.z -= 2.0*math.pi
            elif twist.angular.z < -math.pi:
                twist.angular.z += 2.0*math.pi
            else:
                pass
            twist.angular.z *= 2.0
            self.pub_turtle1_cmdvel.publish(msg=twist)


def main():
    rclpy.init(args=None)
    node = Tracker()
    node.get_logger().info(message="tracking start")
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
