#!/usr/bin/env python3
"""
bicycle_cmd_relay — converts /cmd_vel (Twist) into per-driver reference topics.

Bicycle model:
    steering_angle = atan2(wheelbase * angular_z, linear_x)   [rad, clipped to ±max_steer]
    velocity       = linear_x                                   [m/s]

Outputs:
    ~steering_topic  (std_msgs/Float64)  → steering_driver_controller/reference
    ~traction_topic  (std_msgs/Float64)  → traction_driver_controller/reference
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class BicycleCmdRelay(Node):

    def __init__(self):
        super().__init__('bicycle_cmd_relay')

        self.declare_parameter('wheelbase',       1.65)
        self.declare_parameter('max_steer_angle', 0.4)
        self.declare_parameter('steering_topic',
                               'steering_driver_controller/reference')
        self.declare_parameter('traction_topic',
                               'traction_driver_controller/reference')

        self._wb       = self.get_parameter('wheelbase').value
        self._max_steer = self.get_parameter('max_steer_angle').value
        steer_topic    = self.get_parameter('steering_topic').value
        tract_topic    = self.get_parameter('traction_topic').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._steer_pub = self.create_publisher(Float64, steer_topic, qos)
        self._tract_pub = self.create_publisher(Float64, tract_topic, qos)
        self._cmd_sub   = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_cb, qos)

        self.get_logger().info(
            f'bicycle_cmd_relay ready  wheelbase={self._wb} m  '
            f'max_steer={math.degrees(self._max_steer):.1f} deg  '
            f'steering→{steer_topic}  traction→{tract_topic}')

    def _cmd_cb(self, msg: Twist) -> None:
        vx = msg.linear.x
        wz = msg.angular.z

        # Bicycle model: δ = atan2(L·ω, v)
        # Guard against v≈0 to avoid NaN; saturate to ±max_steer
        if abs(vx) > 1e-3:
            angle = math.atan2(self._wb * wz, vx)
        elif abs(wz) > 1e-6:
            angle = math.copysign(self._max_steer, wz)
        else:
            angle = 0.0

        angle = max(-self._max_steer, min(self._max_steer, angle))

        steer_msg = Float64()
        steer_msg.data = angle
        self._steer_pub.publish(steer_msg)

        tract_msg = Float64()
        tract_msg.data = vx
        self._tract_pub.publish(tract_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BicycleCmdRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
