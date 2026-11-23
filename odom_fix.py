#!/usr/bin/env python3
"""Re-stamps the robot odometry for slam_toolbox (runs in the edurob-slam container, see start.sh --slam).

The robot's /tf timestamps are broken: the seconds are right, but the nanoseconds barely change
(+0.2 ms per message instead of +200 ms), so every pose looks up to ~0.7 s old and all poses
within one second carry almost the same time. slam_toolbox looks up the pose at the exact time
of each scan and would get wrong poses.

So this node republishes the robot's odom -> base_link with the laptop clock as a separate tree:
    odom_fix -> base_fix -> laser_fix      (the lidar driver uses frame_id laser_fix)
slam_toolbox works on that tree and adds map -> odom_fix.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

DELAY = 0.04  # s, age of an odometry message when it arrives over WiFi (measured vs. lidar while turning)
LASER_Z = 0.08     # m, lidar above base_link (from the robot's /tf_static)
LASER_YAW = 0.0    # lidar mount angle, measured: not rotated (README.md)


class OdomFix(Node):
    def __init__(self):
        super().__init__('odom_fix')
        self.broadcaster = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_fix', 'laser_fix'
        t.transform.translation.z = LASER_Z
        t.transform.rotation.z, t.transform.rotation.w = math.sin(LASER_YAW / 2), math.cos(LASER_YAW / 2)
        self.static.sendTransform(t)
        self.last_stamp = None
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') != 'odom' or t.child_frame_id.lstrip('/') != 'base_link':
                continue
            stamp = self.get_clock().now() - Duration(seconds=DELAY)
            if self.last_stamp is not None and stamp <= self.last_stamp:
                continue  # tf needs increasing stamps
            self.last_stamp = stamp
            out = TransformStamped()
            out.header.stamp = stamp.to_msg()
            out.header.frame_id, out.child_frame_id = 'odom_fix', 'base_fix'
            out.transform = t.transform
            self.broadcaster.sendTransform(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(OdomFix())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
