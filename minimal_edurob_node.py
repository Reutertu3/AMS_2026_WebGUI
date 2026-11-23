import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MinimalEduNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # cmd_vel publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        timer_period = 0.1  # seconds
        self.start = self.get_clock().now()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Get current position
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            self.get_logger().info(f'New pose: {t.transform.translation.x}, {t.transform.translation.y}, {t.transform.rotation.z}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform: {ex}')

        # Rechts für 0.5s und links für 0.5s
        t = (self.get_clock().now() - self.start).nanoseconds / 1e9
        msg = Twist()
        if t < 1.0:
            msg.linear.y = -0.1  # right
        elif t > 1.0 and t < 2.0:
            msg.linear.y = 0.1  # left
        else:
            raise SystemExit
        self.publisher_.publish(msg)

    def stop(self):
        # The robot keeps the last cmd_vel forever, so make sure a zero gets through
        for _ in range(20):
            self.publisher_.publish(Twist())
            time.sleep(0.05)


def main(args=None):
    # Default rclpy SIGINT handler shuts the context down before our finally runs,
    # so stop() could not publish. Let Python raise KeyboardInterrupt instead.
    with rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO):
        node = MinimalEduNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
            pass
        finally:
            node.stop()


if __name__ == '__main__':
    main()
