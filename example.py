import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

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
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Get current position
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            self.get_logger().info(f'New pose: {t.transform.translation.x}, {t.transform.translation.y}, {t.transform.rotation.z}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform: {ex}')
            return
        
        # Set speeds
        t = (self.get_clock().now() - self.start).nanoseconds / 1e9
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_edurob_node = MinimalEduNode()

            rclpy.spin(minimal_edurob_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
