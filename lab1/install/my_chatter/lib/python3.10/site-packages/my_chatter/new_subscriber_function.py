import rclpy
from rclpy.node import Node

from my_chatter_msgs.msg import TimestampString


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            TimestampString,
            'user_messages',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        sent_message = msg.user_input
        sent_timestamp = msg.timestamp
        received_timestamp = self.get_clock().now().nanoseconds
        self.get_logger().info(f"Message: {sent_message}, Sent at: {sent_timestamp}, Received at: {received_timestamp}")


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
