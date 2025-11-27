import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PyListener(Node):
    def __init__(self):
        super().__init__('py_listener')
        self.subscription = self.create_subscription(
            String,
            'py_chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = PyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
