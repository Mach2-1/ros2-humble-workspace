import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PyTalker(Node):
    def __init__(self):
        super().__init__('py_talker')
        self.publisher_ = self.create_publisher(String, 'py_chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from ROS2 (Python)! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PyTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
