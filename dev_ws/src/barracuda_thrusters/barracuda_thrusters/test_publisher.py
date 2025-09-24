import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.pubs = []
        for i in range(8):
            self.pubs.append(self.create_publisher(Float32, f'thrusters/input{i}', 10))
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        for idx in range(8):
            msg = Float32()
            msg.data = 23.56
            # self.publisher_.publish(msg)
            self.pubs[idx].publish(msg)
            self.get_logger().info(f'Publishing: {msg.data} for thruster {idx}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()