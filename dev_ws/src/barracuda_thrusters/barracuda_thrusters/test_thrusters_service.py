import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from barracuda_thrusters_interfaces.srv import TestThrusters

class TestThrustersService(Node):
    def __init__(self):
        super().__init__('test_thrusters_service')
        self.srv = self.create_service(TestThrusters, 'test_thrusters_service', self.test_thrusters_callback)
        self.get_logger().info('Created service')

    def test_thrusters_callback(self, request, response):
        if request.thrusters == []:
            request.thrusters = [0, 1, 2, 3, 4, 5, 6, 7]
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.thrusters, request.force))
        return response

def main():
    rclpy.init()

    test_thrusters_service = TestThrustersService()

    rclpy.spin(test_thrusters_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

