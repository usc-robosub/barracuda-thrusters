import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# from barracuda_thrusters_interfaces.srv import TestThrusters
from std_srvs.srv import Empty

NTHRUSTERS = 8

class TestThrustersService(Node):
    def __init__(self):
        super().__init__('test_thrusters_service')
        self.srv = self.create_service(TestThrusters, 'test_thrusters_service', self.test_thrusters_callback)
        self.get_logger().info('Created service')
        self.pubs = [None] * NTHRUSTERS
        for thruster_idx in range(NTHRUSTERS):
            self.pubs[thruster_idx] = self.create_publisher(Float32, f'thrusters/input{thruster_idx}', 10)

    def test_thrusters_callback(self, request, response):
        response.res = f'Publishing force {request.force} to thruster topics: '
        thrusters = [i for i in range(NTHRUSTERS)] if not request.thrusters else request.thrusters
        for thruster_idx in thrusters:
            msg = Float32()
            msg.data = request.force
            self.pubs[thruster_idx].publish(msg)
            response.res += f'{thruster_idx} '
        return response

def main():
    rclpy.init()

    test_thrusters_service = TestThrustersService()

    rclpy.spin(test_thrusters_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

