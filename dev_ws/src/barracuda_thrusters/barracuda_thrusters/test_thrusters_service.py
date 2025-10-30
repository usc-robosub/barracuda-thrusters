import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# from barracuda_thrusters_interfaces.srv import TestThrusters
from std_srvs.srv import Trigger
import time

NTHRUSTERS = 8
TEST_FORCE = 2

class TestThrustersService(Node):
    def __init__(self):
        super().__init__('test_thrusters_service')
        self.srv = self.create_service(Trigger, 'test_thrusters_service', self.test_thrusters_callback)
        self.get_logger().info('Created service')
        self.pubs = [None] * NTHRUSTERS
        for thruster_idx in range(NTHRUSTERS):
            self.pubs[thruster_idx] = self.create_publisher(Float32, f'thrusters/input{thruster_idx}', 10)

    def test_thrusters_callback(self, request, response):
        self.get_logger().info('in service callback')
        response.success = True
        response.message = 'service callback ran'
        for thruster_idx in range(NTHRUSTERS):
            publish_force_to_thruster(thruster_idx, TEST_FORCE)
            time.sleep(1)
            publish_force_to_thruster(thruster_idx, 0)
            time.sleep(0.5)
        return response

    def publish_force_to_thruster(self, thruster_idx, force):
            msg = Float32()
            msg.data = force
            self.pubs[thruster_idx].publish(msg)

def main():
    rclpy.init()

    test_thrusters_service = TestThrustersService()

    rclpy.spin(test_thrusters_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

