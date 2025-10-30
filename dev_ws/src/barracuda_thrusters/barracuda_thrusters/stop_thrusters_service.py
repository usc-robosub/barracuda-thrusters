import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

NTHRUSTERS = 8
TEST_FORCE = 2

class TestThrustersService(Node):
    def __init__(self):
        super().__init__('stop_thrusters_service')
        self.srv = self.create_service(Trigger, 'stop_thrusters_service', self.test_thrusters_callback)
        self.get_logger().info('Created service')
        self.pubs = [None] * NTHRUSTERS
        for thruster_idx in range(NTHRUSTERS):
            self.pubs[thruster_idx] = self.create_publisher(Float32, f'thrusters/input{thruster_idx}', 10)

    def test_thrusters_callback(self, request, response):
        self.timer = self.create_timer(1, self.timer_callback)
        self.get_logger().info('in service callback')
        response.success = True
        response.message = 'service callback ran'
        for thruster_idx in range(NTHRUSTERS):
            for i in range(2):
                self.publish_force_to_thruster(thruster_idx, 0)
        return response

    def publish_force_to_thruster(self, thruster_idx, force):
            msg = Float32()
            msg.data = float(force)
            self.pubs[thruster_idx].publish(msg)
    
    def timer_callback(self):
        self.timer.cancel()

def main():
    rclpy.init()

    stop_thrusters_service = TestThrustersService()

    rclpy.spin(stop_thrusters_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

