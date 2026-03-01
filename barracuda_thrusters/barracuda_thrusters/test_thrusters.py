"""
Automated Thruster Diagnostic Publisher

Use Case: 
This node is a diagnostic tool used to verify thruster hardware mapping and I2C communication. 
It cycles through all 8 thrusters (t0-t7) one by one, sending a temporary test effort value (5.0) 
to a single thruster every 2 seconds while keeping the others at 0.0.

Usage:
Run this node in a separate terminal alongside the main `barracuda_thrusters` node. 
- For Local Testing (Mac/Windows): Use it to watch the console logs and verify that the correct mock I2C addresses and registers are being targeted.
- For Hardware Testing (Jetson/Pi): Use it to physically verify motor spin direction and wiring on the actual robot without needing a full control system running.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ThrusterTestPublisher(Node):
    def __init__(self):
        super().__init__('thruster_test_publisher')
        
        self.publisher_ = self.create_publisher(JointState, '/barracuda/cmd_thrust', 10)
        
        timer_period = 2.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.n_thrusters = 8
        self.current_thruster_idx = 0
        self.test_effort_value = 5.0  # The fake thrust value to send
        
        self.get_logger().info('Starting automated thruster diagnostic sequence...')

    def timer_callback(self):
        msg = JointState()
        msg.name = ['t0', 't1', 't2', 't3', 't4', 't5', 't6', 't7']
        
        # Initialize all thrusters to 0.0
        msg.effort = [0.0] * self.n_thrusters
        
        msg.effort[self.current_thruster_idx] = self.test_effort_value
        
        self.get_logger().info(f'Testing Thruster {self.current_thruster_idx} (Array Index {self.current_thruster_idx}) at {self.test_effort_value} effort')
        
        self.publisher_.publish(msg)
        
        self.current_thruster_idx = (self.current_thruster_idx + 1) % self.n_thrusters

def main(args=None):
    rclpy.init(args=args)
    test_node = ThrusterTestPublisher()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('Diagnostic sequence stopped.')
    finally:
        test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()