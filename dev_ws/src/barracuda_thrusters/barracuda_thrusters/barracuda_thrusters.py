import rclpy
from rclpy.node import Node

from .f2pwm import F2PWM
from .serial_sender import SerialSender

from std_msgs.msg import Float32

class BarracudaThrusters(Node):

    def __init__(self):
        super().__init__('barracuda_thrusters')

        self.declare_parameter('n_thrusters', 8)
        self.n_thrusters = self.get_parameter('n_thrusters').value

        self.f2pwm = F2PWM()
        self.serial_sender = SerialSender(self.n_thrusters)

        for thruster_idx in range(self.n_thrusters):
            topic = f'thrusters/input{thruster_idx}'
            self.create_subscription(
                Float32, 
                topic, 
                lambda msg, thruster_idx=thruster_idx: self.subscriber_callback(msg, thruster_idx), 
                10
            )
           
    def subscriber_callback(self, msg, thruster_idx):
        thruster_force_newtons = msg.data
        pwm_duty_cycle_val = self.f2pwm.to_us(thruster_force_newtons)
        self.serial_sender.send(pwm_duty_cycle_val, thruster_idx)

def main():
    rclpy.init()

    barracuda_thrusters = BarracudaThrusters()

    rclpy.spin(barracuda_thrusters)

    barracuda_thrusters.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
