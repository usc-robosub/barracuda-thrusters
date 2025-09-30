import rclpy
from rclpy.node import Node

from . import f2pwm
from . import serial

from std_msgs.msg import Float32

class BarracudaThrusters(Node):

    def __init__(self):
        super().__init__('barracuda_thrusters')

        self.declare_parameter('n_thrusters', 8)
        self.n_thrusters = self.get_parameter('n_thrusters').value
        assert self.n_thrusters == len(serial.thruster_regs), 'n_thrusters must equal length of the thruster_reg dict'

        self.verify_config_values()

        for thruster_idx in range(self.n_thrusters):
            topic = f'thrusters/input{thruster_idx}'
            self.create_subscription(
                Float32, 
                topic, 
                lambda msg, thruster_idx=thruster_idx: self.subscriber_callback(msg, thruster_idx), 
                10
            )

    def verify_config_values(self):
        teensy0_pwm_freq = serial.read_reg16(serial.pwm_freq_reg[serial.ADDR0])
        teensy0_pwm_bit_res = serial.read_reg16(serial.pwm_bit_res_reg[serial.ADDR0])

        teensy1_pwm_freq = serial.read_reg16(serial.pwm_freq_reg[serial.ADDR1])
        teensy1_pwm_bit_res = serial.read_reg16(serial.pwm_bit_res_reg[serial.ADDR1])
        if not (teensy0_pwm_freq is None and teensy0_pwm_bit_res is None and teensy1_pwm_freq is None and teensy1_pwm_bit_res is None): 
            pwm_config_assertion = 'one or both of the teensy pwm freq reg values are not equal to f2pwm.PWM_FREQ'
            assert teensy0_pwm_freq == teensy1_pwm_freq == f2pwm.PWM_FREQ, pwm_config_assertion
            # self.get_logger().error(pwm_config_assertion)
            pwm_bit_res_assertion = 'one or both of the teensy pwm bit res reg values are not equal to f2pwm.PWM_BIT_RES'
            assert teensy0_pwm_bit_res == teensy1_pwm_bit_res == f2pwm.PWM_BIT_RES, pwm_bit_res_assertion

    def subscriber_callback(self, msg, thruster_idx):
        thruster_force_newtons = msg.data
        pwm_duty_cycle_val = f2pwm.to_duty_cycle(thruster_force_newtons)
        serial.write_reg16(serial.thruster_regs[thruster_idx], pwm_duty_cycle_val)

def main():
    rclpy.init()

    barracuda_thrusters = BarracudaThrusters()

    rclpy.spin(barracuda_thrusters)

    barracuda_thrusters.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
