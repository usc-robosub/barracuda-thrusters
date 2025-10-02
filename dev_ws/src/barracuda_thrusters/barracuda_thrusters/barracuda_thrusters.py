import rclpy
from rclpy.node import Node

from . import f2pwm
from . import teensy

from std_msgs.msg import Float32

class BarracudaThrusters(Node):

    def __init__(self):
        super().__init__('barracuda_thrusters')

        self.declare_parameter('n_thrusters', 8)
        self.n_thrusters = self.get_parameter('n_thrusters').value

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
        for i2c_addr in teensy.i2c_addresses:
            teensy_pwm_freq = teensy.read_i2c_16(i2c_addr, teensy.PWM_FREQ_REG)
            teensy_pwm_bit_res = teensy.read_i2c_16(i2c_addr, teensy.PWM_BIT_RES_REG)
            teensy_t200_init = teensy.read_i2c_16(i2c_addr, teensy.T200_INIT_REG)
            if not (teensy_pwm_freq is None and teensy_pwm_bit_res is None and teensy_t200_init is None):
                if teensy_pwm_freq != f2pwm.PWM_FREQ:
                    self.get_logger().info('one or both of the teensy pwm freq reg values are not equal to f2pwm.PWM_FREQ')
                assert teensy_pwm_freq == f2pwm.PWM_FREQ, 'one or both of the teensy pwm freq reg values are not equal to f2pwm.PWM_FREQ'

                if teensy_pwm_bit_res != f2pwm.PWM_BIT_RES:
                    self.get_logger().info('one or both of the teensy pwm freq reg values are not equal to f2pwm.PWM_FREQ')
                assert teensy_pwm_bit_res == f2pwm.PWM_BIT_RES, 'one or both of the teensy pwm bit res reg values are not equal to f2pwm.PWM_BIT_RES' 
                
                if teensy_pwm_freq != f2pwm.PWM_FREQ:
                    self.get_logger().info('one or both of the teensy pwm freq reg values are not equal to f2pwm.PWM_FREQ')
                assert teensy_t200_init == f2pwm.T200_INIT, 'one or both of the teensy t200 init reg values are not equal to f2pwm.T200_INIT' 

    def subscriber_callback(self, msg, thruster_idx):
        thruster_force_newtons = msg.data
        pwm_duty_cycle_val = f2pwm.to_duty_cycle(thruster_force_newtons)

        # writes to teensy 0 for thrusters 0-3, teensy 1 for thrusters 4-7
        teensy.write_i2c_16(
            teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)], 
            teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)], 
            pwm_duty_cycle_val
        )

def main():
    rclpy.init()

    barracuda_thrusters = BarracudaThrusters()

    rclpy.spin(barracuda_thrusters)

    barracuda_thrusters.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
