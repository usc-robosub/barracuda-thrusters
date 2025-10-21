import rclpy
from rclpy.node import Node
from rclpy.task import Future

from . import f2pwm
from . import teensy

from std_msgs.msg import Float32

import time

class BarracudaThrusters(Node):

    def __init__(self):
        super().__init__('barracuda_thrusters')

        # self.declare_parameter('n_thrusters', 8)
        # self.n_thrusters = self.get_parameter('n_thrusters').value
        self.n_thrusters = 8    
        self.verify_config_values()

        for thruster_idx in range(self.n_thrusters):
            self.reset_thruster(thruster_idx)
        
        future = Future()
        timer = self.create_timer(1.0,lambda:future.set_result(True))
        rclpy.spin_until_future_complete(self,future)
        timer.cancel()
        self.shake_thrusters()

        for thruster_idx in range(self.n_thrusters):
            self.reset_thruster(thruster_idx)
        

        for thruster_idx in range(self.n_thrusters):
            topic = f'thrusters/input{thruster_idx}'
            self.create_subscription(
                Float32, 
                topic, 
                lambda msg, thruster_idx=thruster_idx: self.subscriber_callback(msg, thruster_idx), 
                10
            )

    def verify_config_values(self):
        if teensy.GPIO is None:
            self.get_logger().info('teensy.GPIO is None: no I2C available')
            return
        for i2c_addr in teensy.i2c_addresses:
            teensy_pwm_freq = teensy.read_i2c_16(i2c_addr, teensy.PWM_FREQ_REG)
            teensy_pwm_bit_res = teensy.read_i2c_16(i2c_addr, teensy.PWM_BIT_RES_REG)
            teensy_t200_init = teensy.read_i2c_16(i2c_addr, teensy.T200_INIT_REG)

            if teensy_pwm_freq is None or teensy_pwm_bit_res is None or teensy_t200_init is None:
                self.get_logger().error(f'could not read from {i2c_addr:#04x}')
                continue

            if teensy_pwm_freq != f2pwm.PWM_FREQ:
                self.get_logger().warn(f'PWM freq at {i2c_addr:#04x} not equal to f2pwm.PWM_FREQ')
            if teensy_pwm_bit_res != f2pwm.PWM_BIT_RES:
                self.get_logger().warn(f'PWM bit res at {i2c_addr:#04x} not equal to f2pwm.PWM_BIT_RES')                
            if teensy_t200_init != f2pwm.T200_INIT:
                self.get_logger().warn(f'PWM T200 init at {i2c_addr:#04x} not equal to f2pwm.T200_INIT')
               
    # sends the duty cycle val for 1500us (stopped pwm width) to all thrusters - this inits/stops the thrusters
    def reset_thruster(self, thruster_idx):
            self._write_to_thruster_reg(thruster_idx, f2pwm.to_duty_cycle(force_newtons=0))
    
    def shake_thrusters(self):
        for thruster_idx in range(self.n_thrusters):
            #thruster_idx = thruster_idx
            shake_force = 5.0
        # msg.data = shake_force
        # self.thruster_pubs[thruster_idx].publish(msg)

            self.get_logger().info(f"shaking thruster {thruster_idx}, data: {shake_force}")

            self._write_to_thruster_reg(thruster_idx, f2pwm.to_duty_cycle(force_newtons=shake_force))
        future = Future()
        timer = self.create_timer(3.0,lambda:future.set_result(True))
        rclpy.spin_until_future_complete(self,future)
        timer.cancel()
            

    def subscriber_callback(self, msg, thruster_idx):
        thruster_force_newtons = msg.data
        pwm_duty_cycle_val = f2pwm.to_duty_cycle(thruster_force_newtons)
        self._write_to_thruster_reg(thruster_idx, pwm_duty_cycle_val)
    
    # helper function to write a value to a thruster register on the teensy
    def _write_to_thruster_reg(self, thruster_idx, val):
        try:
            # writes to teensy 0 for thrusters 0-3, teensy 1 for thrusters 4-7
            teensy.write_i2c_16(
                teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)], 
                teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)], 
                val
            )
        except Exception as e:  
            self.get_logger().warning(
                f"Write failed at addr {teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)]:#04x}, reg {teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)]}: {e}"
            )

def main():
    rclpy.init()

    barracuda_thrusters = BarracudaThrusters()

    rclpy.spin(barracuda_thrusters)

    barracuda_thrusters.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
