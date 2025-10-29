import rclpy
from rclpy.node import Node

from . import teensy

from std_msgs.msg import Float32

from gpiozero import Button

class BarracudaThrusters(Node):

    def __init__(self):
        super().__init__('barracuda_thrusters')

        # self.declare_parameter('n_thrusters', 8)
        # self.n_thrusters = self.get_parameter('n_thrusters').value
        self.n_thrusters = 8    

        for thruster_idx in range(self.n_thrusters):
            topic = f'thrusters/input{thruster_idx}'
            self.create_subscription(
                Float32, 
                topic, 
                lambda msg, thruster_idx=thruster_idx: self.subscriber_callback(msg, thruster_idx), 
                10
            )

        # TODO: read killswitch gpio pin and send to teensy
        # killswitch pin hi: latch is closed, killed = 0
        # killswitch pin lo: latch is open, killed = 1 
        killswitch_pin = Button(4)
        def write_to_killswitch_regs(killed):
            for addr in teensy.i2c_addresses: teensy.write_i2c_char(addr, teensy.KILLSWITCH_REG, killed)
        if killswitch_pin.is_pressed:
            write_to_killswitch_regs(0)
        killswitch_pin.when_pressed = lambda: write_to_killswitch_regs(0)
        killswitch_pin.when_released = lambda: write_to_killswitch_regs(1)

    def subscriber_callback(self, msg, thruster_idx):
        thruster_force_newtons = msg.data

        # writes to teensy 0 for thrusters 0-3, teensy 1 for thrusters 4-7
        try:
            teensy.write_i2c_16(
                teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)], 
                teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)], 
                thruster_force_newtons
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
