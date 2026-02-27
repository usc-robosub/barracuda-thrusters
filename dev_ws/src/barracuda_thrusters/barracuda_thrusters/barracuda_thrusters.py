import rclpy
import numpy as np
import Jetson.GPIO as GPIO
from rclpy.node import Node
from sensor_msgs.msg import JointState

from . import teensy


class BarracudaThrusters(Node):
    def __init__(self):
        super().__init__("barracuda_thrusters")

        self.n_thrusters = 8

        self.cmd_thrust_subscription = self.create_subscription(
            JointState, "cmd_thrust", self.joint_state_subscriber_callback, 10
        )

        # killswitch gpio setup #
        #########################
        # gpiozero (pi library) defaults to BCM pin numbering. BCM 4 is Physical Pin 7.
        self.killswitch_pin = 4 

        try:
            # Set pin numbering mode to BCM
            GPIO.setmode(GPIO.BCM)
            
            # Setup pin as an input and enable the internal pull-up resistor.
            # This mimics gpiozero.Button's default behavior.
            GPIO.setup(self.killswitch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            def write_to_killswitch_regs(killed):
                self.get_logger().info(
                    f"killswitch signal is now {'lo' if killed == '0'.encode() else 'hi'}"
                )
                for addr in teensy.i2c_addresses:
                    teensy.write_i2c_char(addr, teensy.KILLSWITCH_REG, killed)

            # Callback function triggered by hardware interrupts
            def killswitch_callback(channel):
                # Because of the pull-up, LOW means the latch is closed ("pressed")
                if GPIO.input(self.killswitch_pin) == GPIO.LOW:
                    write_to_killswitch_regs("0".encode())
                else:
                    # HIGH means the latch is open ("released")
                    write_to_killswitch_regs("1".encode())

            # Check initial state on node startup
            if GPIO.input(self.killswitch_pin) == GPIO.LOW:
                write_to_killswitch_regs("0".encode())

            # Attach the event detection to listen for BOTH rising and falling edges.
            # bouncetime=50 adds a 50ms software debounce to ignore electrical noise 
            # from the physical killswitch contacts bouncing when flipped.
            GPIO.add_event_detect(
                self.killswitch_pin, 
                GPIO.BOTH, 
                callback=killswitch_callback, 
                bouncetime=50
            )

        except Exception as e:
            self.get_logger().warn(f"problem with gpio setup: {e}")

    def joint_state_subscriber_callback(self, msg):
        # teensy registers expect 32-bit floats 
        thruster_efforts = np.array(msg.effort, dtype=np.float32)

        for thruster_idx in range(self.n_thrusters):
            # writes to teensy 0 for thrusters 0-3, teensy 1 for thrusters 4-7
            try:
                teensy.write_i2c_float(
                    teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)],
                    teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)],
                    thruster_efforts[thruster_idx],
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Write failed at addr {teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)]:#04x}, reg {teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)]}: {e}"
                )

    def destroy_node(self):
        # Ensure GPIO pins are released when the node is gracefully shut down
        try:
            GPIO.cleanup()
        except Exception as e:
            self.get_logger().warn(f"Failed to clean up GPIO: {e}")
        super().destroy_node()


def main():
    rclpy.init()

    barracuda_thrusters = BarracudaThrusters()

    rclpy.spin(barracuda_thrusters)

    barracuda_thrusters.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
