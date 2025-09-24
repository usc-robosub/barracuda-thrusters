from smbus import *
import struct
from collections import namedtuple

class SerialSender:
    ThrusterLoc = namedtuple("ThrusterLoc", ["i2c_address", "register"])

    def __init__(self, n_thrusters):
        self.serial_enabled = True
        try:
            import RPi.GPIO as GPIO
        except Exception as e:
            self.serial_enabled = False
            print(f'exception importing RPi.GPIO: {e}\nwill print instead of sending over serial')
        
        try:
            self.bus = SMBus(1)
        except Exception as e:
            self.bus = False
            self.serial_enabled = False
            print(f'exception initializing i2c bus: {e}\nwill print instead of sending over serial')
        
        self.thrusters = {
            0: SerialSender.ThrusterLoc(0x2D, 0),
            1: SerialSender.ThrusterLoc(0x2D, 2),
            2: SerialSender.ThrusterLoc(0x2D, 4),
            3: SerialSender.ThrusterLoc(0x2D, 6),
            # 4: SerialSender.ThrusterLoc(0x2E, 0),
            # 5: SerialSender.ThrusterLoc(0x2E, 2),
            # 6: SerialSender.ThrusterLoc(0x2E, 4),
            # 7: SerialSender.ThrusterLoc(0x2E, 6),
        }
        assert n_thrusters == len(self.thrusters), 'n_thrusters must equal length of the thruster organization dict'
    
    def send(self, pwm_duty_cycle_val, thruster_idx):
        print(f'sending {pwm_duty_cycle_val} to thruster {thruster_idx}')
        if not self.serial_enabled:
            return

        assert self.bus is not False, 'self.bus is False (it should not be if self.serial_enabled is true)'
        i2c_address = self.thrusters[thruster_idx].i2c_address
        thruster_register = self.thrusters[thruster_idx].register
        # H is for unsigned short (16-bit)
        data = list(struct.pack('<H', duty_cycle_val))
        try:
            self.bus.write_i2c_block_data(i2c_address, thruster_register, data)
        except Exception as e:
            print(f"Error writing to target: {e}")
            print(i2c_address, thruster_register)
            print("Check that the wiring is correct and you're using the correct pins.")
            exit()