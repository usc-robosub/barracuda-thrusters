from rclpy.logging import get_logger
from smbus import SMBus
import struct
from collections import namedtuple

logger = get_logger('Teensy')

i2c_addresses = [0x2d, 0x2e]

PWM_FREQ_REG = 0
PWM_BIT_RES_REG = 2
T200_INIT_REG = 4

thruster_registers = [6, 8, 10, 12]
    
def write_i2c_16(addr, reg, val):
    if GPIO is None or bus is None:
        logger.info(f'sending {val} to address {addr:02x}, reg {reg}')
        return
    
    assert bus is not None, 'self.bus is None (it should not be if RPi.GPIO was imported)'

    # H is for unsigned short (16-bit)
    data = list(struct.pack('<H', val))
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        print(f"Error writing to target: {e}")
        print(addr, reg)
        print("Check that the wiring is correct and you're using the correct pins.")
        exit()


def read_i2c_16(addr, reg):
    if GPIO is None or bus is None:
        logger.info(f'reading from address {addr:02x}, reg {reg}')
        return None
    try:
        data = bus.read_i2c_block_data(addr, reg, 2)
    except Exception as e:
        print(f"Error writing to target: {e}")
        print(addr, reg)
        print("Check that the wiring is correct and you're using the correct pins.")
        exit()
    return data

# run on module import
try:
    import RPi.GPIO as GPIO
except Exception as e:
    GPIO = None
    print(f'exception importing RPi.GPIO: {e}\nwill print instead of sending over serial')

try:
    bus = SMBus(1)
except Exception as e:
    bus = None
    print(f'exception initializing i2c bus: {e}\nwill print instead of sending over serial')