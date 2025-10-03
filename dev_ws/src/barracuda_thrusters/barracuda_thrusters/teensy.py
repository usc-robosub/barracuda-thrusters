from rclpy.logging import get_logger
from smbus import SMBus
import struct
from collections import namedtuple

logger = get_logger('Teensy')

i2c_addresses = [0x2d, 0x2e]

PWM_FREQ_REG = 8
PWM_BIT_RES_REG = 10
T200_INIT_REG = 12

thruster_registers = [0, 2, 4, 6]
    
def write_i2c_16(addr, reg, val):
    if GPIO is None or bus is None:   
        return
    
    logger.info(f'sending {val} to address {addr:02x}, reg {reg}')

    # H is for unsigned short (16-bit)
    data = list(struct.pack('<H', val))
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        logger.error(f'I2C write failed at addr {addr:#04x}, reg {reg}: {e}')


def read_i2c_16(addr, reg):
    if GPIO is None or bus is None:
        return None

        logger.info(f'reading from address {addr:02x}, reg {reg}')

    try:
        val = struct.unpack("<H", bytes(bus.read_i2c_block_data(addr, reg, 2)))[0]
        return val
    except Exception as e:
        logger.error(f'I2C read failed at addr {addr:#04x}, reg {reg}: {e}')
        return None

# run on module import
try:
    import RPi.GPIO as GPIO
except Exception as e:
    GPIO = None
    logger.warn(f'exception importing RPi.GPIO: {e}')

try:
    bus = SMBus(1)
except Exception as e:
    bus = None
    logger.warn(f'exception initializing i2c bus: {e}')