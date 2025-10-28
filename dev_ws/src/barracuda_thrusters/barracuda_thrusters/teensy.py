from rclpy.logging import get_logger
from smbus import SMBus
import struct
from collections import namedtuple

logger = get_logger('Teensy')

i2c_addresses = [0x2d, 0x2e]

KILLSWITCH_REG = 16

thruster_registers = [0, 4, 8, 12]
    
def write_i2c_float(addr, reg, val):
    if GPIO is None or bus is None:   
        return
    
    logger.info(f'sending {val} to address {addr:02x}, reg {reg}')

    # f is for float (16-bit)
    data = list(struct.pack('<f', val))
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        logger.error(f'I2C float write failed at addr {addr:#04x}, reg {reg}: {e}')

def write_i2c_char(addr, reg, val):
    if GPIO is None or bus is None:   
        return
    
    logger.info(f'sending {val} to address {addr:02x}, reg {reg}')

    # c is for float (8-bit)
    data = list(struct.pack('<c', val))
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        logger.error(f'I2C char write failed at addr {addr:#04x}, reg {reg}: {e}')


def read_i2c_char(addr, reg):
    if GPIO is None or bus is None:
        return None

        logger.info(f'reading from address {addr:02x}, reg {reg}')

    try:
        val = struct.unpack("<c", bytes(bus.read_i2c_block_data(addr, reg, 2)))[0]
        return val
    except Exception as e:
        logger.error(f'I2C char read failed at addr {addr:#04x}, reg {reg}: {e}')
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