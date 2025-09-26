from rclpy.logging import get_logger
from smbus import SMBus
import struct
from collections import namedtuple

logger = get_logger('Serial')

RegLoc = namedtuple('RegLoc', ['i2c_addr', 'reg'])

ADDR0 = 0x2d
ADDR1 = 0x2e

_PWM_FREQ_REG = 8
_PWM_BIT_RES_REG = 10

thruster_regs = {
    0: RegLoc(ADDR0, 0),
    1: RegLoc(ADDR0, 2),
    2: RegLoc(ADDR0, 4),
    3: RegLoc(ADDR0, 6),
    4: RegLoc(ADDR1, 0),
    5: RegLoc(ADDR1, 2),
    6: RegLoc(ADDR1, 4),
    7: RegLoc(ADDR1, 6),
}

pwm_freq_reg = {
    ADDR0: RegLoc(ADDR0, _PWM_FREQ_REG),
    ADDR1: RegLoc(ADDR1, _PWM_FREQ_REG)
}

pwm_bit_res_reg = {
    ADDR0: RegLoc(ADDR0, _PWM_BIT_RES_REG),
    ADDR1: RegLoc(ADDR1, _PWM_BIT_RES_REG) 
}

def write_reg16(reg_loc: RegLoc, val):
    if GPIO is None or bus is None:
        logger.info(f'sending {val} to address {reg_loc.i2c_addr}, reg {reg_loc.reg}')
        return
    
    assert bus is not None, 'self.bus is None (it should not be if RPi.GPIO was imported)'

    # H is for unsigned short (16-bit)
    data = list(struct.pack('<H', val))
    try:
        bus.write_i2c_block_data(reg_loc.i2c_addr, reg_loc.reg, data)
    except Exception as e:
        print(f"Error writing to target: {e}")
        print(reg_loc.i2c_addr, reg_loc.reg)
        print("Check that the wiring is correct and you're using the correct pins.")
        exit()


def read_reg16(reg_loc: RegLoc):
    try:
        data = bus.read_i2c_block_data(reg_loc.i2c_addr, reg_loc.reg, 2)
    except Exception as e:
        print(f"Error writing to target: {e}")
        print(reg_loc.i2c_addr, reg_loc.reg)
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