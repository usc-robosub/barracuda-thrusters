from rclpy.logging import get_logger
import struct

logger = get_logger('Teensy')

i2c_addresses = [0x2d, 0x2e]

KILLSWITCH_REG = 16

thruster_registers = [0, 4, 8, 12]
    
def write_i2c_float(addr, reg, val):
    if bus is None:   
        return

    # f is for float (32-bit)
    # e is for float (16-bit)
    data = list(struct.pack('<f', val))

    logger.info(f'sending {round(val, 8)} (bytes={data}) to address {addr:02x}, reg {reg}')
    
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        logger.error(f'I2C float write failed at addr {addr:#04x}, reg {reg}: {e}')

def write_i2c_char(addr, reg, val):
    if bus is None:   
        return
    
    logger.info(f'sending {val} to address {addr:02x}, reg {reg}')

    # c is for float (8-bit)
    data = list(struct.pack('<c', val))
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        logger.error(f'I2C char write failed at addr {addr:#04x}, reg {reg}: {e}')


def read_i2c_char(addr, reg):
    if bus is None:
        return None

    logger.info(f'reading from address {addr:02x}, reg {reg}')

    try:
        val = struct.unpack("<c", bytes(bus.read_i2c_block_data(addr, reg, 1)))[0]
        return val
    except Exception as e:
        logger.error(f'I2C char read failed at addr {addr:#04x}, reg {reg}: {e}')
        return None

# --- INITIALIZE HARDWARE OR FALLBACK TO MOCK ---
try:
    # on RPI: pins 3 and 5 map to I2C Bus 1
    # on jetson orin nano: pins 27 and 28 map to I2C Bus 1
    from smbus import SMBus
    bus = SMBus(1) 
except Exception as e:
    # catch both ImportError (no library) AND FileNotFoundError/OSError (no hardware)
    logger.warning(f'Physical I2C bus failed to initialize ({e}). Falling back to Mock SMBus.')
    from .mock_smbus import SMBus as MockSMBus
    bus = MockSMBus(1)
# -----------------------------------------------