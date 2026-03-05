from rclpy.logging import get_logger

logger = get_logger('MockSMBus')

class SMBus:
    def __init__(self, bus_number):
        self.bus_number = bus_number
        logger.info(f"Mock SMBus connected virtually to bus {self.bus_number}")
        
    def write_i2c_block_data(self, addr, cmd, vals):
        # We do nothing here; your existing logger.info in teensy.py will still print the attempt
        pass
        
    def read_i2c_block_data(self, addr, cmd, num=1):
        # Return an array of empty bytes so struct.unpack doesn't crash
        return [b'\x00'] * num

logger.warning("Loaded Mock SMBus for local testing.")