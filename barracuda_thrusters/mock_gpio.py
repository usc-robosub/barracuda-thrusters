import logging

class MockGPIO:
    # Mock constants
    BCM = "BCM"
    IN = "IN"
    PUD_UP = "PUD_UP"
    LOW = 0
    HIGH = 1
    BOTH = "BOTH"
    
    @staticmethod
    def setmode(mode): 
        pass
        
    @staticmethod
    def setup(pin, mode, pull_up_down=None): 
        pass
        
    @staticmethod
    def input(pin): 
        return MockGPIO.HIGH 
        
    @staticmethod
    def add_event_detect(pin, edge, callback=None, bouncetime=None): 
        pass
        
    @staticmethod
    def cleanup(): 
        pass

logging.getLogger("barracuda_thrusters").warning("Loaded Mock GPIO for local testing.")