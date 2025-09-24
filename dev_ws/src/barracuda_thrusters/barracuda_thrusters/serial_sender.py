import RPi.GPIO as GPIO
from smbus import *
import struct
from collections import namedtuple

class SerialSender:
    ThrusterLoc = namedtuple("ThrusterLoc", ["i2c_address", "register"])

    def __init__(self):
        self.thrusters = {
            0: ThrusterLoc(0x2D, 0),
            1: ThrusterLoc(0x2D, 2),
            2: ThrusterLoc(0x2D, 4),
            3: ThrusterLoc(0x2D, 6),
            4: ThrusterLoc(0x2E, 0),
            5: ThrusterLoc(0x2E, 2),
            6: ThrusterLoc(0x2E, 4),
            7: ThrusterLoc(0x2E, 6),
        }
    
    def send():
        print("sending")