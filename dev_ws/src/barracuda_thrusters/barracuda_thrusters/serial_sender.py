# import RPi.GPIO as GPIO
from smbus import *
import struct
from collections import namedtuple

class SerialSender:
    ThrusterLoc = namedtuple("ThrusterLoc", ["i2c_address", "register"])

    def __init__(self, n_thrusters):
        self.thrusters = {
            0: SerialSender.ThrusterLoc(0x2D, 0),
            1: SerialSender.ThrusterLoc(0x2D, 2),
            2: SerialSender.ThrusterLoc(0x2D, 4),
            3: SerialSender.ThrusterLoc(0x2D, 6),
            4: SerialSender.ThrusterLoc(0x2E, 0),
            5: SerialSender.ThrusterLoc(0x2E, 2),
            6: SerialSender.ThrusterLoc(0x2E, 4),
            7: SerialSender.ThrusterLoc(0x2E, 6),
        }
        assert n_thrusters == len(self.thrusters), 'n_thrusters must equal length of the thruster organization dict'
    
    def send():
        print("sending")