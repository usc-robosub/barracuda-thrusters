#! /usr/bin/env python3

# 1) subscribe to thrusters/i/input topics to get force value for each thruster (in same force unit as in a wrench message)
# 2) convert force values (assuming it's in Newtons) to kgF
# 3) determine pulse widths (in microseconds) to send to each of the thrusters using kgF to pulse width (us) conversion spreadsheet
# 4) determine duty cycle value given the bit granularity & frequency being used by analogWrite function on the Teensy (e.g. 256-bit, 333Hz frequency)
# 5) send a message to appropriate register on appropriate teensy

from smbus2 import *
import struct
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
# from thruster_data_handler import ThrusterDataHandler
from collections import namedtuple
import RPi.GPIO as GPIO
# from barracuda_thruster_msgs.srv import SetThrustZero


import os
import sys
import numpy as np
import pandas as pd
import rospkg

class ThrusterDataHandler:
    """
    Handler for T200 thruster data that provides conversion between kgf force and PWM values.
    """
    
    def __init__(self, spreadsheet_path=None):
        """
        Initialize the handler with the path to the T200 performance data spreadsheet.
        
        Args:
            spreadsheet_path (str): Path to the Excel spreadsheet with thruster data.
                                   If None, attempts to find it in the package's data directory.
        """
        if spreadsheet_path is None:
            # Get the package path using ROS package utilities
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('barracuda_thruster_output_controller') 
            
            # Construct path to the data file (in the data directory)
            spreadsheet_path = os.path.join(package_path, 'data', 
                                           'T200-Public-Performance-Data-10-20V-September-2019.xlsx')
        
        self.spreadsheet_path = spreadsheet_path
        
        # Load and process the data from the Excel file
        self._load_data()
    
    def _load_data(self):
        """
        Load kgf and PWM data from the 18V sheet in the Excel spreadsheet.
        """
        try:   
            df = pd.read_excel(self.spreadsheet_path, sheet_name="18 V")
            
            self.kgf_values = df[" Force (Kg f)"].values
            self.pwm_values = df[" PWM (µs)"].values
            
        except Exception as e:
            raise Exception(f"Error loading thruster data: {e}")
    
    def kgf_to_pwm_us(self, kgf_input):
        """
        Convert kgf force value to PWM (μs) value using linear interpolation.
        
        Args:
            kgf_input (float): The input force in kgf to convert
            
        Returns:
            float: The corresponding PWM value in microseconds (μs)
        
        Raises:
            ValueError: If the input kgf value is outside the range of the available data
        """
        
        # Special case: return 1500us if force is zero (at the center of the range of pulse widths that translate to zero force)
        if (kgf_input == 0):
            return 1500
        
        # Check if the exact value exists in our dataset
        if kgf_input in self.kgf_values:
            index = np.where(self.kgf_values == kgf_input)[0][0]
            return self.pwm_values[index]
        
        # Use binary search to find where the input value would fit
        index = np.searchsorted(self.kgf_values, kgf_input)
        
        # Handle edge cases where input is outside data range
        if index == 0:
            raise ValueError(f"Input value {kgf_input} kgf is below the minimum value in the dataset ({self.kgf_values[0]} kgf)")
        elif index == len(self.kgf_values):
            raise ValueError(f"Input value {kgf_input} kgf is above the maximum value in the dataset ({self.kgf_values[-1]} kgf)")
        
        # Get the surrounding values for interpolation
        kgf_low = self.kgf_values[index - 1]
        kgf_high = self.kgf_values[index]
        pwm_low = self.pwm_values[index - 1]
        pwm_high = self.pwm_values[index]
        
        # Linear interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        pwm_us_interpolated = pwm_low + (kgf_input - kgf_low) * ((pwm_high - pwm_low) / (kgf_high - kgf_low))
        
        return pwm_us_interpolated

if __name__ == "__main__":
    try:
        handler = ThrusterDataHandler()
        
        # Test with a sample kgf value given as a command line arg, value defaults to 0 if no arg/there's an issue with the arg
        try:
            kgf_test = float(sys.argv[1])
        except (ValueError, IndexError):
            kgf_test = 0
        pwm_result = handler.kgf_to_pwm_us(kgf_test)
        
        print(f"For {kgf_test} kgf at 18V, the PWM value is {pwm_result:.2f} μs")
    except Exception as e:
        print(f"Error: {e}")







DISABLE_PIN = 24

# TODO: set up thrust config in config dir/use parameters 
ThrusterConfig = namedtuple('ThrusterConfig', ['i2c_address', 'register'])
thruster_organization = {
    0: ThrusterConfig(0x2d, 0),
    1: ThrusterConfig(0x2d, 2),
    2: ThrusterConfig(0x2d, 4),
    3: ThrusterConfig(0x2d, 6),
    4: ThrusterConfig(0x2d, 0),
    5: ThrusterConfig(0x2d, 2),
    6: ThrusterConfig(0x2d, 4),
    7: ThrusterConfig(0x2d, 6)
}

thruster_data_handler = ThrusterDataHandler()
bus = SMBus(1)

# TODO: make these ros parameters
# configuration values for teensy 
pwm_frequency = 333 # 
pwm_bit_resolution = 8 
    
def on_recv_thruster_kgf(msg, thruster_id):
    pwm_us = thruster_data_handler.kgf_to_pwm_us(msg.data * 0.1019716) # netwon to kgf conversion
    
    # pwm_us / pwm_period = pwm_us * pwm_frequency
    # need to divide pwm_us * pwm_frequency by 10^6 to account for us/s difference, 
    # dividing by 10^3 twice to keep intermediate values smaller
    duty_cycle_val = int(round(((pwm_us / 10**3) * (pwm_frequency / 10**3)) * (2**pwm_bit_resolution)))
    # print(f"received kgf value of: {msg.data} for thruster {thruster_id}, wrote duty cycle val: {duty_cycle_val}")
    
    send_duty_cycle_val_to_thruster(duty_cycle_val, thruster_id)
    
# software kill switch service to stop thrusters
def set_thruster_zero(enable_thrust_zero):
    rospy.wait_for_service("set_thrust_zero")
    try:
        set_thruster_zero_srv = rospy.ServiceProxy("set_thrust_zero", SetThrustZero)
        resp = set_thruster_zero_srv(enable_thrust_zero) 
        if resp.success:
            print(f"Service call succeeded: {resp.message}")
        else:
            print(f"Service call failed: {resp.message}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def check_disable_pin(event):
    global past_enable, enable
    past_enable = enable
    enable = GPIO.input(DISABLE_PIN)

    # via edge detection, if the pin goes from low to high, rising edge, we disable the thrusters
    if(enable == GPIO.HIGH and past_enable == GPIO.LOW):
        set_thruster_zero(True)    

    # I think the current plan was to not have the thrusters be re-enabled, but if we want to re-enable them, we can uncommenting the next line
    #elif enable == GPIO.LOW and past_enable == GPIO.HIGH:
    #    set_thruster_zero(False)  # Falling edge (re-enable)
    
def thruster_controller_node():

    global past_enable, enable
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(DISABLE_PIN, GPIO.IN)
    enable = GPIO.input(DISABLE_PIN)
    past_enable = enable

    rospy.init_node('barracuda_thruster_output_controller')
    print("starting barracuda thruster output controller")

    # set_thruster_zero(False)  # enable thrusters by default

    

    # Create subscribers for each thruster
    try:
        for i in range(8):
            topic = f"/thrusters/{i}/input"
            rospy.Subscriber(topic, FloatStamped, on_recv_thruster_kgf, callback_args=i)   
    except Exception as e:
        print(e)
    
    rospy.Timer(rospy.Duration(0.1), check_disable_pin)
    rospy.spin()   
    
# Helper functions

def send_duty_cycle_val_to_thruster(duty_cycle_val, thruster_id):
    i2c_address = thruster_organization[thruster_id].i2c_address
    thruster_register = thruster_organization[thruster_id].register
    write_int16(bus, i2c_address, thruster_register, duty_cycle_val)

# Writes 2 bytes at a time for uint16_t duty cycle val (or n bytes at a time)
def write_int16(bus, address, register, value):
    """Write a 16-bit integer to a register"""
    data = list(struct.pack('<H', value))  # H is for unsigned short (16-bit)
    safe_write_block(bus, address, register, data)

def safe_write_block(bus, address, register, values):
    try:
        bus.write_i2c_block_data(address, register, values)
    except Exception as e:
        print(f"Error writing to target: {e}")
        print("Check that the wiring is correct and you're using the correct pins.")

def safe_write_byte(bus, address, register, value):
    try:
        bus.write_byte_data(address, register, value)
    except:
        print("Error writing to target.\nCheck that the wiring is correct and you're using the correct pins.")

if __name__ == '__main__':
    thruster_controller_node()
