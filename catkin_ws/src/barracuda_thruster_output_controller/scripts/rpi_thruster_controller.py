#! /usr/bin/env python3

# 1) subscribe to thrusters/i/input topics to get force value for each thruster (in same force unit as in a wrench message)
# 2) convert force values (assuming it's in Newtons) to kgF
# 3) determine pulse widths (in microseconds) to send to each of the thrusters using kgF to pulse width (us) conversion spreadsheet
# 4) determine duty cycle value given the bit granularity & frequency being used by analogWrite function on the Teensy (e.g. 256-bit, 333Hz frequency)
# 5) send a message to appropriate register on appropriate teensy

import rospkg
import csv
import os
from bisect import bisect_left

NEWTONS_PER_KGF = 9.80665
STOPPED_PWM_WIDTH = 1500

class F2PWM:
    def __init__(self, csv_filename, interpolation_mode):
        rospack = rospkg.RosPack()
        csv_path = os.path.join(rospack.get_path('barracuda_thruster_output_controller'), 'data', csv_filename)
        self.pwm_widths = []
        self.force_vals = []
        self._create_lut_lists(csv_path)
    
    # convert force (newtons) to PWM width (microseconds)
    def to_us(self, force_newtons):
        if force_newtons == 0:
            return STOPPED_PWM_WIDTH
        
        # Find index of leftmost value greater than or equal to force_newtons
        lut_idx = bisect_left(self.force_vals, force_newtons)
        if lut_idx == 0:
            raise ValueError("less than lowest allowed force")
        if lut_idx == len(self.force_vals):
            raise ValueError("greater than highest allowed force")

        # Get the surrounding values for interpolation
        force_under = self.force_vals[lut_idx - 1]
        force_over = self.force_vals[lut_idx]
        pwm_width_under = self.pwm_widths[lut_idx - 1]
        pwm_width_over = self.pwm_widths[lut_idx]
        
        # Linear interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        pwm_us_interpolated = pwm_width_under + (force_newtons - force_under) * ((pwm_width_over - pwm_width_under) / (force_over - force_under))
        return pwm_us_interpolated

    # creates lookup table between force values (N) and PWM widths (microseconds)
    def _create_lut_lists(self, csv_path):
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                pwm_width, force_newtons = int(row['PWM (µs)']), float(row['Force (N)'])
                self.pwm_widths.append(pwm_width)
                self.force_vals.append(force_newtons)
                # print(pwm_width, force_newtons)

############################################################################################################################################################


from smbus import *
import struct
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
# from F2PWM import F2PWM
from collections import namedtuple
import RPi.GPIO as GPIO


# TODO: set up thrust config in config dir/use parameters 
ThrusterConfig = namedtuple('ThrusterConfig', ['i2c_address', 'register'])
thruster_organization = {
    0: ThrusterConfig(0x2d, 0),
    1: ThrusterConfig(0x2d, 2),
    2: ThrusterConfig(0x2d, 4),
    3: ThrusterConfig(0x2d, 6),
    4: ThrusterConfig(0x2e, 0),
    5: ThrusterConfig(0x2e, 2),
    6: ThrusterConfig(0x2e, 4),
    7: ThrusterConfig(0x2e, 6)
}

TMP_KILLSWITCH_PIN = 22

# thruster_data_handler = ThrusterDataHandler()
converter = F2PWM('t200_18v_data.csv', 'interpolation mode placeholder')

try:
    bus = SMBus(1)
except Exception as err:
    print(err)

# TODO: make these ros parameters
# configuration values for teensy 
pwm_frequency = 333 # 
pwm_bit_resolution = 8 

def on_recv_killswitch(msg):
    if msg.data == 0:
        # print("should kill now")
        GPIO.output(TMP_KILLSWITCH_PIN, GPIO.LOW) 
    if msg.data == 1: 
        GPIO.output(TMP_KILLSWITCH_PIN, GPIO.HIGH)

    
def on_recv_thruster_force(msg, thruster_id):
    pwm_us = converter.to_us(msg.data)
    # pwm_us / pwm_period = pwm_us * pwm_frequency
    # need to divide pwm_us * pwm_frequency by 10^6 to account for us/s difference, 
    # dividing by 10^3 twice to keep intermediate values smaller
    duty_cycle_val = int(round(((pwm_us / 10**3) * (pwm_frequency / 10**3)) * (2**pwm_bit_resolution)))

    if thruster_id == 0:
        # print(f'sending duty cycle val {duty_cycle_val} to thruster 0')
    send_duty_cycle_val_to_thruster(duty_cycle_val, thruster_id)

def shutdown_callback():
    GPIO.cleanup()
    
def thruster_controller_node():
    
    rospy.init_node('barracuda_thruster_output_controller')
    rospy.on_shutdown(shutdown_callback)
    # Create subscribers for each thruster
    try:
        for i in range(8):
            topic = f"thrusters/{i}/input"
            rospy.Subscriber(topic, FloatStamped, on_recv_thruster_force, callback_args=i)
        rospy.Subscriber("/killswitch", FloatStamped, on_recv_killswitch)   
    except Exception as e:
        print(e)
    
    rospy.spin()   
    
# Helper functions
def send_duty_cycle_val_to_thruster(duty_cycle_val, thruster_id):
    i2c_address = thruster_organization[thruster_id].i2c_address
    thruster_register = thruster_organization[thruster_id].register
    # H is for unsigned short (16-bit)
    data = list(struct.pack('<H', duty_cycle_val))
    try:
        bus.write_i2c_block_data(i2c_address, thruster_register, data)
    except Exception as e:
        print(f"Error writing to target: {e}")
        print(i2c_address, thruster_register)
        print("Check that the wiring is correct and you're using the correct pins.")
        exit()


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TMP_KILLSWITCH_PIN, GPIO.OUT)
    GPIO.output(TMP_KILLSWITCH_PIN, GPIO.HIGH)
    thruster_controller_node()