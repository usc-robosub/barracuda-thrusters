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
from thruster_data_handler import ThrusterDataHandler
from collections import namedtuple
import RPi.GPIO as GPIO
from barracuda_thruster_msgs.srv import SetThrustZero

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
pwm_frequency = 400 # period = 2500 µs --> frequency = 400 Hz
pwm_bit_resolution = 15 # highest bit resolution allowed for pwm signals on teensy
    
def on_recv_thruster_kgf(msg, thruster_id):
    pwm_us = thruster_data_handler.kgf_to_pwm_us(msg.data)
    
    # pwm_us / pwm_period = pwm_us * pwm_frequency
    # need to divide pwm_us * pwm_frequency by 10^6 to account for us/s difference, 
    # dividing by 10^3 twice to keep intermediate values smaller
    duty_cycle_val = int(round(((pwm_us / 10**3) * (pwm_frequency / 10**3)) * (2**pwm_bit_resolution)))
    print(f"received kgf value of: {msg.data} for thruster {thruster_id}, wrote duty cycle val: {duty_cycle_val}")
    
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
    enable = GPIO.input(DISABLE_PIN)
    past_enable = enable

    rospy.init_node('barracuda_thruster_output_controller')

    set_thruster_zero(False)  # enable thrusters by default

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DISABLE_PIN, GPIO.IN)

    # Create subscribers for each thruster
    for i in range(8):
        topic = f"/thrusters/{i}/input"
        rospy.Subscriber(topic, FloatStamped, on_recv_thruster_kgf, callback_args=i)   
    
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





