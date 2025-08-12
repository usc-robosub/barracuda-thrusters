#! /usr/bin/env python3

# 1) subscribe to thrusters/i/input topics to get force value for each thruster (in same force unit as in a wrench message)
# 2) convert force values (assuming it's in Newtons) to kgF
# 3) determine pulse widths (in microseconds) to send to each of the thrusters using kgF to pulse width (us) conversion spreadsheet
# 4) determine duty cycle value given the bit granularity & frequency being used by analogWrite function on the Teensy (e.g. 256-bit, 333Hz frequency)
# 5) send a message to appropriate register on appropriate teensy

from smbus import *
import struct
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from F2PWM import F2PWM
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
        print("should kill now")
        GPIO.output(TMP_KILLSWITCH_PIN, GPIO.LOW)  

    
def on_recv_thruster_force(msg, thruster_id):
    pwm_us = converter.to_us(msg.data)
    # pwm_us / pwm_period = pwm_us * pwm_frequency
    # need to divide pwm_us * pwm_frequency by 10^6 to account for us/s difference, 
    # dividing by 10^3 twice to keep intermediate values smaller
    duty_cycle_val = int(round(((pwm_us / 10**3) * (pwm_frequency / 10**3)) * (2**pwm_bit_resolution)))
    send_duty_cycle_val_to_thruster(duty_cycle_val, thruster_id)
    
    
def thruster_controller_node():
    rospy.init_node('barracuda_thruster_output_controller')
    # Create subscribers for each thruster
    try:
        for i in range(8):
            topic = f"/thrusters/{i}/input"
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
    thruster_controller_node()