#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from barracuda_thruster_output_controller.srv import DisableThrusters, EnableThrusters

# once a certain treshold has been reached, send out a kill signal

DISABLE_PIN = 24

def disable_thrusters_handler():
    GPIO.output(DISABLE_PIN, GPIO.HIGH)

    return True

def enable_thrusters_handler():
    GPIO.output(DISABLE_PIN, GPIO.LOW)
    
    return True

def main():
    rospy.init_node('thruster_disable_node')

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DISABLE_PIN, GPIO.OUT)

    disable_service = rospy.service('disable_thruster', DisableThrusters, disable_thrusters_handler)
    enable_service = rospy.service('enable_thruster', EnableThrusters, enable_thrusters_handler)

    rospy.spin()


if __name__ == '__main__':
    main()