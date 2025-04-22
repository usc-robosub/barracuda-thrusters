#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from barracuda_thruster_output_controller.srv import DisableThrusters, EnableThrusters

# once a certain treshold has been reached, send out a kill signal

DISABLE_PIN = 24

def thrusters_control_handler(req):
    if req.status:
        GPIO.output(DISABLE_PIN, GPIO.HIGH)
    else
        GPIO.output(DISABLE_PIN, GPIO.LOW)


def main():
    rospy.init_node('thruster_disable_node')

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DISABLE_PIN, GPIO.OUT)

    thruster_service = rospy.service('thrustcontr', SetThruster, thrusters_control_handler)
     
    rospy.spin()


if __name__ == '__main__':
    main()