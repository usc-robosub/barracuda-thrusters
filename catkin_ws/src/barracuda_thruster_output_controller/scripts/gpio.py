import rospy
import RPi.GPIO as GPIO
from barracuda_thruster_output_controller.srv import ReinitThrusters


KILLSWITCH_STATUS_INPUT_PIN = -1
LED_OUTPUT_PIN = 22

def gpio_node():
    GPIO.setmode(GPIO.BCM)
    GPIO.setmode(KILLSWITCH_STATUS_INPUT_PIN, GPIO.IN)
    GPIO.setup(LED_OUTPUT_PIN, GPIO.OUT)
    GPIO.add_event_detect(KILLSWITCH_STATUS_INPUT_PIN, GPIO.FALLING, callback=reinit_thrusters)
    
    rospy.init_node('gpio')
    rospy.on_shutdown(shutdown_callback)
    rospy.wait_for_service('reinit_thrusters')
    try:
        reinit_thrusters = rospy.ServiceProxy('reinit_thrusters', ReinitThrusters)
        resp = reinit_thrusters()
    except rospy.ServiceException as e:
        print(f'Reinit thrusters service call failed: {e}')
    

    rate = rospy.Rate(100)  # 100 Hz = 10ms polling
    while not rospy.is_shutdown():
        GPIO.output(LED_OUTPUT_PIN, GPIO.input(KILLSWITCH_STATUS_INPUT_PIN))
        rate.sleep()

def shutdown_callback():
    GPIO.cleanup()
    print('GPIO cleaned up')



if __name__ == '__main__':
    gpio_node()