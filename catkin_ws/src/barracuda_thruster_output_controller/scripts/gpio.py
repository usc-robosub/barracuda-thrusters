import rospy
import RPi.GPIO as GPIO
from barracuda_thruster_output_controller.srv import ReinitThrusters
from geometry_msgs.msg import Wrench


KILLSWITCH_STATUS_INPUT_PIN = 8 # Right side 12 down
LED_OUTPUT_PIN = 22

def gpio_node():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(KILLSWITCH_STATUS_INPUT_PIN, GPIO.IN)
        GPIO.setup(LED_OUTPUT_PIN, GPIO.OUT)
        rospy.wait_for_service('reinit_thrusters')
        print('got service')
        reinit_thrusters = rospy.ServiceProxy('reinit_thrusters', ReinitThrusters)
        # GPIO.add_event_detect(KILLSWITCH_STATUS_INPUT_PIN, GPIO.FALLING, callback=dummy_callback)
        
        rospy.init_node('gpio')
        rospy.on_shutdown(shutdown_callback)

        # input('input to test reinit\n')
        reinit_thrusters()


        rate = rospy.Rate(100)  # 100 Hz = 10ms polling
        prev_killswitch_status = GPIO.input(KILLSWITCH_STATUS_INPUT_PIN)
        while not rospy.is_shutdown():
            GPIO.output(LED_OUTPUT_PIN, GPIO.input(KILLSWITCH_STATUS_INPUT_PIN))
            cur_killswitch_status = GPIO.input(KILLSWITCH_STATUS_INPUT_PIN)
            if prev_killswitch_status == False and cur_killswitch_status == True:
                print("HI-->LO EDGE\n")
                reinit_thrusters()
                # Publish target pose (or target wrench)
                # pub = rospy.Publisher("thruster_manager/input", Wrench, queue_size=10)
                # wrench_msg = Wrench()

                # wrench_msg.force.x = 10.0 
                # wrench_msg.force.y = 0.0
                # wrench_msg.force.z = -2.0

                # wrench_msg.torque.x = 0.0  
                # wrench_msg.torque.y = 0.0
                # wrench_msg.torque.z = 0.0
                # pub.publish(wrench_msg)

            prev_killswitch_status = cur_killswitch_status
            rate.sleep()
    finally:
        GPIO.cleanup()

def shutdown_callback():
    GPIO.cleanup()
    print('GPIO cleaned up')



if __name__ == '__main__':
    gpio_node()