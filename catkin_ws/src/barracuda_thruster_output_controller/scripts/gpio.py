import rospy
import RPi.GPIO as GPIO
# from barracuda_thruster_output_controller.srv import ReinitThrusters
from geometry_msgs.msg import Wrench
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
import time


KILLSWITCH_STATUS_INPUT_PIN = 8 # Right side 12 down
LED_OUTPUT_PIN = 22

def gpio_node():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(KILLSWITCH_STATUS_INPUT_PIN, GPIO.IN)
        GPIO.setup(LED_OUTPUT_PIN, GPIO.OUT)
        # rospy.wait_for_service('reinit_thrusters')
        # print('got service')
        # reinit_thrusters = rospy.ServiceProxy('reinit_thrusters', ReinitThrusters)
        # GPIO.add_event_detect(KILLSWITCH_STATUS_INPUT_PIN, GPIO.FALLING, callback=dummy_callback)
        
        rospy.init_node('gpio')
        rospy.on_shutdown(shutdown_callback)

        publishers = []
        for i in range(8):
            topic = f"thrusters/{i}/input"
            pub = rospy.Publisher(topic, FloatStamped, queue_size=10)
            publishers.append(pub)

        # input('input to test reinit\n')
        # reinit_thrusters()


        rate = rospy.Rate(100)  # 100 Hz = 10ms polling
        prev_killswitch_status = GPIO.input(KILLSWITCH_STATUS_INPUT_PIN)
        while not rospy.is_shutdown():
            # GPIO.output(LED_OUTPUT_PIN, GPIO.input(KILLSWITCH_STATUS_INPUT_PIN))
            cur_killswitch_status = GPIO.input(KILLSWITCH_STATUS_INPUT_PIN)
            # print(cur_killswitch_status)
            if prev_killswitch_status == False and cur_killswitch_status == True:
                print("HI-->LO EDGE\n")
                
                for i in range(8):
                    msg = FloatStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.data = 0
                    publishers[i].publish(msg)
                # Publish target wrench
                pub = rospy.Publisher("thruster_manager/input", Wrench, queue_size=10)
                wrench_msg = Wrench()

                wrench_msg.force.x = 10.0 
                wrench_msg.force.y = 0.0
                wrench_msg.force.z = -2.0

                wrench_msg.torque.x = 0.0  
                wrench_msg.torque.y = 0.0
                wrench_msg.torque.z = 0.0
                pub.publish(wrench_msg)

                time.sleep(8)

                # stop thrusters
                for i in range(8):
                    msg = FloatStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.data = 0
                    publishers[i].publish(msg)

                

            prev_killswitch_status = cur_killswitch_status
            rate.sleep()
    finally:
        GPIO.cleanup()

def shutdown_callback():
    GPIO.cleanup()
    print('GPIO cleaned up')



if __name__ == '__main__':
    gpio_node()