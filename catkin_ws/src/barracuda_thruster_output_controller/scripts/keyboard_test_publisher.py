import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from sshkeyboard import listen_keyboard
import threading
from collections import namedtuple
import time
import os

killswitch_val = 1

thruster_forces = [0] * 8

MIN_THRUSTER_FORCE = -10 # newtons
MAX_THRUSTER_FORCE = 10 # newtons

THRUSTER_INDICES = [4, 5, 6 ,7]
cur_thruster_idx = THRUSTER_INDICES[0]

FORCE_INCREMENT = 1

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

def start_keyboard_thread():
    try:
        keyboard_thread = threading.Thread(target=start_keyboard_listener)
        keyboard_thread.daemon = True
        keyboard_thread.start()
    except:
        print("error starting keyboard thread")
        exit(1)

def start_keyboard_listener():
    listen_keyboard(on_press=press, until=None)

def press(key):
    global cur_thruster_idx, thruster_forces, killswitch_val

    print(key)
    if key == 'k':
        if thruster_forces[cur_thruster_idx] - FORCE_INCREMENT >= MIN_THRUSTER_FORCE:
            thruster_forces[cur_thruster_idx] -= FORCE_INCREMENT
    if key == 'i':
        if thruster_forces[cur_thruster_idx] + FORCE_INCREMENT <= MAX_THRUSTER_FORCE:
            thruster_forces[cur_thruster_idx] += FORCE_INCREMENT
    if key == 'r':
        for i in THRUSTER_INDICES:
            thruster_forces[i - THRUSTER_INDICES[0]] = 0
    if key == 'd':
        killswitch_val = 0
    if key == 'q':
        os.system("stty sane")
        rospy.signal_shutdown("program exited")
    
    if f'{THRUSTER_INDICES[0]}' <= key <= f'{THRUSTER_INDICES[-1]}':
        cur_thruster_idx = int(key)
        print(f"cur_thruster_idx={cur_thruster_idx}")
        cur_address = thruster_organization[cur_thruster_idx].i2c_address
        print(f"cur_address={cur_address}")
        cur_register = thruster_organization[cur_thruster_idx].register
        print(f"cur_register={cur_register}")

    print(f"thruster {cur_thruster_idx} force: {thruster_forces[cur_thruster_idx]}")


def keyboard_test_publisher_node():
    global thruster_forces, killswitch_val
    print("starting keyboard test publisher")
    rospy.init_node('thruster_test_publisher')

    # Create publishers for each thruster
    publishers = []
    for i in THRUSTER_INDICES:
        topic = f"/thrusters/{i}/input"
        pub = rospy.Publisher(topic, FloatStamped, queue_size=10)
        publishers.append(pub)
    sw_killswitch_pub = rospy.Publisher("/killswitch", FloatStamped, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        for i in THRUSTER_INDICES:
            msg = FloatStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = thruster_forces[i]
            publishers[i - THRUSTER_INDICES[0]].publish(msg) 
        msg2 = FloatStamped()
        msg2.header.stamp = rospy.Time.now()
        msg2.data = killswitch_val
        sw_killswitch_pub.publish(msg2) 


if __name__ == '__main__':
    start_keyboard_thread()
    keyboard_test_publisher_node()