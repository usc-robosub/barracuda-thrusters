import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from sshkeyboard import listen_keyboard
import threading
from collections import namedtuple

thruster_forces = [0] * 8

cur_thruster_idx = 0

MIN_THRUSTER_FORCE = -10 # newtons
MAX_THRUSTER_FORCE = 10 # newtons

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
    global cur_thruster_idx, thruster_forces
    if key == 'k':
        if thruster_forces[cur_thruster_idx] - 2 >= MIN_THRUSTER_FORCE:
            thruster_forces[cur_thruster_idx] -= 2
    if key == 'i':
        if thruster_forces[cur_thruster_idx] + 2 <= MAX_THRUSTER_FORCE:
            thruster_forces[cur_thruster_idx] += 2
    if key == 'r':
        for i in range(8):
            thruster_forces[cur_thruster_idx] = 0
    
    if '0' <= key <= '7':
        cur_thruster_idxthruster_idx = int(key)
        print(f"cur_thruster_idx={cur_thruster_idx}")
        cur_address = thruster_organization[thruster_idx].i2c_address
        print(f"cur_address={cur_address}")
        cur_register = thruster_organization[thruster_idx].register
        print(f"cur_register={cur_register}")

def keyboard_test_publisher_node():
    global thruster_forces
    print("starting keyboard test publisher")
    rospy.init_node('thruster_test_publisher')
    # Create publishers for each thruster
    publishers = []
    for i in range(8):
        topic = f"/thrusters/{i}/input"
        pub = rospy.Publisher(topic, FloatStamped, queue_size=10)
        publishers.append(pub)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        for i in range(8):
            msg = FloatStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = thruster_forces[i]
            publishers[i].publish(msg)      

if __name__ == '__main__':
    
    keyboard_test_publisher_node()