# by default this will sweep all thrusters from 0 to 20 to -20 to 0 newtons
# if you give thruster indices as params, only those thrusters will be swept

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32