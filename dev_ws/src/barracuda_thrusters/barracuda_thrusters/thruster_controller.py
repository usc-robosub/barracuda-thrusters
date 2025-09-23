import rclpy
from rclpy.node import Node

from smbus import *
import struct
from collections import namedtuple
import RPi.GPIO as GPIO 
#from F2PWM import F2PWM

# no longer using uuv_gazebo_ros_plugins_msgs package
# from barracuda_msgs.msg import Float (FloatStamped?)

class BarracudaThrusters(Node):
  
  def __init__(self):
    super().__init('barracuda_thrusters')
    for i in range(NTHRUSTERS):
      topic = f'thrusters/{i}/input'
      # self._subscriptions.append(self.create_subscription(Float, topic, lambda msg: self.on_recv_force(msg, i), 10)
    
    # self.converter = F2PWM('t200_18v_data.csv')

  # def on_recv_force(self, msg, thruster_idx):
  #   self.
def main():
    print('Hi from barracuda_thrusters.')


if __name__ == '__main__':
    main()
