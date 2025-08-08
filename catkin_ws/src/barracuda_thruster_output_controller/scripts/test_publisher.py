#!/usr/bin/env python3

import rospy
import time
import random
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

def test_publisher_node():
    # Initialize ROS node
    rospy.init_node('thruster_test_publisher')
    
    # Create publishers for each thruster
    publishers = []
    for i in range(8):
        topic = f"/thrusters/{i}/input"
        pub = rospy.Publisher(topic, FloatStamped, queue_size=10)
        publishers.append(pub)
    
    # Give publishers time to connect
    rospy.sleep(1.0)
    
    # Publication rate (Hz) - higher rate for smoother transitions
    rate = rospy.Rate(20)  
    
    # Test duration in seconds
    test_duration = 10.0
    start_time = rospy.get_time()
    
    # Initialize current force values for each thruster
    current_forces = [random.uniform(-2.0, 2.0) for _ in range(8)]
    
    print("Starting thruster test publisher...")
    
    while not rospy.is_shutdown() and (rospy.get_time() - start_time) < test_duration:
        # Update each thruster with a small delta
        for i in range(8):
            # Generate small random change (delta)
            delta = random.uniform(-0.2, 0.2)
            
            # Update current force with delta
            current_forces[i] += delta
            
            # Ensure we stay within the -4 to 6 kgF range
            current_forces[i] = max(-4.0, min(6.0, current_forces[i]))
            
            # Create and publish message
            msg = FloatStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = current_forces[i]
            publishers[i].publish(msg)
        
        # Log current values occasionally
        current_time = rospy.get_time() - start_time
        if int(current_time * 2) % 2 == 0 and int(current_time * 10) % 10 == 0:
            print(f"Time: {current_time:.1f}s - Current forces (kgF):")
            for i, force in enumerate(current_forces):
                print(f"  Thruster {i}: {force:.2f}")
            
        rate.sleep()
    
    # At the end, gradually reduce all thrusters to zero
    print("Test complete. Gradually reducing all thrusters to zero...")
    
    # Take 2 seconds to ramp down to zero
    ramp_down_duration = 2.0
    ramp_down_start = rospy.get_time()
    
    while not rospy.is_shutdown() and (rospy.get_time() - ramp_down_start) < ramp_down_duration:
        progress = (rospy.get_time() - ramp_down_start) / ramp_down_duration
        
        for i in range(8):
            # Linear interpolation to zero
            target_force = current_forces[i] * (1 - progress)
            
            msg = FloatStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = target_force
            publishers[i].publish(msg)
            
        rate.sleep()
    
    # Final set to zero to ensure all thrusters stop
    zero_msg = FloatStamped()
    zero_msg.header.stamp = rospy.Time.now()
    zero_msg.data = 0.0
    
    for pub in publishers:
        pub.publish(zero_msg)
        
    print("Thruster test publisher finished.")

    rospy.spin()

def main():
    try:
        test_publisher_node()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error in thruster test publisher: {e}")

if __name__ == '__main__':
    main()
