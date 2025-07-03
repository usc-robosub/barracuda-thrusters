#!/usr/bin/env python3

import rospy
import sys
from barracuda_thruster_output_controller.srv import DisableMotors

def test_kill_switch():
    """Test script for the DisableMotors service"""
    rospy.init_node('kill_switch_test')
    
    # Wait for service to be available
    rospy.wait_for_service('disable_motors')
    
    try:
        # Create service proxy
        disable_motors = rospy.ServiceProxy('disable_motors', DisableMotors)
        
        if len(sys.argv) > 1:
            disable_value = sys.argv[1].lower() in ['true', '1', 'yes', 'on', 'disable']
        else:
            print("Usage: rosrun barracuda_thruster_output_controller test_kill_switch.py <true|false>")
            print("  true/1/yes/on/disable: Enable kill switch (disable motors)")
            print("  false/0/no/off/enable: Disable kill switch (enable motors)")
            return
        
        # Call the service
        response = disable_motors(disable_value)
        
        if response.success:
            print(f"SUCCESS: {response.message}")
        else:
            print(f"FAILED: {response.message}")
            
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_kill_switch()
    except rospy.ROSInterruptException:
        pass