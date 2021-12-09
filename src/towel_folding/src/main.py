#!/usr/bin/env python
import numpy as np
import rospy
from towel_folding.srv import Move  # Import service type
import sys

def main():
    # Initialize the movement client node
    rospy.init_node('movement_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('movement')

    # Move to calibration position
    


def send_move_cmd(location, purpose):
    x = location[0]
    y = location[1]
    z = location[2]
    quat_x = location[3]
    quat_y = location[4]
    quat_z = location[5]
    quat_w = location[6]

    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy('movement', Move)
        # Log data
        rospy.loginfo(purpose)
        # Call patrol service via the proxy
        patrol_proxy(x, y, z, quat_x, quat_y, quat_z, quat_w)

    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    main()
