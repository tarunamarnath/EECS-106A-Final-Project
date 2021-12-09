#!/usr/bin/env python
import numpy as np
import rospy
from towel_folding.srv import Move  # Import service type
import sys
# from points import get_translation_vectors
# from take_picture import take_image

def main():
    # Initialize the movement client node
    rospy.init_node('motion_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/motion')

    # Move to calibration position
    print(send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "Calibrate"))

    # Take image
    # image = take_image() # Alexis

    # Get offsets from AR tag
    # translation_vectors = get_translation_vectors(image)

    # Get AR tag position
    # AR_pos = # Vidish

    # Compute real-world coordinates
    # Alexis


    # Perform movements



    # Stop saving to save compute power
    # takeImage.stop_saving() 

    # Get offsets from AR tag
    image_path = 'camera_image.png'
    # translation_vectors = get_translation_vectors(image_path)

    # Get AR tag position
    # AR_pos =  # Vidish

    # Compute real-world coordinates
    # Alexis


    # Perform movements

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
        patrol_proxy = rospy.ServiceProxy('/motion', Move)
        # Log data
        rospy.loginfo(purpose)
        # Call patrol service via the proxy
        return patrol_proxy(x, y, z, quat_x, quat_y, quat_z, quat_w)

    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    main()
