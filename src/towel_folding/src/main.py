#!/usr/bin/env python
import numpy as np
import rospy

# from towel_folding.src.compute_folding_movements import compute_movements
# from towel_folding.src.takeImage import stop_saving

from towel_folding.srv import Move  # Import service type
from takeImage import stop_saving
from points import get_translation_vectors
from compute_folding_movements import compute_movements
from detect_ar import transform_generator

def main():
    # Start service for AR tag position
    transform_generator()
    # Initialize the movement client node
    rospy.init_node('motion_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/motion')

    # Move to calibration position
    send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "Calibrate")

    # Stop saving image to reduce compute intensity
    stop_saving()

    # Use CV to calculate offsets from AR tag
    image_path = 'camera_image.png'
    translation_vectors = get_translation_vectors(image_path)

    # Compute movements
    movements = compute_movements(translation_vectors)

    # Perform movements
    for movement in movements:
        send_move_cmd([movement[1], movement[2], movement[3], 0, 1, 0, 0], movement[0])



def send_move_cmd(location, purpose):
    x = location[0]
    y = location[1]
    z = location[2]
    quat_x = location[3]
    quat_y = location[4]
    quat_z = location[5]
    quat_w = location[6]

    result = ""

    while result != "Success":
        try:
            # Acquire service proxy
            patrol_proxy = rospy.ServiceProxy('/motion', Move)
            # Log data
            rospy.loginfo(purpose)
            # Call patrol service via the proxy
            result = patrol_proxy(x, y, z, quat_x, quat_y, quat_z, quat_w)

        except rospy.ServiceException as e:
            rospy.loginfo(e)

if __name__ == '__main__':
    main()
