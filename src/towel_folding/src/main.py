#!/usr/bin/env python
import numpy as np
import rospy
import roslaunch

# from towel_folding.src.compute_folding_movements import compute_movements
from takeImage import startImages, stop_saving

from towel_folding.srv import Move  # Import service type
import takeImage
from points import get_translation_vectors
from compute_folding_movements import compute_movements
from detect_ar import transform_generator

def main():
    # Start service for AR tag position
    # transform_generator()
    # Initialize the movement client node
    rospy.init_node('motion_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/towel_folding/motion')
    

    # Move to calibration position
    send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "", "Calibrate")
    takeImage.startImages()
    rospy.sleep(2.0)
    takeImage.stop_saving()

    # send_move_cmd([.404, .153, -.098, 0.0, 1.0, 0.0, 0.0], "motion2")

    # Stop saving image to reduce compute intensity
    

    # Use CV to calculate offsets from AR tag
    image_path = 'src/towel_folding/src/camera_image.png'
    translation_vectors = get_translation_vectors(image_path)

    print(translation_vectors)

    ar_tag_loc = np.array([.704, -0.048, -0.27])
    base = np.array([.804, .253, .098])


    towel_coords = ar_tag_loc + 0.7 * translation_vectors 

    base_towel = towel_coords
    rospy.sleep(1.0)

    # Compute movements
    # movements = compute_movements(base_towel)

    # print(movements)

    print(towel_coords)
    
    lift = np.array([0, 0, .1])
    # send_move_cmd_towel([.5, .456, -.25], purpose="Blah")
    # send_move_cmd_towel([.807, .452, -.25], purpose="Blah")
    # send_move_cmd_towel([.811, .117, -.25], purpose="Blah")
    # send_move_cmd_towel([.532, .084, -.25], purpose="Blah")
    send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "calibrate", purpose="Calibrate")
    send_move_cmd_towel(base_towel[0] + lift, purpose="Bottom Left Lift")
    send_move_cmd_towel(base_towel[0], purpose="Bottom Left")
    send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "close", purpose="Close")
    send_move_cmd_towel(base_towel[0] + lift, purpose="Bottom Left Lift")
    # send_move_cmd_towel(base_towel[1], purpose="Top Left")

    send_move_cmd_towel(base_towel[2] + lift + np.array([0, base_towel[0][1] - base_towel[1][1], 0]), purpose="Top Right Lift Inside")
    send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "open", purpose="Open")
    # send_move_cmd_towel(base_towel[2], purpose="Top Right")

    send_move_cmd_towel(base_towel[1] + lift, purpose="Top Left Lift")
    send_move_cmd_towel(base_towel[1], purpose="Top Left")
    send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "close", purpose="Close")
    send_move_cmd_towel(base_towel[1] + lift, purpose="Top Left Lift")
    send_move_cmd_towel(base_towel[3] + lift, purpose="Bot Left Lift")
    # send_move_cmd_towel(base_towel[3], purpose="Bot Left")
    send_move_cmd([.804, .253, .098, 0.0, 1.0, 0.0, 0.0], "open", purpose="Open")


    # Perform movements
    # for movement in movements:
    #     send_move_cmd([movement[1], movement[2], movement[3], 0, 1, 0, 0], movement[0])


def send_move_cmd_towel(location, purpose):
    x = location[0]
    y = location[1]
    z = location[2]
    quat_x = 0
    quat_y = 1
    quat_z = 0
    quat_w = 0
    close = " "

    result = ""

    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy('/towel_folding/motion', Move)
        # Log data
        rospy.loginfo(purpose)
        # Call patrol service via the proxy
        patrol_proxy(x, y, z, quat_x, quat_y, quat_z, quat_w, "")

    except rospy.ServiceException as e:
        rospy.loginfo(e)

def send_move_cmd(location, close, purpose):
    x = location[0]
    y = location[1]
    z = location[2]
    quat_x = location[3]
    quat_y = location[4]
    quat_z = location[5]
    quat_w = location[6]
    close = close

    result = ""

    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy('/towel_folding/motion', Move)
        # Log data
        rospy.loginfo(purpose)
        # Call patrol service via the proxy
        patrol_proxy(x, y, z, quat_x, quat_y, quat_z, quat_w, close)

    except rospy.ServiceException as e:
        rospy.loginfo(e)

if __name__ == '__main__':
    main()
