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

    # Use CV to calculate offsets from AR tag
    image_path = 'src/towel_folding/src/camera_image.png'
    translation_vectors = get_translation_vectors(image_path)

    # Hard coded for now
    ar_tag_loc = np.array([.704, -0.048, -0.27])
    # 0.7 empirically tested to be slightly inside
    towel_coords = ar_tag_loc + 0.7 * translation_vectors 
    # Sleep
    rospy.sleep(1.0)

    # Compute movements
    # movements = compute_movements(towel_coords)

    # print(movements)

    print(towel_coords)
    
    # How far above the table to move the towel
    lift = np.array([0, 0, .1])
    # Temporary 0s for gripper
    gripper = [0 for i in range(7)]
    # Calibrate
    send_move_cmd(gripper, "Calibrate", "calibrate")
    # Hover over bottom-left
    send_move_cmd(towel_coords[0] + lift, "Bottom Left Lift")
    # Go down to bottom-left
    send_move_cmd(towel_coords[0], purpose="Bottom Left")
    # Close
    send_move_cmd(gripper, "Close", "close")
    # Lift back up at bottom-left
    send_move_cmd(towel_coords[0] + lift, purpose="Bottom Left Lift")
    # Get to top-right to drop off towel, but don't go all the way 
    send_move_cmd(towel_coords[2] + lift + np.array([0, towel_coords[0][1] - towel_coords[1][1], 0]), purpose="Top Right Lift Inside")
    # Let go of towel
    send_move_cmd(gripper, "Open", "open")

    # Hover over top-left
    send_move_cmd(towel_coords[1] + lift, purpose="Top Left Lift")
    # Drop down to towel
    send_move_cmd(towel_coords[1], purpose="Top Left")
    # Grab towel
    send_move_cmd(gripper, "Close", "close")
    # Lift towel
    send_move_cmd(towel_coords[1] + lift, purpose="Top Left Lift")
    # Move to bottom-left
    send_move_cmd(towel_coords[3] + lift, purpose="Bot Left Lift")
    # Drop towel
    send_move_cmd(gripper, "Open", "open")

# def send_move_cmd(location, purpose, close=""):
#     x = location[0]
#     y = location[1]
#     z = location[2]
#     quat_x = 0
#     quat_y = 1
#     quat_z = 0
#     quat_w = 0

#     try:
#         # Acquire service proxy
#         patrol_proxy = rospy.ServiceProxy('/towel_folding/motion', Move)
#         # Log data
#         rospy.loginfo(purpose)
#         # Call patrol service via the proxy
#         patrol_proxy(x, y, z, quat_x, quat_y, quat_z, quat_w, close)

#     except rospy.ServiceException as e:
        # rospy.loginfo(e)

def send_move_cmd(location, purpose, close=""):
    x = location[0]
    y = location[1]
    z = location[2]
    if len(location) == 3:
        quat_x = 0
        quat_y = 1
        quat_z = 0
        quat_w = 0
    else:
        quat_x = location[3]
        quat_y = location[4]
        quat_z = location[5]
        quat_w = location[6]

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
