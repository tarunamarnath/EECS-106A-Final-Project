#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from baxter_interface import gripper as robot_gripper


def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
        arm = 'right'

    right_gripper = robot_gripper.Gripper(arm)

    def make_request(x, y, z, link):
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"
        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0

        try:
            # Send the request to the service
            response = compute_ik(request)

            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        # pos1 = [.754, .541, -.147]
        pos1 = [.804, .253, .098]

        pos2 = [.770, .541, .147]
        pos3 = [.798, .198, .155]
        pos4 = [.675, .190, -.176]

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = arm + "_gripper"
        if robo == 'sawyer':
            link += '_tip'


        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # Move to Pos2
        # make_request(*pos2, link=link)

        # Calibrate
        # print('Calibrating...')
        # right_gripper.calibrate()
        # rospy.sleep(2.0)

        # # Move to Pos2
        make_request(*pos4, link=link)


        # Move to Pos1
        make_request(*pos1, link=link)

        # # Close
        # print('Closing...')
        # right_gripper.close()
        # rospy.sleep(1.0)

        # rospy.sleep(2.0)


        # # Move to Pos3
        # make_request(*pos3, link=link)

        # # Move to Pos4
        # make_request(*pos4, link=link)

        # # Open
        # print('Opening...')
        # right_gripper.open()
        # rospy.sleep(1.0)

        # # Move to Pos3
        # make_request(*pos3, link=link)

        # # Move to Pos2
        # make_request(*pos2, link=link)

        # # Move to Pos3
        # make_request(*pos3, link=link)

        # # Move to Pos4
        # make_request(*pos4, link=link)

        # # Sleep
        # rospy.sleep(5.0)
        # # Close
        # print('Closing...')
        # right_gripper.close()
        # rospy.sleep(1.0)

        # rospy.sleep(2.0)

        # # Move to Pos2
        # make_request(*pos2, link=link)

        # # move to Pos 1
        # make_request(*pos1, link=link)

        # # Open
        # print('Opening...')
        # right_gripper.open()
        # rospy.sleep(1.0)

        # rospy.sleep(2.0)


# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])
