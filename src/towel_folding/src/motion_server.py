#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
import numpy as np
from numpy import linalg
from towel_folding.srv import Move  # Service type
import sys
from baxter_interface import gripper as robot_gripper


def motion_callback(request):
    requested_position = [request.x, request.y, request.z, request.quat_x, request.quat_y, request.quat_z, request.quat_w]
    print(requested_position)
    robo = "baxter"
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    
    arm = 'left'

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    gripper = robot_gripper.Gripper(arm)
    link = arm + "_gripper"

    def make_request(x, y, z, quat_x, quat_y, quat_z, quat_w, link):
        
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"
        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.x = quat_x
        request.ik_request.pose_stamped.pose.orientation.y = quat_y
        request.ik_request.pose_stamped.pose.orientation.z = quat_z
        request.ik_request.pose_stamped.pose.orientation.w = quat_w

        orien_const = OrientationConstraint()
        orien_const.link_name = "left_gripper"
        orien_const.header.frame_id = "base"
        orien_const.orientation.y = -1.0

        constraints = Constraints()
        # constraints.orientation_constraints = [orien_const]
        try:
            # Send the request to the service
            response = compute_ik(request)
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")
            group.set_path_constraints(constraints)
            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)
            # Plan IK and execute
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    if request.close == "close":
        gripper.close()
    elif request.close == "calibrate":
        gripper.calibrate()
    elif request.close == "open":
        gripper.open()
    else:
        make_request(*requested_position, link=link)
    return "Success"


def motion_server():
    # Initialize the server node for turtle1
    rospy.init_node('towel_motion_server', anonymous=True)

    # Register service
    rospy.Service(
        "/towel_folding/motion",  # Service name
        Move,  # Service type
        motion_callback  # Service callback
    )
    rospy.loginfo('Running motion server...')
    rospy.spin() # Spin the node until Ctrl-C


# Python's syntax for a main() method
if __name__ == '__main__':
    motion_server()
