#!/usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
import rospy
from std_srvs.srv import Empty
from turtle_patrol.srv import Move  # Service type
from turtlesim.srv import TeleportAbsolute
import sys


def motion_callback(request):
    rospy.wait_for_service('clear')
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/' + request.name + '/teleport_absolute')
    clear_proxy = rospy.ServiceProxy('clear', Empty)
    teleport_proxy = rospy.ServiceProxy(
        '/'+ request.name + '/teleport_absolute',
        TeleportAbsolute
    )
    x = request.x
    y = request.y
    theta = request.theta
    name = request.name

    vel = request.vel  # Linear velocity
    omega = request.omega  # Angular velocity

    pub = rospy.Publisher(
        '/'+ request.name + '/cmd_vel', Twist, queue_size=50)
    cmd = Twist()
    cmd.linear.x = vel
    cmd.angular.z = omega
    # Publish to cmd_vel at 5 Hz
    rate = rospy.Rate(5)
    # Teleport to initial pose
    teleport_proxy(9, 5, np.pi/2)
    # Clear historical path traces
    clear_proxy()
    while not rospy.is_shutdown():
        pub.publish(cmd)  # Publish to cmd_vel
        rate.sleep()  # Sleep until
    return cmd  # This line will never be reached

def motion_server():
    # Initialize the server node for turtle1
    rospy.init_node('motion_server')
    # Register service
    rospy.Service(
        "/motion",  # Service name
        Move,  # Service type
        motion_callback  # Service callback
    )
    rospy.loginfo('Running motion server...')
    rospy.spin() # Spin the node until Ctrl-C


# Python's syntax for a main() method
if __name__ == '__main__':
    motion_server()