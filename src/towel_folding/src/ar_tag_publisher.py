#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the dependencies as described in example_pub.py
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

import rospy

# Define the method which contains the node's main functionality
def listener():

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher('marker_information', TransformStamped, queue_size=10)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("ar_marker_3", "world", rospy.Time())
            rospy.loginfo(trans)
            pub.publish(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
        r.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('ar_marker_location')
    listener()
