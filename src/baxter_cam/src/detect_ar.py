#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import roslaunch

from geometry_msgs.msg import Twist, Vector3, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

#Define the method which contains the main functionality of the node.
def transform_generator(ar_frame, gripper_frame):
  
  ## INIT

  node = roslaunch.core.Node('ar_track_alvar', 'ar_track_alvar')
  launch = roslaunch.scriptapi.ROSLaunch()
  launch.start()

  process = launch.launch(node)
  print(process.is_alive())

  ### FINISH INIT

  #Create a publisher and a tf buffer, which is primed with a tf listener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # Loop until we have a transform
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform(ar_frame, gripper_frame, time)

      print("translation: " + str(trans.transform.translation))
      print("translation: " + str(trans.transform.rotation))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
  process.stop()

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':

  try:
    controller(sys.argv[1], sys.argv[2], rospy.Time())
  except rospy.ROSInterruptException:
    pass