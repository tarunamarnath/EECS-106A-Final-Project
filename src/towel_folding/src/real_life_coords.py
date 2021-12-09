#! /usr/bin/env python
import rospy
import numpy as np
import tf2_ros
import tf2_py as tf2
from geometry_msgs.msg import Vector3Stamped

def get_real_life_coords(ar_z, ar_towel_coords):
  """
  assume that the ar_coords are 4x2 numpy arrays - 4 dots from the towel
  ar_z : the z value of the ar tag to be used as z value of towel values 
  """
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
  real_coords = np.empty(4)
  print("ar zero: ", ar_real_zero)
  i = 0
  for x, y in ar_towel_coords:
    point_in = Vector3Stamped()
    point_in.vector.x = x
    point_in.vector.y = y
    point_in.vector.z = ar_z
    point_in.header.stamp = rospy.time()
    point_in.header.frame_id = "output_frame"
    # find robot_target_frame
    # tfBuffer pi if necessary: https://github.com/ros/geometry2/blob/indigo-devel/tf2_ros/src/tf2_ros/buffer_interface.py#L37 
    real_coords[i] = tfBuffer.transform(point_in, "robot_left_gripper_frame")
    i = i + 1

  return real_coords
  


def main():
  return 

if __name__ == '__main__':
  main()



towel_coords = np.array([[1, 1], [2, 2], [3, 3], [4, 4]])
ar_real_zero = np.array([[2, 2, 2], [0.5, 0.5, 0.5, 0.5]])

get_real_life_coords(ar_real_zero,towel_coords)