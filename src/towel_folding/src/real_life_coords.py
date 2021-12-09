#! /usr/bin/env python
import rospy
import numpy as np
import tf2_ros

def get_real_life_coords(ar_real_zero, ar_towel_coords):
  """
  assume that the ar_coords are 4x2 numpy arrays - 4 dots from the towel
  translation (x, y, z)  in meters and rotation( quaternion, x, y, z, w). grippers to AR tag 
  """
  real_coords = np.empty(4)
  print("ar zero: ", ar_real_zero)
  pointCount = 0
  #z = ar_real_zero.translation.z
  for x, y in ar_towel_coords:
    geometry_msgs.PointStamped point_out
    print(x,y)
    pointCount = pointCount + 1
    #np.sum(ar_real_zero, coord)
    #real_coords[pointCount][0] = 
  


def main():
  return 

if __name__ == '__main__':
  main()



towel_coords = np.array([[1, 1], [2, 2], [3, 3], [4, 4]])
ar_real_zero = np.array([[2, 2, 2], [0.5, 0.5, 0.5, 0.5]])

get_real_life_coords(ar_real_zero,towel_coords)