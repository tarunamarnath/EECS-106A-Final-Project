#! /usr/bin/env python
import rospy
import numpy as np
import tf2_ros
import tf2_py as tf2
from geometry_msgs.msg import PointStamped

def get_base_frame_coords(ar_to_towel_coords):
  """
  Returns coordinates of important towel points in the base frame
  """
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)

  ar_frame_coords = {}
  base_frame_coords = np.empty(8)

  ar_frame_coords['bottom_left'] = ar_to_towel_coords[0]
  ar_frame_coords['top_left'] = ar_to_towel_coords[1]
  ar_frame_coords['top_right'] = ar_to_towel_coords[2]
  ar_frame_coords['bottom_right'] = ar_to_towel_coords[3]

  # Compute center coordinates
  ar_frame_coords['top_center'] = [ar_frame_coords['top_left'][0] + ar_frame_coords['top_right'][0], ar_frame_coords['top_left'][1] + ar_frame_coords['top_right'][1]]
  ar_frame_coords['bottom_center'] = [ar_frame_coords['bottom_left'][0] + ar_frame_coords['bottom_right'][0], ar_frame_coords['bottom_left'][1] + ar_frame_coords['bottom_right'][1]]
  ar_frame_coords['left_center'] = [ar_frame_coords['top_left'][0] + ar_frame_coords['bottom_left'][0], ar_frame_coords['top_left'][1] + ar_frame_coords['bottom_left'][1]]
  ar_frame_coords['right_center'] = [ar_frame_coords['top_right'][0] + ar_frame_coords['bottom_right'][0], ar_frame_coords['top_right'][1] + ar_frame_coords['bottom_right'][1]]

  for loc in ar_frame_coords.keys():
    point_in = PointStamped()
    point_in.point.x = ar_frame_coords[loc][0]
    point_in.point.y = ar_frame_coords[loc][1]
    point_in.point.z = 1
    
    point_in.header.stamp = rospy.Time.now()
    point_in.header.frame_id = "ar_marker_3" # RENAME to AR tag frame (if necessary)
    # find robot_target_frame
    # tfBuffer pi if necessary: https://github.com/ros/geometry2/blob/indigo-devel/tf2_ros/src/tf2_ros/buffer_interface.py#L37 
    base_frame_coords[loc + "_offset"] = tfBuffer.transform(point_in, "base")

    # point_in = Vector3Stamped()
    # point_in.vector.x = ar_frame_coords[loc][0]
    # point_in.vector.y = ar_frame_coords[loc][1]
    # point_in.vector.z = 0
    # point_in.header.stamp = rospy.time()
    # point_in.header.frame_id = "ar_marker_3" # RENAME to AR tag frame (if necessary)
    # # find robot_target_frame
    # # tfBuffer pi if necessary: https://github.com/ros/geometry2/blob/indigo-devel/tf2_ros/src/tf2_ros/buffer_interface.py#L37 
    # base_frame_coords[loc] = tfBuffer.transform("base", point_in)

  return base_frame_coords
  
def compute_movements(ar_towel_coords):
  """
  Compute movements necessary for towel folding
  """
  base_frame_coords = get_base_frame_coords(ar_towel_coords)

  movement_transforms = []

  # Move over bottom-left of cloth
  location = base_frame_coords['bottom_left_offset']
  movement_transforms.append("Bottom-left", location.vector.x, location.vector.y, location.vector.z)
  


def main():
  return 

if __name__ == '__main__':
  main()



# towel_coords = np.array([[1, 1], [2, 2], [3, 3], [4, 4]])
# ar_real_zero = np.array([[2, 2, 2], [0.5, 0.5, 0.5, 0.5]])

# get_real_life_coords(ar_real_zero,towel_coords)