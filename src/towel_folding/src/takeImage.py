#! /usr/bin/env python
# rospy for the subscriber
import cv2
from numpy.lib.npyio import save
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# Instantiate CvBridge
bridge = CvBridge()

save_image = True

def takeImage(data):
    """
    Goal: knowing that you are already in the correct position, take an image and return it
    Baxter topic: /cameras/right_hand_camera/image
    """
    global save_image
    print(save_image)
    if (save_image):
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite('./src/towel_folding/src/camera_image.png', cv2_img)
            exit

        except CvBridgeError as e:
            print(e)
            # Save your OpenCV2 image as a jpeg 
            
    rospy.sleep(20)
  


def startImages():
    #ic = ImgCapture()
    # rospy.init_node('capture_image')
    # Define your image topic
    image_topic = "/cameras/left_hand_camera/image"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, takeImage)
    # Spin until ctrl + c
    # rospy.spin()
    # while save_image

def stop_saving():
    global save_image
    print("in func")
    save_image = False

if __name__ == '__main__':
    startImages()