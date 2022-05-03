#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel
from std_msgs.msg import Float32

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = True

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.direction_pub = rospy.Publisher("/direct_pub", Float32, queue_size = 10)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Make a mask
        # Run module 1 cone detection on masked image
        # Return center of the bounding box
        if (not self.LineFollower):
            bbox = cd_color_segmentation(image, None)
        else:
            mask = np.zeros((image.shape[0],image.shape[1],3),dtype=np.uint8)
            mask[220:250,:,:] = 1 #previous 240 - 260, pixels at top are 0, 220 - 250
            new_image = image*mask
            bbox = cd_color_segmentation(new_image, None)

            #Crop the camera's image and return a small section about an eighth of the way up 
        # Another node will use homography transformation to turn this into a point in space
        cv2.rectangle(image, bbox[0], bbox[1], (255,255,255), 3)
        image = cv2.circle(image, ((bbox[0][0] + bbox[1][0]) / 2, bbox[1][1]), 1, (255,0,0), 3)
        
        # print(len(image[0]))
        # quit()

        cone_location = ConeLocationPixel()
        cone_location.u = (bbox[0][0] + bbox[1][0]) / 2
        cone_location.v = (bbox[1][1])

        #check which side the line is on
        #0 for left, 1.0 for right
        line_side = Float32()
        if cone_location.u < 320: #right
            line_side = -1.0
        else:
            line_side = 1.0 #left
        self.direction_pub.publish(line_side)

        self.cone_pub.publish(cone_location)
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
