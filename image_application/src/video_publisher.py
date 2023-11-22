#!/home/en/openvino_env/bin/python3

import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time

def image_publisher():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('output_image', Image, queue_size=100)
    rate = rospy.Rate(20) # 20Hz
    bridge = CvBridge()
    resource = sys.argv[1]
    resource_name = resource
    rospy.loginfo("Trying to open resource: %s", resource_name)
    cap = cv2.VideoCapture(resource)
    if not cap.isOpened():
        rospy.logwarn("Error opening resource: %s", str(resource))
        rospy.logwarn("Maybe opencv VideoCapture can't open it")
        exit(0)
    rospy.loginfo("Correctly opened resource, starting to show feed.")
    rval, frame = cap.read()
    
    while rval:
        rval, frame = cap.read()
        # cv2.imshow("Stream: " + resource_name, frame)
        # ROS image stuff
        if frame is not None:
            frame = np.uint8(frame)
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(image_message)
        key = cv2.waitKey(50)   # 50 milisecond
        # exit on ESC, you may want to uncomment the rospy.loginfo to know which key is ESC for you
        if key == 27 or key == 1048603:
            break
        rate.sleep()
    # cv2.destroyWindow("Stream: " + resource_name)

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass