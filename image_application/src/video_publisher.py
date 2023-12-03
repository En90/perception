#!/home/en/openvino_env/bin/python3

import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time

cap = cv2.VideoCapture()

def myhook():
    rospy.logwarn("shutdown video publisher")
    cap.release()
    cv2.destroyAllWindows()

def image_publisher():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('output_image', Image, queue_size=100)
    
    if rospy.has_param('~repeat'):
        repeat_:bool = rospy.get_param("~repeat")
        rospy.loginfo("repeat the video")
    else:
        repeat_ = False
        
    if rospy.has_param('~frequency'):
        fq_:int = rospy.get_param("~frequency")
        rospy.loginfo("set video frequeny: %d", fq_)
    else:
        fq_ = 20
        
    if rospy.has_param('~height'):
        height_:int = rospy.get_param("~height")
        rospy.loginfo("set video height: %d", height_)
    else:
        height_ = 640
        
    if rospy.has_param('~width'):
        width_:int = rospy.get_param("~width")
        rospy.loginfo("set video width: %d", width_)
    else:
        width_ = 1080
    
    rate = rospy.Rate(fq_) # 20Hz
    bridge = CvBridge()
    resource = sys.argv[1]
    resource_name = resource
    rospy.loginfo("Trying to open resource: %s", resource_name)
    cap.open(resource)
    if not cap.isOpened():
        rospy.logwarn("Error opening resource: %s", str(resource))
        rospy.logwarn("Maybe opencv VideoCapture can't open it")
        exit(0)
    rospy.loginfo("Correctly opened resource, starting to show feed.")
    rval, frame = cap.read()
    
    while cap.isOpened() and not rospy.is_shutdown():
        rval, frame = cap.read()
        # cv2.imshow("Stream: " + resource_name, frame)
        # ROS image stuff
        if (frame is not None) and rval:
            frame = np.uint8(frame)
        else:
            if repeat_:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            else:
                exit(0)
        frame = cv2.resize(frame, (width_, height_), interpolation= cv2.INTER_LINEAR)
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_message.header.stamp = rospy.get_rostime()
        pub.publish(image_message)
        key = cv2.waitKey(1000//fq_)   # 50 milisecond
        # exit on ESC, you may want to uncomment the rospy.loginfo to know which key is ESC for you
        if key == 27 or key == 1048603:
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.on_shutdown(myhook)
        image_publisher()
    except rospy.ROSInterruptException:
        pass