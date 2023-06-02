#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import numpy as np

from jetson_inference import segNet
from jetson_utils import videoSource, videoOutput, cudaOverlay, cudaDeviceSynchronize, Log

from segnet_utils import *


	#ROS_CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);

class safe_land_decide:
    def __init__(self):
        #self.pub = rospy.Publisher("/number_count", Int64, queue_size=10) 
        self.sub = rospy.Subscriber("/safeland_cv/class_mask", Image, self.callback) 

    def callback(self):
#GetClassLabel(...)
        



        #need to change later
        #new_msg = Int64()
        #new_msg.data = self.counter
        #self.pub.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('safe_land_decide')
    safe_land_decide()
    rospy.spin()