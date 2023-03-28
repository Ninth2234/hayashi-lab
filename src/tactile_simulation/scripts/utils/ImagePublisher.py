#!/usr/bin/env python3
import rospy


from time import sleep,time
import numpy as np

import pybullet as p





from utils.utils import *
from utils.ForceProjection import *

from std_msgs.msg import Header
from sensor_msgs.msg import Image


import cv2
from cv_bridge import CvBridge


class ImagePublisher:
    def __init__(self):
        self.pub = rospy.Publisher("image",Image,queue_size=2)

        self.width = 1080
        self.height = 720

        fov = 62
        aspect = self.width / self.height
        near = 0.02
        far = 2

        self.view_matrix = p.computeViewMatrix([0, 0, -0.2], [0, 0, 1], [1, 0, 0])
        self.projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

        self.msg = Image()
        self.msg.header = Header()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "1"
        self.msg.height = self.height
        self.msg.width = self.width
        self.msg.encoding = "mono8"

        self.bridge = CvBridge()

    ## This funtion is so slow.
    def publish(self):
        
        image = p.getCameraImage(self.width,self.height,self.view_matrix,self.projection_matrix,shadow=False,renderer=p.ER_BULLET_HARDWARE_OPENGL)[2]
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        self.msg = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        
        self.pub.publish(self.msg)
            