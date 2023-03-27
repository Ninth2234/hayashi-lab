#!/usr/bin/env python3
import rospy
import rospkg

from time import sleep,time
import numpy as np

import pybullet as p
import pybullet_data
import OpenGL

from geometry_msgs.msg import PoseArray,Pose,Point,Quaternion
from visualization_msgs.msg import MarkerArray, Marker


from std_msgs.msg import Header
from utils.utils import *

class MeshPointPublisher:
    def __init__(self, softBodyId, softBodyNumMarker):

        # Create publisher for PoseArray
        self.pub_pose_array = rospy.Publisher('/mesh_position', PoseArray, queue_size=10)
        self.pub_marker_array = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        # Create PoseArray message
        self.msg = PoseArray()
        self.msg.header = Header()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "1"
        self.softBodyId = softBodyId
        self.softBodyNumMarker = softBodyNumMarker


        self.markerArrayMsg = MarkerArray()
        



    def publish(self):
        self.getMeshData()
        # Publish PoseArray message

        poseArray = []
        for meshPoint in self.meshPoints:
            id,(x,y,z) = meshPoint
            poseArray.append(Pose(position=Point(x=x,y=y,z=z)))

        self.msg.poses=poseArray            

        self.pub_pose_array.publish(self.msg)

    def getMeshData(self):
        self.meshPoints = getVerticesOnMarker(self.softBodyId,self.softBodyNumMarker)
        self.meshPoints.sort()

    def displayMarkerRviz(self):
        if self.meshPoints == None:
            return

        markerArray = []
        for meshPoint in self.meshPoints:
            id,(x,y,z) = meshPoint

            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "my_frame"
            marker.pose = Pose(position=Point(x=x,y=y,z=z),orientation=Quaternion(0,0,0,1))
            marker.type = 2 ## sphere
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = z/2
            marker.color.b = (1-z/2)*2
            marker.color.a = 1
            marker.action = 0
            marker.id = id

            markerArray.append(marker)
        
        self.markerArrayMsg.markers = markerArray

        self.pub_marker_array.publish(self.markerArrayMsg)

