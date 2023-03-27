#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Point


import numpy_ros
import rospy

import csv

import threading

import random

from utils.my_ros_numpy import *

from time import sleep

FILE_DIRECTORY = "data/"

class DataLogger:
    def __init__(self):
    
        rospy.init_node('logger')
    
        self.sub_feature = rospy.Subscriber('/features', Float32MultiArray, self.callback_feature)
        self.sub_mesh_position = rospy.Subscriber('/mesh_position', PoseArray, self.callback_mesh_position)
    
        #self.pub_contact_pose = rospy.Publisher('/sensor_data', String, queue_size=10)
        
        
        self.lock_feature = threading.Lock()
        self.lock_mesh = threading.Lock()


        self.pub = rospy.Publisher("/target_point", Point, queue_size=10)
        

    def callback_feature(self, data):
        self.lock_feature.acquire()
        self.feature_data = float32MultiArray2Numpy(data)
        self.lock_feature.release()
        

    def callback_mesh_position(self, data):
        self.lock_mesh.acquire()
        self.mesh_position_data = poseArray2Numpy(data)
        self.lock_mesh.release()
        

    def publish_random_point(self):
        self.targetpoint = Point(x=random.random()*1-0.5,y=random.random()*1-0.5,z= random.random()*0.2+0.1)
        #print(point)
        self.pub.publish(self.targetpoint)

    
    def capture_data(self,step):
        self.lock_feature.acquire()
        self.lock_mesh.acquire()

        if hasattr(self, 'feature_data') and hasattr(self, 'mesh_position_data'):
            np.savetxt(FILE_DIRECTORY+f'feature_{step}.csv', self.feature_data, delimiter=',')
            np.savetxt(FILE_DIRECTORY+f'mesh_{step}.csv', self.mesh_position_data, delimiter=',')

        self.lock_mesh.release()
        self.lock_feature.release()
        


if __name__ == '__main__':
    try:
        # Create SensorDataLogger object
        dataLogger = DataLogger()

        # Publish sensor data at a rate of 10 Hz
        rate = rospy.Rate(0.5)
        # while not rospy.is_shutdown():
        for step in range(1000):
            if rospy.is_shutdown():
                break
            
            dataLogger.publish_random_point()

            sleep(1.5)

            dataLogger.capture_data(step)
            
            print("---------------")
            print()
            print("step",step)
            print(dataLogger.targetpoint)
            print()
            print("---------------")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
