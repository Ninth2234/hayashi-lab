from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray,Pose

from numpy_ros import to_numpy
#import rospy
import numpy as np
from math import sqrt



def poseArray2Numpy(msg:PoseArray):
    
    length = len(msg.poses)
    num_point = int(sqrt(length))
    

    ret = np.ndarray(shape=(length,3))

    # for pose in msg.poses:
        
    #     print(to_numpy(pose.position))
    for i in range(length):
        ret[i,:] = to_numpy(msg.poses[i].position)

    return ret



def float32MultiArray2Numpy(msg:Float32MultiArray):
    
    # print(len(msg.data))
    return np.array(msg.data).reshape(msg.layout.dim[0].size, msg.layout.dim[1].size)
    