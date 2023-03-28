#!/usr/bin/env python3
import rospy
import rospkg

from time import sleep,time
import numpy as np

import pybullet as p
import pybullet_data
import OpenGL

from utils.ContactTip import ContactTip1
from utils.meshPointPublisher import MeshPointPublisher
from utils.utils import *
from utils.ForceProjection import *
from utils.ImagePublisher import ImagePublisher

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import cv2
from cv_bridge import CvBridge

############### PARAMETER #################
TIME_STEP = 0.002

### ftest

##########################################


packagePath = rospkg.RosPack().get_path('tactile_simulation')

print(">>>", packagePath)

physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1,p.COV_ENABLE_WIREFRAME,1)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setTimeStep(TIME_STEP)

p.setAdditionalSearchPath(packagePath)

contactTip = ContactTip1([0,0,2])
# planeId = p.loadURDF(packagePath+"/urdf/plane.urdf", [0,0,0], [0,0,0,1])

clothId = p.loadSoftBody("softbody/myCloth.obj", basePosition = [0,0,1.5], scale = 1., mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=400, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)
p.changeVisualShape(clothId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
setBoundaryAtEdge(clothId)    


rospy.init_node("tactile_sim")


    


x = 100
imagePublisher = ImagePublisher()

meshPointPublisher = MeshPointPublisher(clothId,7)

forceDist = ForceDist(clothId, contactTip.id)

while p.isConnected() and not rospy.is_shutdown():

    srtTime = time()    
    if x>100:
        imagePublisher.publish()
        x=0
    x+=1

    # print(forceDist.queue.qsize)
    
    contactTip.update()
    meshPointPublisher.publish()
    meshPointPublisher.displayMarkerRviz()
    forceDist.captureAndHold()
        
    p.stepSimulation()  
    sleep(0.001)
    #print(time()-srtTime)
    














