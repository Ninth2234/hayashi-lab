#!/usr/bin/env python3
import rospy
import rospkg

from time import sleep,time
import numpy as np

import pybullet as p
import pybullet_data
import OpenGL

from utils.ContactTip import ContactTip
from utils.meshPointPublisher import MeshPointPublisher
from utils.utils import *

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

contactTip = ContactTip([0,0,2])
# planeId = p.loadURDF(packagePath+"/urdf/plane.urdf", [0,0,0], [0,0,0,1])

clothId = p.loadSoftBody("softbody/myCloth.obj", basePosition = [0,0,1.5], scale = 1., mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)
p.changeVisualShape(clothId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
setBoundaryAtEdge(clothId)    


rospy.init_node("tactile_sim")

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
            
    
def callback(msg:Point):
    contactTip.setPosition(msg.x,msg.y,msg.z)

rospy.Subscriber("/target_point", Point, callback)

x = 100
imagePublisher = ImagePublisher()

meshPointPublisher = MeshPointPublisher(clothId,7)

while p.isConnected() and not rospy.is_shutdown():

    srtTime = time()    
    if x>100:
        imagePublisher.publish()
        x=0
    x+=1
    
    contactTip.update()
    meshPointPublisher.publish()
    meshPointPublisher.displayMarkerRviz()
    
        
    p.stepSimulation()  

    #print(time()-srtTime)
    














