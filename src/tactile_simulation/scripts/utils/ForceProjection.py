import numpy as np
import pybullet as p
from utils.utils import *
from queue import Queue
from threading import Thread
from std_msgs.msg import Float32MultiArray,MultiArrayLayout,MultiArrayDimension
import rospy

def isVectorPassThroughFace(vector, vertices):
    """
    Checks if a given vector passes through a square face defined by four vertices.

    Args:
    vector: numpy array of shape (3,) representing the vector to be checked
    vertices: list of numpy arrays of shape (3,) representing the vertices of the square face

    Returns:
    True if the vector passes through the square face, False otherwise
    """

    vector = np.array(vector)


    # Calculate the normal vector of the square face
    normal = np.cross(vertices[1]-vertices[0], vertices[3]-vertices[0])

    # Check if the vector is parallel to the square face
    if np.dot(vector, normal) == 0:
        return False

    # Calculate the point of intersection between the vector and the plane containing the square face
    P0 = vertices[0]
    t = np.dot(normal, P0 - vector) / np.dot(normal, vector)
    P = vector + t * vector

    if abs(t)>0.6:
        return False
    

    # Check if the intersection point lies within the square
    min_x = min(vertices[i][0] for i in range(4))
    max_x = max(vertices[i][0] for i in range(4))
    min_y = min(vertices[i][1] for i in range(4))
    max_y = max(vertices[i][1] for i in range(4))
    min_z = min(vertices[i][2] for i in range(4))
    max_z = max(vertices[i][2] for i in range(4))

        
    if min_x <= P[0] <= max_x and min_y <= P[1] <= max_y and min_z <= P[2] <= max_z:
        return True
    else:
        return False
    


# @debugTimer
def getFaceIndex(vector, origin, softBodyId, showFace=False):
    for index,face in getFacesVertices(softBodyId):            
        vertices = [point-origin for point in face]
        
        if isVectorPassThroughFace(vector,vertices):
            # if(showFace):
            #     showSqureFace(vertices,linkPos)
            # p.addUserDebugLine(origin, np.add(vector,origin),[255,255,0],lineWidth=2)
            return index[0]
        
    return -1



def getForceDist(softBodyId, contactObjId):
    retImg = np.zeros((17,17),np.float32)    
    for point in p.getContactPoints(softBodyId,contactObjId):
        
        # contactFlag,bodyAId,bodyBId,linkA,linkB,posOnA,posOnB,normal,distance,force,*_ = point
        _,_,_,_,_,_,_,normal,_,force,*_ = point

        linkPos,*_ = p.getLinkState(contactObjId, 3)
        
        idx = getFaceIndex(normal,linkPos,softBodyId)

        if idx > 0:
            u,v = int(idx%17),int(idx//17)
            retImg[u][v] = force
                    
    return retImg



class ForceDist:
    def __init__(self, softBodyId, contactObjId):
        self.softBodyId = softBodyId
        self.contactObjId = contactObjId
        self.queue = Queue()
        self.queue.qsize = 1

        self.msg = Float32MultiArray()
        self.msg.layout.dim.append(MultiArrayDimension())
        self.msg.layout.dim[0].label = "rows"
        self.msg.layout.dim[0].size = 17
        self.msg.layout.dim[0].stride = 17*17

        self.msg.layout.dim.append(MultiArrayDimension())
        self.msg.layout.dim[1].label = "cols"
        self.msg.layout.dim[1].size = 17
        self.msg.layout.dim[1].stride = 17

        self.publisher = rospy.Publisher("force_dist",Float32MultiArray,queue_size=10)
        self.worker = Thread(target=self.work)
        self.worker.start()
        
        
        
        

    def captureAndHold(self):
        if self.queue.empty():
            linkPos,*_ = p.getLinkState(self.contactObjId, 3)
            self.queue.put([p.getContactPoints(self.softBodyId,self.contactObjId),getFacesVertices(self.softBodyId),linkPos])

    def work(self):
        while True:            
            if not self.queue.empty():
                self.result = self.calculate(self.queue.get())                
                
                self.msg.data = self.result.flatten().tolist()
                self.publisher.publish(self.msg)
                
                

    def calculate(self,payload):
        retImg = np.zeros((17,17),np.float32)

        points, faces, linkPos = payload
        for point in points:

            _,_,_,_,_,_,_,normal,_,force,*_ = point
            
            idx = self.getFaceIndex(normal, linkPos, faces)

            if idx > 0:
                u,v = int(idx%17),int(idx//17)
                retImg[u][v] = force
                
        return retImg.copy()
    

    def getFaceIndex(self, vector, origin, faces):
        for index,face in faces:
            vertices = [point-origin for point in face]            
            if isVectorPassThroughFace(vector,vertices):
                return index[0]            
        return -1
    

