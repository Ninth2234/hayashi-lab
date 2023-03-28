import pybullet as p
import math
import numpy as np

from time import time

""" setBoundaryAtEdge with softBodyId """
def setBoundaryAtEdge(softBodyId):
    totalPoints,points = p.getMeshData(softBodyId,flags=1)
    xDatas = [point[0] for point in points]
    yDatas = [point[1] for point in points]
    xMin,xMax = [min(xDatas),max(xDatas)]
    yMin,yMax = [min(yDatas),max(yDatas)]

    def isEdge(point):
        x,y,z = points[i]
        return x == xMin or x == xMax or y == yMin or y == yMax

    for i in range(totalPoints):
        xyz = points[i]
        if(isEdge(xyz)):
            p.createSoftBodyAnchor(softBodyId, i, -1, -1)

""" show vertices of softbody mesh with input softBodyId """
def showVertices(softBodyId, showMeshId=True, LENGTH = 0.05):
    

    totalPoints,points = p.getMeshData(softBodyId,flags=1)
    #position,quarternion = p.getBasePositionAndOrientation(clothId)    

    for i in range(totalPoints):
        x,y,z = [points[i][j] for j in range(3)]

        p.addUserDebugLine([x-LENGTH,y+LENGTH,z],[x+LENGTH,y-LENGTH,z],lineColorRGB=[1,1,0])
        p.addUserDebugLine([x+LENGTH,y+LENGTH,z],[x-LENGTH,y-LENGTH,z],lineColorRGB=[1,1,0])
        if(showMeshId):
            p.addUserDebugText(str(i),[x,y,z], textColorRGB=[1,1,0])



def showPoint(point, LENGTH = 0.05, color=[1,1,0], lineWidth=1):
    x,y,z = point
    p.addUserDebugLine([x-LENGTH,y+LENGTH,z],[x+LENGTH,y-LENGTH,z],lineColorRGB=color,lineWidth = lineWidth)
    p.addUserDebugLine([x+LENGTH,y+LENGTH,z],[x-LENGTH,y-LENGTH,z],lineColorRGB=color,lineWidth = lineWidth)


def getFacesVertices(softBodyId):
    totalPoints,points = p.getMeshData(softBodyId,flags=1)

    numVerticesOnEdge = int(math.sqrt(totalPoints))

    ret = []

    for j in range(numVerticesOnEdge-1):
        for i in range(numVerticesOnEdge-1):
            index = i+j*numVerticesOnEdge
            i1 = index
            i2 = index+1
            i3 = index+numVerticesOnEdge
            i4 = index+numVerticesOnEdge+1

            p1 = np.array(points[i1])
            p2 = np.array(points[i2])
            p3 = np.array(points[i3])
            p4 = np.array(points[i4])



            ret.append(((i1,i2,i3,i4),(p1,p2,p4,p3)))
            
    return ret

def showSqureFace(squareVertices, origin=[0,0,0], COLOR = (255,255,0), lineWidth = 3):
    if len(squareVertices) != 4:
        print("showSqureFace : error")
        return
    
    for i in range(4):
        p1,p2 = squareVertices[i], squareVertices[(i+1)%4]
        p.addUserDebugLine(np.add(p1,origin),np.add(p2,origin),lineColorRGB=COLOR,lineWidth=lineWidth)


def debugTimer(func):
    def wrapper(*args, **kwargs):
        start_time = time()
        result = func(*args, **kwargs)
        end_time = time()
        print(f"{func.__name__} took {(end_time - start_time):.6f} seconds to execute.")
        return result
    return wrapper        

def showVerticesOnMarker(softBodyId, totalMarker):
    LENGTH = 0.05

    totalPoints,points = p.getMeshData(softBodyId,flags=1)
    verticesOnEdge = math.sqrt(totalPoints)
    numVerticesBetweenMarker = int((verticesOnEdge-2-totalMarker)/(totalMarker+1))
    excludedVertices = [0, verticesOnEdge-1]

    for i in range(totalPoints):
        x,y,z = [points[i][j] for j in range(3)]
        xi = i%verticesOnEdge
        yi = i//verticesOnEdge
        
        if (xi%(numVerticesBetweenMarker+1) == 0) and (yi%(numVerticesBetweenMarker+1) == 0) and xi not in excludedVertices and yi not in excludedVertices:
            p.addUserDebugLine([x-LENGTH,y+LENGTH,z],[x+LENGTH,y-LENGTH,z],lineColorRGB=[1,1,0])
            p.addUserDebugLine([x+LENGTH,y+LENGTH,z],[x-LENGTH,y-LENGTH,z],lineColorRGB=[1,1,0])
            p.addUserDebugText(str(i),[x,y,z], textColorRGB=[1,1,0])


def getVerticesOnMarker(softBodyId, totalMarker):

    ret = list()

    LENGTH = 0.05

    totalPoints,points = p.getMeshData(softBodyId,flags=1)
    verticesOnEdge = math.sqrt(totalPoints)
    numVerticesBetweenMarker = int((verticesOnEdge-2-totalMarker)/(totalMarker+1))
    excludedVertices = [0, verticesOnEdge-1]

    for i in range(totalPoints):
        x,y,z = [points[i][j] for j in range(3)]
        xi = i%verticesOnEdge
        yi = i//verticesOnEdge
        
        if (xi%(numVerticesBetweenMarker+1) == 0) and (yi%(numVerticesBetweenMarker+1) == 0) and xi not in excludedVertices and yi not in excludedVertices:
            ret.append((i,(x,y,z)))

    return ret