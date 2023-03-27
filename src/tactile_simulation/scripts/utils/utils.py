import pybullet as p
import math

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
def showVertices(softBodyId):
    LENGTH = 0.05

    totalPoints,points = p.getMeshData(softBodyId,flags=1)
    #position,quarternion = p.getBasePositionAndOrientation(clothId)    

    for i in range(totalPoints):
        x,y,z = [points[i][j] for j in range(3)]

        p.addUserDebugLine([x-LENGTH,y+LENGTH,z],[x+LENGTH,y-LENGTH,z],lineColorRGB=[1,1,0])
        p.addUserDebugLine([x+LENGTH,y+LENGTH,z],[x-LENGTH,y-LENGTH,z],lineColorRGB=[1,1,0])
        p.addUserDebugText(str(i),[x,y,z], textColorRGB=[1,1,0])

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