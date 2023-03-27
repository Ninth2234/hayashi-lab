import pybullet as p
import time
import math
import pybullet_data



##URDF_PATH = "/home/hayashi/catkin_ws/src/tactile_simulation/urdf/plane.urdf"
URDF_PATH = "/home/hayashi/catkin_ws/src/tactile_simulation/scripts/utils/ContactTip.urdf"


class PrismaticJoint:
  def __init__(self, jointIndex, jointName, jointLowerLimit, jointUpperLimit):
    self.jointIndex = jointIndex
    self.jointName = jointName.decode('utf-8')
    self.jointLowerLimit = jointLowerLimit
    self.jointUpperLimit = jointUpperLimit
    self.debugParamId:int = 0

    return
  


class ContactTip:
  def __init__(self, basePosition):
    self.id = p.loadURDF(URDF_PATH, basePosition, [0,0,0,1], 0)
    self.joints = []

    self.buttonId = p.addUserDebugParameter(" manual control", 3,1,0)
    self.isManual = 0
    
    p.createConstraint(self.id,-1,-1,-1,p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], basePosition)    
  
    for i in range(p.getNumJoints(self.id)):
      jointIndex, jointName, jointType, _, _, _, _, _, jointLowerLimit, jointUpperLimit,*_ = p.getJointInfo(self.id,i)
      
      if jointType == p.JOINT_PRISMATIC:
        self.joints.append(PrismaticJoint(jointIndex, jointName, jointLowerLimit, jointUpperLimit))

    for j in self.joints:
      j.debugParamId = p.addUserDebugParameter(" "+j.jointName,j.jointLowerLimit,j.jointUpperLimit)

  def update(self):
    if p.readUserDebugParameter(self.buttonId)%2 == 0:
      self.isManual = 0
    else :
      self.isManual = 1

    if self.isManual:
      for j in self.joints:
        desiredPos =  p.readUserDebugParameter(j.debugParamId)        
        p.setJointMotorControl2(self.id, j.jointIndex, p.POSITION_CONTROL, desiredPos)
    
  def setPosition(self, x, y, z):
    if not self.isManual:
      desiredPos = [z, y, x]
      for j in self.joints:
        p.setJointMotorControl2(self.id, j.jointIndex, p.POSITION_CONTROL, desiredPos.pop(),maxVelocity=0.5)

      






if __name__ == '__main__':
  BASE_POSITION = [0, 0 ,2]

  p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.loadURDF("plane.urdf")
  p.setGravity(0, 0, -10)

  contactTip = ContactTip(BASE_POSITION)

  while 1:
    
    contactTip.update()

    p.stepSimulation()
    time.sleep(1./240.)


