#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

# Motor swing down until its sense touch and stay stationary for a while.
# Then, put the finger up and do it again.

# ( ) control the swing

THIS_MOTOR_ID = 2
SLEEP_TIME = 2

LOWER_POSITION = 1000
UPPER_POSITION = 3000

LOWER_LIMIT = 0
UPPER_LIMIT = 4000

class finger:
    def __init__(self):
        self._sendCommand = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)
        print(self.sendCommand('CW_Angle_Limit', LOWER_LIMIT))
        print(self.sendCommand('CCW_Angle_Limit', UPPER_LIMIT))
        self.loop()

    def sendCommand(self,addr_name, value):
        return self._sendCommand(id =THIS_MOTOR_ID, addr_name=addr_name, value=value)

    def loop(self):
        rospy.wait_for_service("/dynamixel_workbench/dynamixel_command")
        while not rospy.is_shutdown():            
            moving_speed = rospy.get_param("/dxl/moving_speed",1000)
            torque_limit = rospy.get_param("/dxl/torque_limit",1000)
            self.sendCommand('Moving_Speed', moving_speed)
            self.sendCommand('Torque_Limit', torque_limit)

            self.sendCommand('Goal_Position',LOWER_POSITION)
            rospy.sleep(SLEEP_TIME)    
            self.sendCommand('Goal_Position',UPPER_POSITION)
            rospy.sleep(SLEEP_TIME)


if __name__ == '__main__':
    finger()
