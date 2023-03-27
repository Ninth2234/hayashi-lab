#!/usr/bin/env python3


import rospy
import os
from std_msgs.msg import Float32
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from include.control_table import *

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 2                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = "/dev/ttyUSB0"    # Check which port is being used on your controller

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def beforeStart():
    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def openPort(portHandler):
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()





def set_goal_pos_callback(data):
    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)

def _get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position


# This function simply get position from Dynamixel
# Noted, modulo 2**16 is preventing some data corrupted
# To check uncomment the print
def get_present_pos(dxl):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl.id, ADDR_PRESENT_POSITION)
    
    #print(dxl_comm_result, dxl_error, bin(dxl_present_position), dxl_present_position,dxl_present_position%2**16) 

    return dxl_present_position%2**16

def get_present_load(dxl):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl.id, ADDR_PRESENT_LOAD)
    
    print(hex(dxl_present_position), dxl_present_position, ((-1)**(dxl_present_position//2**10%2))*(dxl_present_position%2**9)) 

    return dxl_present_position

def read_write_py_node():
    rospy.init_node('read_write_py_node')
    rospy.Subscriber('set_position', SetPosition, set_goal_pos_callback)
    rospy.Service('get_position', GetPosition, get_present_pos)
    rospy.spin()

def main():
    beforeStart()
    openPort(portHandler)

    # Enable Dynamixel Torque
    print(DXL_ID, portHandler.baudrate)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL has been successfully connected")

    print("Ready to get & set Position.")

 #   read_write_py_node()


class dxl:
    def __init__(self, portHandler, id):
        self.id = id
        self.port = portHandler
        self.publisher()

    def publisher(self):
        rospy.init_node('dxl_state_publisher')
        pub_pos = rospy.Publisher('position', Float32, queue_size=10)
        pub_load = rospy.Publisher('load', Float32, queue_size=10)
        rospy.Subscriber('set_position', SetPosition, set_goal_pos_callback)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            pub_pos.publish(get_present_pos(self))            
            pub_load.publish(get_present_load(self))
            
            rate.sleep()




if __name__ == '__main__':
    main()
    print("heay")
    dxl(portHandler, 2)