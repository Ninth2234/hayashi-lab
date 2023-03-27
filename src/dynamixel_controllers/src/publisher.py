#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from dynamixel_workbench_msgs.srv import DynamixelCommand

class dynaxiel_state_publisher:
    def __init__(self):
        self.pub_pos = rospy.Publisher('position', Float32, queue_size = 10)
        self.pub_vel = rospy.Publisher('velocity', Float32, queue_size = 10)
        self.pub_effort = rospy.Publisher('effort', Float32, queue_size = 10)
        self.pub_current = rospy.Publisher('current', Float32, queue_size = 10)
        self.loop()

    def callback(self,data):
        self.pub_pos.publish(data.position[0])
        self.pub_vel.publish(data.velocity[0])
        self.pub_effort.publish(data.effort[0])

        #print(data)


    def loop(self):
        rospy.wait_for_service("/dynamixel_workbench/dynamixel_command")
        print("I'm listening.")
        rospy.init_node('dynamixel_state_publisher')
        rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, self.callback)
        rospy.spin()


if __name__ == '__main__':
    dynaxiel_state_publisher()
