#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3

JOINT_VALUES = []

def callback(msg):
    global JOINT_VALUES
    rospy.loginfo('Getting the joint values')
    joint_value = [msg.x, msg.y, msg.z]
    JOINT_VALUES.append(joint_value)
    rospy.loginfo(f'Outputing the list of joint values {joint_value}')

def listener():
    rospy.loginfo('Initing joint_values node')
    rospy.init_node('get_joint_values')
    sub = rospy.Subscriber('/planned_joint_positions', Vector3, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
    print(JOINT_VALUES)
