#! /usr/bin/env python

import sys
import rospy
import socket
import pickle
import moveit_msgs.msg
import moveit_commander
from amps_to_angle import amps_to_angle
from geometry_msgs.msg import PoseStamped

def publish_scene(scene):
    if scene.get_objects():
        if "floor" in scene.get_objects():
            rospy.loginfo("The floor existed")
            return True
    else:
        rospy.loginfo("Building the object")
        # publish a scene
        pub_scene = PoseStamped()
        pub_scene.header.frame_id = robot.get_planning_frame()

        # add a table the block is at first
        pub_scene.pose.position.x = -0.5
        pub_scene.pose.position.y = 0
        pub_scene.pose.position.z = -0.2
        scene.add_box("floor", pub_scene, (4, 4, 0.2))
        rospy.sleep(1)
        rospy.loginfo("Added the floor")

        if scene.get_objects():
            if "floor" in scene.get_objects():
                rospy.loginfo("Add floor succeeded")
                return True
            else:
                return False
        else:
            rospy.loginfo("The scene is not pulished, start publishing again")
            publish_scene(scene)
            return False


if __name__ =='__main__':
    socketObject = socket.socket()

    socketObject.connect(("192.168.0.50", 35491))

    recvd_data = socketObject.recv(1024)


    data = pickle.loads(recvd_data)
    print(data)

    boom_minimum = [0, 0.0058]
    boom_maximum = [-1.9, 0.0169]
    stick_minimum = [0, 0.0039]
    stick_maximum = [-1.8, 0.0168]
    bucket_minimum = [0, 0.00410]
    bucket_maximum = [-3, 0.0139]

    # boom_minimum = [0, 0.006]
    # boom_maximum = [-2.24414, 0.0164]
    # stick_minimum = [0, 0.0039]
    # stick_maximum = [-1.93801, 0.0165]
    # bucket_minimum = [0, 0.00410]
    # bucket_maximum = [-3.32415, 0.0139]

    boom_angle = amps_to_angle(boom_minimum[0], boom_minimum[1], boom_maximum[0], boom_maximum[1], data[0])
    stick_angle = amps_to_angle(stick_minimum[0], stick_minimum[1], stick_maximum[0], stick_maximum[1], data[1])
    bucket_angle = amps_to_angle(bucket_minimum[0], bucket_minimum[1], bucket_maximum[0], bucket_maximum[1], data[2])
    angle_data = [boom_angle, stick_angle, bucket_angle]
    print(angle_data)
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    publish_scene(scene)
    arm = moveit_commander.MoveGroupCommander("arm")
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, 
        queue_size=1)
    group_variable_values_arm = arm.get_current_joint_values()
    group_variable_values_arm[0] = angle_data[0]
    group_variable_values_arm[1] = angle_data[1]
    group_variable_values_arm[2] = angle_data[2]
    arm.set_joint_value_target(group_variable_values_arm)
    plan = arm.plan()
    arm.go(wait=True)
    moveit_commander.roscpp_shutdown()

