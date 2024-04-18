#! /usr/bin/env python3
import sys
import rospy
import argparse
import moveit_msgs.msg
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Vector3 
from tf.transformations import quaternion_from_euler
from moveit_commander import RobotCommander, MoveGroupCommander, \
    PlanningSceneInterface, roscpp_initialize, roscpp_shutdown


class ControlRobot():
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('pub_planned_joints')
        self.joint_publisher = rospy.Publisher('planned_joint_positions', Vector3, queue_size=10)
        self.joint_count_publisher = rospy.Publisher('planned_joint_values_count', Int32, queue_size=10)
        self.rate = rospy.Rate(6)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, 
            queue_size=1)
        # self.joint1_angle = [[0, 0.0055], [-2, 0.0169]]
        # self.joint2_angle = [[0, 0.0039], [-1.93801, 0.0165]]
        # self.joint3_angle = [[0, 0.00410], [-3.32415, 0.0139]]
        self.joint1_angle = [[0, 0.0055], [-1.9, 0.0169]]
        self.joint2_angle = [[0, 0.0039], [-1.8, 0.0165]]
        self.joint3_angle = [[0, 0.00410], [-3, 0.0139]]

        self.boom_min_max = [0.0055, 0.0169]
        self.stick_min_max = [0.0039, 0.0165]
        self.bucket_min_max = [0.00410, 0.0139]

        # self.joint1_angle = [[0, 0.0055], [-2.24414, 0.0169]]
        # self.joint2_angle = [[0, 0.0039], [-1.93801, 0.0165]]
        # self.joint3_angle = [[0, 0.00410], [-3.32415, 0.0139]]

        # self.boom_min_max = [0.0055, 0.0169]
        # self.stick_min_max = [0.0039, 0.0165]
        # self.bucket_min_max = [0.00410, 0.0139]

    def set_target(self, x, y, z, roll, pitch, yaw):
        pose_target = Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]
        return pose_target

    def plan(self, x, y, z, roll, pitch, yaw):
        pose_target = self.set_target(x, y, z, roll, pitch, yaw)
        self.group.set_pose_target(pose_target)
        self.group.allow_replanning(True)
        self.group.allow_looking(True)
        plan = self.group.plan()
        rospy.sleep(3)
        self.group.go(wait=True)
        return plan

    def interpolation(self, d, x, y):
        output = d[0][1] + (x - d[0][0]) * ((d[1][1] - d[0][1]) / (d[1][0] - d[0][0]))
        # y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        if output > y[1]:
            return y[1]
        elif output < y[0]:
            return y[0]
        else:
            return output

    def pub_positions(self, plan):
        planned_num = len(plan[1].joint_trajectory.points)
        joint_count = int((planned_num+1)/6)+1
        print(f"{joint_count} joint values would be generated")
        self.joint_count_publisher.publish(joint_count)
        i = 0
        for datas in plan[1].joint_trajectory.points:
            if i%6 != 0 and i != planned_num -1:
                i += 1
                continue
            # Build a Vector3 message type, ***NOTICE***: the x, y, z
            # values are the joint values for Joint1, Joint2, and Joint3
            pub_msg = Vector3()
            list_positions = list(datas.positions)
            print(list_positions)

            # Finding the interpolation

            boom_ma = self.interpolation(self.joint1_angle, list_positions[0],  self.boom_min_max)
            stick_ma = self.interpolation(self.joint2_angle, list_positions[1], self.stick_min_max)
            bucket_ma = self.interpolation(self.joint3_angle, list_positions[2], self.bucket_min_max)

            pub_msg.x = boom_ma
            pub_msg.y = stick_ma
            pub_msg.z = bucket_ma
            print(f"This is the {i}th value that is published.\n {pub_msg}")
            self.joint_publisher.publish(pub_msg)
            i += 1
            self.rate.sleep()

    def go(self, plan):
        self.group.go(wait=True)


if __name__=='__main__':
    # Get the position and the value of roll, pitch, and yaw.
    parser = argparse.ArgumentParser()
    parser.add_argument("-x", "--position_x", type=float, required=True, 
                        help="an float for the position of x")
    parser.add_argument("-y", "--position_y", type=float, required=True,
                        help="an float for the position of y")
    parser.add_argument("-z", "--position_z", type=float, required=True,
                        help="an float for the position of z")
    parser.add_argument("-r", "--roll", type=float, required=True,
                        help="an float for the value of roll")
    parser.add_argument("-p", "--pitch", type=float, required=True,
                        help="aan float for the value of pitch")
    parser.add_argument("-ya", "--yaw", type=float, required=True,
                        help="an float for the value of yaw")
    args = parser.parse_args()
    control_robot = ControlRobot()
    # Assign the position and the value of roll, pitch, and yaw.
    plan = control_robot.plan(args.position_x, args.position_y,
        args.position_z, args.roll, args.pitch, args.yaw)
    control_robot.pub_positions(plan)
    control_robot.go(plan)
    roscpp_shutdown()
