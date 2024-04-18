import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
# input the group name of moving here
group = moveit_commander.MoveGroupCommander("arm")
# Show the trajectory of the planned path that is calculated
print("Start getting information ======================")
# print("Robot Groups:")
# print(robot.get_group_names())
# print("Reference frame: %s" % group.get_planning_frame())
# print("End effector: %s" % group.get_end_effector_link())
print("Current Pose:")
print(group.get_current_pose())
print("Robot State:")
print(robot.get_current_state())
print("All desired information has shown================")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory, 
    queue_size=1)
pose_target = geometry_msgs.msg.Pose()
# the goal value
print(pose_target)

# pose_target.position.x = 0.0
# pose_target.position.y = 0.0
# pose_target.position.z = 0.7
# pose_target.orientation.x = 0.0
# pose_target.orientation.y = 0.0
# pose_target.orientation.z = 0.0
# pose_target.orientation.w = 1.0

# pose_target.position.x = 0.0
# pose_target.position.y = 0.0
# pose_target.position.z = 0.75
# pose_target.orientation.x = 0.707
# pose_target.orientation.y = 1.0693821421304017e-05
# pose_target.orientation.z = 1.0693821421304017e-05
# pose_target.orientation.w = 0.707

# pose_target.position.x = -1.7922248041441333
# pose_target.position.y = 0
# pose_target.position.z = 1.284597238617806
# pose_target.orientation.x = -0.0
# pose_target.orientation.y = -0.9513997916139328
# pose_target.orientation.z = 0.0
# pose_target.orientation.w = 0.3079584980431052

# group.set_pose_target(pose_target)
# print(pose_target)
# group.allow_replanning(True)


group_variable_values_arm = group.get_current_joint_values()
group_variable_values_arm[0] = -0.5
group_variable_values_arm[1] = -0.5
group_variable_values_arm[2] = -1.5708
group.set_joint_value_target(group_variable_values_arm)

# group.allow_looking(True)
plan = group.plan()
rospy.sleep(10)
group.go(wait=True)
moveit_commander.roscpp_shutdown()