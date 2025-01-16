#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def move_robot_to_pose(pose):
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_demo', anonymous=True)

    # Connect to the MoveIt interfaces
    robot = moveit_commander.RobotCommander()
    group_name = "manipulator"
    # Add the UR5 robot namespace before the group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose = pose

    # Set the target pose for the end effector
    move_group.set_pose_target(target_pose)

    # Plan the motion
    plan = move_group.plan()

    # Execute the motion
    move_group.execute(plan)

if __name__ == '__main__':
    try:
        # Define the desired goal pose (modify as needed)
        goal_pose = Pose()
        goal_pose.position.x = 0.5
        goal_pose.position.y = 0.5
        goal_pose.position.z = 0.5

        # Call the function to move the robot to the desired pose
        move_robot_to_pose(goal_pose)

    except rospy.ROSInterruptException:
        pass
