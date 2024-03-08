#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from gazebo_msgs.msg import LinkStates
import math
import itertools

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller_node', anonymous=True)
        self.pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.end_effector_callback)
        rospy.Subscriber("/camera1/image_raw", Image, self.image_callback)
        rospy.sleep(1)  # Wait for publisher to connect
        self.end_effector_position = None
        self.image_data = None
        bridge = CvBridge()
        self.joint_set_dataset = []

    def end_effector_callback(self, data):
        wrist_3_index = data.name.index("robot::wrist_3_link")
        self.end_effector_position = data.pose[wrist_3_index].position
    
    def image_callback(self, data):
        self.image_data = data

    def move_arm(self, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.header = Header()
        traj_msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start = rospy.Duration(1.0)  # Adjust duration as needed

        traj_msg.points.append(point)

        self.pub.publish(traj_msg)
        rospy.loginfo("Sent trajectory command to move arm.")

    def move_arm_to_zero(self):
        zero_positions = [0.0] * 6
        self.move_arm(zero_positions)

    def generate_joint_set_dataset(self):
        # Define the possible values for the middle four joints
        values = [-3.92699, -3.14159, -2.35619, -1.5708, -0.785398, 0, 0.785398, 1.5708, 2.35619, 3.14159, 3.92699]

        # Generate all possible combinations for the middle four joints
        combinations = itertools.product(values, repeat=4)

        # Iterate over each combination
        for combination in combinations:
            # Insert 0 at the beginning and end of the combination
            joint_positions = [0] + list(combination) + [0]
            print("Moving arm to zero positions...")
            self.move_arm_to_zero()  # Move to zero position first
            rospy.sleep(2)  # Adjust sleep duration for simulation to stabilize
            print("Moving arm to new joint angles:", joint_positions)
            self.move_arm(joint_positions)  # Then move to new joint angles
            rospy.sleep(2)  # Adjust sleep duration for simulation to stabilize
            if self.image_data is not None:
                bridge = CvBridge()
                image_cv = bridge.imgmsg_to_cv2(self.image_data, desired_encoding="bgr8")
                cv2.imwrite("image_" + str(len(self.joint_set_dataset)) + ".jpg", image_cv)
                rospy.loginfo("Saved image: %s", "image_" + str(len(self.joint_set_dataset)) + ".jpg")

            if self.end_effector_position:
                self.joint_set_dataset.append((joint_positions, self.end_effector_position))
                rospy.loginfo("Recorded joint set: %s, End effector position: %s", joint_positions, self.end_effector_position)
            else:
                rospy.logwarn("End effector position not received.")

if __name__ == '__main__':
    arm_controller = ArmController()
    arm_controller.generate_joint_set_dataset()
    rospy.spin()
