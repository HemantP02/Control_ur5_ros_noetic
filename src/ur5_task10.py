#!/usr/bin/env python

import rospy
from math import atan2, sqrt, cos,acos, sin, pi
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import numpy as np

# DH parameters for UR5
DH_PARAMS = [
    [0, 0, 0.089159, np.pi/2],
    [0, -0.425, 0, 0],
    [0, -0.39225, 0, 0],
    [0, 0.10915, 0, np.pi/2],
    [0, 0, 0.09465, -np.pi/2],
    [0, 0, 0.0823, 0]
]

# Joint limits for UR5 (in radians)
JOINT_LIMITS = {
    'shoulder_pan_joint': (-np.pi, np.pi),
    'shoulder_lift_joint': (-np.pi / 2, np.pi / 2),
    'elbow_joint': (-np.pi, np.pi),
    'wrist_1_joint': (-np.pi, np.pi),
    'wrist_2_joint': (-np.pi, np.pi),
    'wrist_3_joint': (-np.pi, np.pi)
}
current_joint_positions = [0.0] * 6  # Initialize with zeros
traj_pub = None  # Initialize the traj_pub variable

# Callback function to get current joint positions
def joint_states_callback(msg):
    global current_joint_positions
    current_joint_positions = msg.position

# Function to compute forward kinematics (end-effector position from joint positions)
def forward_kinematics(joint_positions):
    theta1, theta2, theta3, theta4, theta5, theta6 = joint_positions

    x = cos(theta1)*(cos(theta2 + theta3)*cos(theta4)*cos(theta5) - sin(theta4)*sin(theta5))*(0.089159 + 0.425*cos(theta2) - 0.39225*sin(theta2 + theta3)) - cos(theta1)*(sin(theta2 + theta3)*cos(theta4)*cos(theta5) + cos(theta2 + theta3)*sin(theta4))*(0.10915*sin(theta5) + 0.09465*cos(theta5) + 0.0823) - sin(theta1)*(sin(theta2 + theta3)*cos(theta4)*cos(theta5) + cos(theta2 + theta3)*sin(theta4))*(0.10915*cos(theta5) - 0.09465*sin(theta5)) - sin(theta1)*(cos(theta2 + theta3)*cos(theta4)*cos(theta5) - sin(theta4)*sin(theta5))*(0.089159 + 0.425*cos(theta2) - 0.39225*sin(theta2 + theta3))
    y = cos(theta1)*(cos(theta2 + theta3)*cos(theta4)*sin(theta5) + cos(theta4)*cos(theta5)*sin(theta2 + theta3))*(0.089159 + 0.425*cos(theta2) - 0.39225*sin(theta2 + theta3)) - cos(theta1)*(sin(theta2 + theta3)*cos(theta4)*sin(theta5) - cos(theta4)*cos(theta5)*cos(theta2 + theta3))*(0.10915*sin(theta5) + 0.09465*cos(theta5) + 0.0823) - sin(theta1)*(sin(theta2 + theta3)*cos(theta4)*sin(theta5) - cos(theta4)*cos(theta5)*cos(theta2 + theta3))*(0.10915*cos(theta5) - 0.09465*sin(theta5)) - sin(theta1)*(cos(theta2 + theta3)*cos(theta4)*sin(theta5) + cos(theta4)*cos(theta5)*sin(theta2 + theta3))*(0.089159 + 0.425*cos(theta2) - 0.39225*sin(theta2 + theta3))
    z = -L1*sin(theta2 + theta3)*(0.089159 + 0.425*cos(theta2) - 0.39225*sin(theta2 + theta3)) + cos(theta2 + theta3)*(0.10915*sin(theta5) + 0.09465*cos(theta5) + 0.0823) + D4*sin(theta4) + L2*cos(theta2 + theta3)

    return Point(x, y, z)



# Function to compute inverse kinematics (joint positions from end-effector position)
# def inverse_kinematics(end_effector_position):
#     x, y, z = end_effector_position.x, end_effector_position.y, end_effector_position.z

#     # Calculate theta1 (shoulder_pan_joint)
#     theta1 = atan2(y, x)

#     # Calculate distances and angles
#     d = sqrt(x*x + y*y) - L1
#     r = sqrt(d*d + (z - D4)*(z - D4))
#     alpha = atan2(z - D4, d)
#     beta = atan2(D4, d)

#     # Calculate theta3 (elbow_joint)
#     theta3 = pi - alpha - beta

#     # Calculate theta2 (shoulder_lift_joint)
#     q1 = L2 + L1*cos(theta3)
#     q2 = L1*sin(theta3)
#     theta2 = atan2(q1, q2) - theta3
#     # Calculate remaining joint angles
#     theta4 = 0.0  # Assume 0 for simplicity
#     theta5 = 0.0  # Assume 0 for simplicity
#     theta6 = 0.0  # Assume 0 for simplicity

#     return [theta1, theta2, theta3, theta4, theta5, theta6]

def inverse_kinematics(end_effector_position):
    x, y, z = end_effector_position.x, end_effector_position.y, end_effector_position.z
    # Define desired position
    px = x - DH_PARAMS[5][1] * np.cos(z)
    py = y - DH_PARAMS[5][1] * np.sin(z)
    # Calculate theta1 (shoulder_pan_joint)
    theta1 = atan2(py, px)
    # Calculate distances and angles
    d = sqrt(px*px + py*py)
    r = sqrt(d*d + (z - DH_PARAMS[0][2])*(z - DH_PARAMS[0][2]))
    alpha = atan2(z - DH_PARAMS[0][2], d)
    beta = atan2(DH_PARAMS[0][2], d)
    # Calculate theta3 (elbow_joint)
    theta3 = np.pi - alpha - beta
    # Calculate theta2 (shoulder_lift_joint)
    q1 = DH_PARAMS[2][1] + DH_PARAMS[1][1] * np.cos(theta3)
    q2 = DH_PARAMS[1][1] * np.sin(theta3)
    theta2 = atan2(r - DH_PARAMS[1][1] * np.sin(theta3), d - DH_PARAMS[1][1] * np.cos(theta3)) - theta3
    # Calculate theta4 (wrist_1_joint)
    theta4 = atan2(end_effector_position.z - DH_PARAMS[3][2], d - DH_PARAMS[1][1] * cos(theta3) - DH_PARAMS[2][2])
    # Calculate theta5 (wrist_2_joint)
    theta5 = acos(sin(theta4) * sin(DH_PARAMS[3][0]))
    # Calculate theta6 (wrist_3_joint)
    theta6 = atan2(-cos(theta4) * sin(DH_PARAMS[3][0]), sin(theta4) * cos(theta5))
    return [theta1, theta2, theta3, theta4, theta5, theta6]

# Function to move the arm to a desired position (end-effector position)
def move_arm_to_position(x, y, z):
    global current_joint_positions, traj_pub

    # Compute desired joint positions using inverse kinematics
    desired_joint_positions = inverse_kinematics(Point(x, y, z))

    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # Populate trajectory point with desired joint positions
    point = JointTrajectoryPoint()
    point.positions = desired_joint_positions
    point.time_from_start = rospy.Duration(1.0)  # Adjust as needed

    # Add trajectory point to the message
    traj_msg.points.append(point)

    # Publish trajectory message to move the arm
    traj_pub.publish(traj_msg)

def main():
    global traj_pub

    rospy.init_node('ur5_controller')

    # Initialize publisher for trajectory command
    traj_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

    # Subscribe to joint states topic
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    rospy.sleep(1)  # Wait for the publisher to be registered

    # Wait for joint_states to be available
    rospy.wait_for_message('/joint_states', JointState)

    # Define desired position (x, y, z)
    desired_x = 0.750
    desired_y = 0.0
    desired_z = .20

    # Move arm to desired position
    move_arm_to_position(desired_x, desired_y, desired_z)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
