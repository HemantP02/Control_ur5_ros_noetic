#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import numpy as np

# ***** Coefficients *****
global d1, a2, a3, a7, d4, d5, d6
d1 =  0.1273
a2 = -0.612
a3 = -0.5723
a7 = 0.075
d4 =  0.163941
d5 =  0.1157
d6 =  0.0922

d = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) 
a = np.array([0, -0.425, -0.39225, 0, 0, 0]) 
alph = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])  

current_joint_positions = [0.0] * 6  # Initialize with zeros
traj_pub = None  # Initialize the traj_pub variable

# Callback function to get current joint positions
def joint_states_callback(msg):
    global current_joint_positions
    current_joint_positions = msg.position

# ***** Forward Kinematics *****
def AH(n, th, c):
    T_a = np.identity(4)
    T_a[0,3] = a[n-1]
    T_d = np.identity(4)
    T_d[2,3] = d[n-1]

    Rzt = np.array([[np.cos(th[n-1,c]), -np.sin(th[n-1,c]), 0, 0],
                    [np.sin(th[n-1,c]), np.cos(th[n-1,c]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    Rxa = np.array([[1, 0, 0, 0],
                    [0, np.cos(alph[n-1]), -np.sin(alph[n-1]), 0],
                    [0, np.sin(alph[n-1]), np.cos(alph[n-1]), 0],
                    [0, 0, 0, 1]])

    A_i = np.dot(T_d, np.dot(Rzt, np.dot(T_a, Rxa)))

    return A_i

def HTrans(th, c):  
    A_1 = AH(1, th, c)
    A_2 = AH(2, th, c)
    A_3 = AH(3, th, c)
    A_4 = AH(4, th, c)
    A_5 = AH(5, th, c)
    A_6 = AH(6, th, c)
      
    T_06 = np.dot(A_1, np.dot(A_2, np.dot(A_3, np.dot(A_4, np.dot(A_5, A_6)))))

    return T_06

# Function to compute inverse kinematics (joint positions from end-effector position)
def invKine(desired_pos):
    th = np.zeros((6, 8))
    P_05 = (np.dot(desired_pos, np.array([0, 0, -d6, 1]).T) - np.array([0, 0, 0, 1 ]))
  
    # **** theta1 ****
    psi = np.arctan2(P_05[2-1], P_05[1-1])
    phi = np.arccos(d4 / np.sqrt(P_05[2-1]**2 + P_05[1-1]**2))
    th[0, 0:4] = np.pi/2 + psi + phi
    th[0, 4:8] = np.pi/2 + psi - phi
  
    # **** theta5 ****
    cl = [0, 4] # wrist up or down
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = T_10 @ desired_pos
        th[4, c:c+2] = np.arccos((T_16[2,3]-d4)/d6)
        th[4, c+2:c+4] = -np.arccos((T_16[2,3]-d4)/d6)

    # **** theta6 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = np.linalg.inv(T_10 @ desired_pos)
        th[5, c:c+2] = np.arctan2((-T_16[1,2]/np.sin(th[4, c])), (T_16[0,2]/np.sin(th[4, c])))

    # **** theta3 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = AH(6, th, c)
        T_54 = AH(5, th, c)
        T_14 = (T_10 @ desired_pos) @ np.linalg.inv(T_54 @ T_65)
        P_13 = T_14 @ np.array([0, -d4, 0, 1]).T - np.array([0, 0, 0, 1]).T
        t3 = np.arccos((np.linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3))
        th[2, c] = t3
        th[2, c+1] = -t3

    # **** theta2 and theta 4 ****
    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = np.linalg.inv(AH(6, th, c))
        T_54 = np.linalg.inv(AH(5, th, c))
        T_14 = (T_10 @ desired_pos) @ T_65 @ T_54
        P_13 = T_14 @ np.array([0, -d4, 0, 1]).T - np.array([0, 0, 0, 1]).T
        th[1, c] = -np.arctan2(P_13[1], -P_13[0]) + np.arcsin(a3* np.sin(th[2,c])/np.linalg.norm(P_13))
        T_32 = np.linalg.inv(AH(3, th, c))
        T_21 = np.linalg.inv(AH(2, th, c))
        T_34 = T_32 @ T_21 @ T_14
        th[3, c] = np.arctan2(T_34[1,0], T_34[0,0])
  
    return th

def invKine_xyz(x, y, z):
    desired_pos = np.array([[1, 0, 0, x],
                            [0, 1, 0, y],
                            [0, 0, 1, z],
                            [0, 0, 0, 1]])

    return invKine(desired_pos)

# Function to move the arm to a desired position (end-effector position)
def move_arm_to_position(x, y, z):
    global current_joint_positions, traj_pub

    # Compute desired joint positions using inverse kinematics
    desired_joint_positions = invKine_xyz(x, y, z)

    print("Desired Joint Positions:", desired_joint_positions)  # Debug print

    # Convert numpy array to list
    desired_joint_positions_list = desired_joint_positions.tolist()

    print("Desired Joint Positions List:", desired_joint_positions_list)  # Debug print

    # Replace nan values with 0.0
    for pos in desired_joint_positions_list:
        for i in range(len(pos)):
            if np.isnan(pos[i]):
                pos[i] = 0.0

    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # Populate trajectory point with desired joint positions
    point = JointTrajectoryPoint()
    point.positions = sum(desired_joint_positions_list, [])
    point.time_from_start = rospy.Duration(1.0)  # Adjust as needed

    # Add trajectory point to the message
    traj_msg.points.append(point)

    print("Trajectory Message:", traj_msg)  # Debug print

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
    desired_x = 0.5
    desired_y = 0.1
    desired_z = 0.1

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
