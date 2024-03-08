#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

def move_arm(joint_positions):
    rospy.init_node('move_arm_node', anonymous=True)
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Wait for publisher to connect

    traj_msg = JointTrajectory()
    traj_msg.header = Header()
    traj_msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.velocities = [0.0] * len(joint_positions)
    point.time_from_start = rospy.Duration(1.0)  # Adjust duration as needed

    traj_msg.points.append(point)

    pub.publish(traj_msg)
    rospy.loginfo("Sent trajectory command to move arm.")

if __name__ == '__main__':
    try:
        # Specify the desired joint positions for the UR5 arm
        target_joint_positions_0 = [0, -2.37, 1.57, 1.57, 1.570, 0]  # Adjust as needed
        target_joint_positions_1 = [0, -1.57, 1.57, 1.57, 1.570, 0]
        target_joint_positions_2 = [0, -.77, 1.57, 1.57, 1.570, 0]
        target_joint_positions_3 = [0, -1.57, .77, 1.57, 1.570, 0]
        target_joint_positions_4 = [0, -1.57, 0.0, 1.57, 1.570, 0]
        target_joint_positions_5 = [0, -1.57, -.77, 1.57, 1.570, 0]
        target_joint_positions_6 = [0, -1.57, -1.57, 1.57, 1.570, 0]
        target_joint_positions_7 = [0, -1.57, 1.57, 1.57, .00, 0]
        # Move the arm to the desired joint positions
        move_arm(target_joint_positions_1)
    except rospy.ROSInterruptException:
        pass


# import itertools

# # Define the possible values for each element
# values = [-3.92699, -3.14159, -2.35619, -1.5708, -0.785398, 0, 0.785398, 1.5708, 2.35619, 3.14159, 3.92699]

# # Generate all possible combinations
# combinations = list(itertools.product(values, repeat=6))

# # Print the combinations
# for combination in combinations:
#     print(combination)




# class SearchAlgorithm:
#     def __init__(self, joint_set_dataset):
#         self.joint_set_dataset = sorted(joint_set_dataset, key=lambda x: x[1].distance(target_position))

#     def binary_search(self, target_position):
#         low = 0
#         high = len(self.joint_set_dataset) - 1
#         while low <= high:
#             mid = (low + high) // 2
#             if self.joint_set_dataset[mid][1] == target_position:
#                 return self.joint_set_dataset[mid][0]
#             elif self.joint_set_dataset[mid][1] < target_position:
#                 low = mid + 1
#             else:
#                 high = mid - 1
        
#         # If exact match not found, return the closest point
#         return self.joint_set_dataset[low][0] if self.joint_set_dataset[low][1].distance(target_position) < self.joint_set_dataset[high][1].distance(target_position) else self.joint_set_dataset[high][0]

#     def interpolation_search(self, target_position):
#         low = 0
#         high = len(self.joint_set_dataset) - 1
#         while low <= high and self.joint_set_dataset[low][1] <= target_position <= self.joint_set_dataset[high][1]:
#             # Estimate the position using interpolation formula
#             mid = low + ((high - low) // (self.joint_set_dataset[high][1].distance(self.joint_set_dataset[low][1])) * (target_position - self.joint_set_dataset[low][1]).distance())

#             if self.joint_set_dataset[mid][1] == target_position:
#                 return self.joint_set_dataset[mid][0]
#             elif self.joint_set_dataset[mid][1] < target_position:
#                 low = mid + 1
#             else:
#                 high = mid - 1
        
#         # If exact match not found, return the closest point
#         return self.joint_set_dataset[low][0] if self.joint_set_dataset[low][1].distance(target_position) < self.joint_set_dataset[high][1].distance(target_position) else self.joint_set_dataset[high][0]



