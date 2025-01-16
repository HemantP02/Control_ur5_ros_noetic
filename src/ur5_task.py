#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

def main():
    rospy.init_node('ur5_controller')
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to be registered

    msg = JointTrajectory()
    msg.header = Header()
    msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    point = JointTrajectoryPoint()
    point.positions = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point.time_from_start = rospy.Duration(1)
    msg.points.append(point)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

