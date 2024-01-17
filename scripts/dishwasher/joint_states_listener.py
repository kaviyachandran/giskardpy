import rospy
from sensor_msgs.msg import JointState

import csv


def callback(data):
    joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint",
              "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint", "l_gripper_l_finger_joint"]
    # for (i, d) in enumerate(data.name):
    #     if d in joints:
    #         print(d)
    #         print(data.position[i])
    #         print(data.velocity[i])
    #         print(data.effort[i])
    #         print("-----")
    with open('joint_values.csv', 'a', newline="") as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=' ')
        for (i, d) in enumerate(data.name):
            if d in joints:
                csv_writer.writerow([d, data.position[i], data.velocity[i], data.effort[i]])


def listener():
    rospy.init_node("joint_state_listener")
    sub = rospy.Subscriber("/mujoco/robot_joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
