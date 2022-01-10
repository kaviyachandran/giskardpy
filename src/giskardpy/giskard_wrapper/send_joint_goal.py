import rospy

import actionlib
from giskard_msgs.msg import MoveCmd, JointConstraint
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal
from giskard_msgs.msg import MoveResult


def send_joint_goal():
    action_client = actionlib.SimpleActionClient("/giskard/command", MoveAction)

    action_client.wait_for_server()
    goal = MoveGoal()

    goal.type = 5 ### Plan_and_execute

    joint_goal = MoveCmd()

    joint_constraints = JointConstraint()
    joint_constraints.type = JointConstraint.JOINT
    #joint_constraints.goal_state.header.frame_id = "map"
    #joint_constraints.goal_state.header.stamp = rospy.Time.now()
    joint_constraints.goal_state.name = ['torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'r_wrist_flex_joint',
                                         'r_wrist_roll_joint', 'r_elbow_flex_joint']
    joint_constraints.goal_state.position = [0.21, 0.17, 0.12, -0.91, 0.65, -1.02]
    joint_goal.joint_constraints.append(joint_constraints)

    goal.cmd_seq.append(joint_goal)
    action_client.send_goal(goal)
    action_client.wait_for_result()

    result = action_client.get_result()

    if result.error_codes[0] == MoveResult.SUCCESS:
        print('giskard returned success')
    else:
        print('something went wrong')


if __name__ == "__main__":
    try:
        rospy.init_node('action_client_send_joint_goal')
        send_joint_goal()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
