import rospy
import actionlib
from geometry_msgs.msg import PoseStamped

from giskard_msgs.msg import MoveCmd, CartesianConstraint
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal
from giskard_msgs.msg import MoveResult


def send_cartesian_goal() :
    client_sending_cartesian_goal = actionlib.SimpleActionClient('/giskard/command', MoveAction)
    client_sending_cartesian_goal.wait_for_server()

    action_goal = MoveGoal()
    action_goal.type = MoveGoal.PLAN_AND_EXECUTE

    goal = MoveCmd()

    cartesian_goal = CartesianConstraint()
    cartesian_goal.type = CartesianConstraint.POSE_6D
    cartesian_goal.root_link = "l_gripper_tool_frame"
    cartesian_goal.tip_link = "cuboid_cup"

    x_root_tip = PoseStamped()
    x_root_tip.header.stamp = rospy.Time.now()
    x_root_tip.header.frame_id = "l_gripper_tool_frame"

    x_root_tip.pose.position.x = 0.531
    x_root_tip.pose.position.y = 1.047
    x_root_tip.pose.position.z = 0.193

    x_root_tip.pose.orientation.x = 0
    x_root_tip.pose.orientation.y = 0
    x_root_tip.pose.orientation.z = 0
    x_root_tip.pose.orientation.w = 1
    cartesian_goal.goal = x_root_tip

    goal.cartesian_constraints.append(cartesian_goal)
    action_goal.cmd_seq.append(goal)


    client_sending_cartesian_goal.send_goal(action_goal)
    client_sending_cartesian_goal.wait_for_result()

    result = client_sending_cartesian_goal.get_result()

    if(result.error_codes[0] == MoveResult.SUCCESS) :
        print('giskard returned success')
    else:
        print('something went wrong')

if __name__ == "__main__":
    try:
        rospy.init_node('action_client_send_cartesian_goal')
        send_cartesian_goal()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
