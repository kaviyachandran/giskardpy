import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped
from giskardpy.python_interface import GiskardWrapper

init_joint_states = {'transx_joint': 0.0,
                     'transy_joint': 0.0,
                     'transz_joint': 0.0,
                     'rotx_joint': 0.0,
                     'roty_joint': 0.0,
                     'rotz_joint': 0.0}

# init ros node
rospy.init_node('test')

giskard_wrapper = GiskardWrapper()

obj = "ball_link"
rospy.loginfo('Set a Cartesian goal for the box')
obj_goal = PoseStamped()
obj_goal.header.frame_id = obj
obj_goal.pose.position.x = 0.5
obj_goal.pose.orientation.w = 1
giskard_wrapper.set_cart_goal(goal_pose=obj_goal,
                              tip_link=obj,
                              root_link='world')
giskard_wrapper.plan_and_execute()
