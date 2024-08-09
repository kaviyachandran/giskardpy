import rospy
from geometry_msgs.msg import PoseStamped
from giskardpy.python_interface.python_interface import GiskardWrapper
from giskard_msgs.msg import MoveResult


# init ros node
rospy.init_node('test')
giskard_wrapper = GiskardWrapper()
obj = "free_cup"
rospy.loginfo('Set a Cartesian goal for the box')
obj_goal = PoseStamped()
obj_goal.header.frame_id = obj
obj_goal.pose.position.x = -0.2
obj_goal.pose.position.y = 0.8
obj_goal.pose.position.z = 0.5
obj_goal.pose.orientation.w = 1
giskard_wrapper.motion_goals.add_cartesian_pose(goal_pose=obj_goal,
                                                tip_link=obj,
                                                root_link='map')

giskard_wrapper.add_default_end_motion_conditions()  # it was not executing without this line
rospy.loginfo('Send cartesian goal for box.')
result: MoveResult = giskard_wrapper.execute()

if isinstance(result, MoveResult):
    print("there is something: ", result.error.code)
    for i in result.trajectory.points:
        print("ha ", i.positions)
rospy.loginfo('Goal reached')
