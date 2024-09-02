import rospy

from giskardpy.python_interface.python_interface import GiskardWrapper
from giskardpy.god_map import god_map
from giskardpy.utils.tfwrapper import lookup_transform, lookup_pose, lookup_point
from giskard_msgs.msg import MoveResult


class GiskardReasoner(object):
    def __init__(self):
        self.giskard = GiskardWrapper()

    def reachability(self, object_link: str, end_effector_link: str, root_link: str = "map"):
        # root_link is the expressed in frame

        object_link = god_map.world.search_for_link_name(object_link)
        end_effector_link = god_map.world.search_for_link_name(end_effector_link)

        object_point = lookup_point(root_link, object_link)

        self.giskard.motion_goals.add_cartesian_position(goal_point=object_point,
                                                         goal_link=end_effector_link,
                                                         root_link=root_link,
                                                         max_velocity=0.1,
                                                         weight=1.0,
                                                         name="reachability")

        result: MoveResult = self.giskard.projection()

        if result.error.code == 0:
            rospy.loginfo("Projection successful.")
            return "success"
        elif result.error.code == 102:
            rospy.loginfo("Projection failed.")
            return "out_of_joint_limits"
        else:
            rospy.loginfo("Projection failed.")
            return result.error.msg
