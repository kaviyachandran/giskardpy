from __future__ import division

from geometry_msgs.msg import Vector3Stamped

from giskardpy import casadi_wrapper as w
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA, WEIGHT_ABOVE_CA
import giskardpy.utils.tfwrapper as tf
from giskardpy.utils.tfwrapper import msg_to_homogeneous_matrix


class Pointing(Goal):
    def __init__(self, tip_link, goal_point, root_link, tip_group: str = None, root_group: str = None,
                 pointing_axis=None, max_velocity=0.3, weight=WEIGHT_BELOW_CA, **kwargs):
        """
        Uses the kinematic chain from root_link to tip_link to move the pointing axis, such that it points to the goal point.
        :param tip_link: str, name of the tip of the kin chain
        :param goal_point: PointStamped as json, where the pointing_axis will point towards
        :param root_link: str, name of the root of the kin chain
        :param pointing_axis: Vector3Stamped as json, default is z axis, this axis will point towards the goal_point
        :param weight: float, default WEIGHT_BELOW_CA
        """
        super().__init__(**kwargs)
        self.weight = weight
        self.max_velocity = max_velocity
        self.root = self.world.get_link_name(root_link, root_group)
        self.tip = self.world.get_link_name(tip_link, tip_group)
        self.root_P_goal_point = self.transform_msg(self.root, goal_point)

        if pointing_axis is not None:
            self.tip_V_pointing_axis = self.transform_msg(self.tip, pointing_axis)
            self.tip_V_pointing_axis.vector = tf.normalize(self.tip_V_pointing_axis.vector)
        else:
            self.tip_V_pointing_axis = Vector3Stamped()
            self.tip_V_pointing_axis.header.frame_id = self.tip
            self.tip_V_pointing_axis.vector.z = 1

    def make_constraints(self):
        root_T_tip = self.get_fk(self.root, self.tip)
        root_P_goal_point = w.ros_msg_to_matrix(self.root_P_goal_point)
        tip_V_pointing_axis = w.ros_msg_to_matrix(self.tip_V_pointing_axis)

        root_V_goal_axis = root_P_goal_point - w.position_of(root_T_tip)
        root_V_goal_axis /= w.norm(root_V_goal_axis)  # FIXME avoid /0
        root_V_pointing_axis = w.dot(root_T_tip, tip_V_pointing_axis)

        self.add_vector_goal_constraints(frame_V_current=root_V_pointing_axis,
                                         frame_V_goal=root_V_goal_axis,
                                         reference_velocity=self.max_velocity,
                                         weight=self.weight)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.root}/{self.tip}'
