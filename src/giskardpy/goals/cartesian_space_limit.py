from __future__ import division

from geometry_msgs.msg import Vector3Stamped

from giskardpy import casadi_wrapper as w
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA
import giskardpy.utils.tfwrapper as tf

class CartesianSpaceLimit(Goal):
 ### Refer to 1066 line
    def __init__(self, godmap,
                 tip_link,
                 lower_limit,
                 upper_limit,
                 root_link=None,
                 weight=WEIGHT_BELOW_CA,
                 goal_constraint=True):
        super(CartesianSpaceLimit, self).__init__(godmap)

        if root_link is None:
            self.root = self.get_robot().get_root()
        self.root = root_link
        self.tip = tip_link
        self.goal_constraint = goal_constraint
        self.lower_limit = w.Matrix([lower_limit[u'vector'][u'x'],
                                     lower_limit[u'vector'][u'y'],
                                     lower_limit[u'vector'][u'z']])
        self.upper_limit = w.Matrix([upper_limit[u'vector'][u'x'],
                                     upper_limit[u'vector'][u'y'],
                                     upper_limit[u'vector'][u'z']])
        ##self.goal = goal_pose
        self.weight = weight

    # self.constraints = []

    def make_constraints(self):
        root_T_tip = self.get_fk(self.root, self.tip)
        #current_velocity = self.get_fk_velocity(self.root, self.tip)
        root_P_tip = w.position_of(root_T_tip)

       #weight = self.get_input_float(self.weight)
        t = self.get_input_sampling_period()
        ### ll <= pnow + hv <= ul
        ### or is it just ll <= pnow  <= ul
        ll = (self.lower_limit - root_P_tip[0:3])
        ul = (self.upper_limit - root_P_tip[0:3])

        self.add_constraint(u'x_limit',
                            weight=self.weight,
                            expression=root_P_tip[0],
                            goal_constraint=self.goal_constraint,
                            upper_slack_limit=0, # Makes this a hard constraint
                            lower_slack_limit=0,
                            lower=ll[0],
                            upper=ul[0])

        self.add_constraint(u'y_limit',
                            weight=self.weight,
                            expression=root_P_tip[1],
                            goal_constraint=self.goal_constraint,
                            upper_slack_limit=0,
                            lower_slack_limit=0,
                            lower=ll[1],
                            upper=ul[1]
                            )

        self.add_constraint(u'z_limit',
                            weight=self.weight,
                            expression=root_P_tip[2],
                            goal_constraint=self.goal_constraint,
                            upper_slack_limit=0,
                            lower_slack_limit=0,
                            lower=ll[2],
                            upper=ul[2])
    def __str__(self):
        # helps to make sure your constraint name is unique.
        s = super(CartesianSpaceLimit, self).__str__()
        return u'{}/{}/{}'.format(s, self.root, self.tip)