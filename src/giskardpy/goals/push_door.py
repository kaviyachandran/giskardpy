from giskardpy import casadi_wrapper as cas
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA
from giskardpy.utils import tfwrapper as tf

### ToDo:
# 1. First objective is to get to the closest point on the plane
# 2. a. Get the intermediate point (At the mid of opening)?
#    b.Second objective is to enforce a rotation constraint for the intermediate point
# 3. Find the jacobian matrix of the stretched out arm and then compute the det or svd
# (This should be the trigger to use a different maneuver)

class PushDoor(Goal):
    def __init__(self,
                 weight: float = WEIGHT_BELOW_CA):


    def make_constraints(self):
        pass

    def __str__(self):
        pass

