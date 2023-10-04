from typing import List

from py_trees import Status

from giskardpy.god_map_interpreter import god_map
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import catch_and_raise_to_blackboard, record_time


class SetDriveGoals(GiskardBehavior):
    @catch_and_raise_to_blackboard
    @record_time
    @profile
    def update(self):
        god_map.motion_goal_manager.motion_goals = {}
        # fixme
        # drive_goals: List[Goal] = god_map.get_data(identifier.drive_goals)
        return Status.SUCCESS
