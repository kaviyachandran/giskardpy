#!/usr/bin/env python
import rospy

from giskardpy.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy.configs.giskard import Giskard
from giskardpy.configs.iai_robots.ballbot import BallBotWorldConfig, BallBotCollisionAvoidanceConfig, \
    BallBotStandAloneRobotInterfaceConfig

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=BallBotWorldConfig(),
                      collision_avoidance_config=BallBotCollisionAvoidanceConfig(),
                      robot_interface_config=BallBotStandAloneRobotInterfaceConfig(),
                      behavior_tree_config=StandAloneBTConfig())
    giskard.live()
