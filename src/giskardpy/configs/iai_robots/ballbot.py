import numpy as np

from giskardpy.configs.collision_avoidance_config import DisableCollisionAvoidanceConfig
from giskardpy.configs.robot_interface_config import RobotInterfaceConfig, StandAloneRobotInterfaceConfig
from giskardpy.configs.world_config import WorldWithFixedRobot
from giskardpy.data_types import Derivatives


class BallBotWorldConfig(WorldWithFixedRobot):
    def __init__(self):
        super().__init__({Derivatives.velocity: 0.2,
                          Derivatives.acceleration: np.inf,
                          Derivatives.jerk: 15})


class BallBotCollisionAvoidanceConfig(DisableCollisionAvoidanceConfig):
    def __init__(self):
        #  Todo: Check if ballbot needs a collision matrix
        super().__init__()


class BallBotJointTrajServerMujocoInterface(RobotInterfaceConfig):
    def setup(self):
        self.sync_joint_state_topic('/ballbot/joint_states')
        self.add_follow_joint_trajectory_server(
            namespace='/ballbot/whole_body_controller')


class BallBotStandAloneRobotInterfaceConfig(StandAloneRobotInterfaceConfig):
    def __init__(self):
        super().__init__([
            'transx_joint',
            'transy_joint',
            'transz_joint',
            'rotx_joint',
            'roty_joint',
            'rotz_joint'
        ])
