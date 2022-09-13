from __future__ import division

from typing import Union, Dict, Optional, List

from geometry_msgs.msg import PoseStamped
from pybullet import getAxisAngleFromQuaternion
from sensor_msgs.msg import JointState

from giskardpy import casadi_wrapper as w, identifier
from giskardpy.configs.default_config import ControlModes
from giskardpy.exceptions import ConstraintException, ConstraintInitalizationException
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA
from giskardpy.god_map import GodMap
from giskardpy.model.joints import OmniDrive, DiffDrive


class SetSeedConfiguration(Goal):
    # FIXME deal with prefix
    def __init__(self, seed_configuration: Dict[str, float], **kwargs):
        super().__init__(**kwargs)
        if self.god_map.get_data(identifier.execute) \
                and self.god_map.get_data(identifier.control_mode) != ControlModes.stand_alone:
            raise ConstraintInitalizationException(f'It is not allowed to combine {str(self)} with plan and execute.')
        for joint_name, initial_joint_value in seed_configuration.items():
            if joint_name not in self.world.state:
                raise KeyError(f'world has no joint \'{joint_name}\'')
            self.world.state[joint_name].position = initial_joint_value
        self.world.notify_state_change()


class SetOdometry(Goal):
    def __init__(self, group_name: str, base_pose: PoseStamped, **kwargs):
        super().__init__(**kwargs)
        if self.god_map.get_data(identifier.execute) \
                and self.god_map.get_data(identifier.control_mode) != ControlModes.stand_alone:
            raise ConstraintInitalizationException(f'It is not allowed to combine {str(self)} with plan and execute.')
        brumbrum_joint_name = self.world.groups[group_name].root_link.parent_joint_name
        brumbrum_joint = self.world.joints[brumbrum_joint_name]
        if not isinstance(brumbrum_joint, (OmniDrive, DiffDrive)):
            raise ConstraintInitalizationException(f'Group {group_name} has no odometry joint.')
        base_pose = self.transform_msg(brumbrum_joint.parent_link_name, base_pose).pose
        self.world.state[brumbrum_joint.x_name].position = base_pose.position.x
        self.world.state[brumbrum_joint.y_name].position = base_pose.position.y
        axis, angle = getAxisAngleFromQuaternion([base_pose.orientation.x,
                                                  base_pose.orientation.y,
                                                  base_pose.orientation.z,
                                                  base_pose.orientation.w])
        if axis[-1] < 0:
            angle = -angle
        self.world.state[brumbrum_joint.yaw_name].position = angle


class JointPositionContinuous(Goal):

    def __init__(self, joint_name: str, goal: float, weight: float = WEIGHT_BELOW_CA, max_velocity: float = 1,
                 hard: bool = False, **kwargs):
        """
        This goal will move a continuous joint to the goal position
        :param joint_name: str
        :param goal: float
        :param weight: float, default WEIGHT_BELOW_CA
        :param max_velocity: float, rad/s, default 1423, meaning the urdf/config limits are active
        """
        self.joint_name = joint_name
        self.joint_goal = goal
        self.weight = weight
        self.max_velocity = max_velocity
        self.hard = hard
        super().__init__(**kwargs)

        if not self.world.is_joint_continuous(joint_name):
            raise ConstraintException(f'{self.__class__.__name__} called with non continuous joint {joint_name}')

    def make_constraints(self):
        """
        example:
        name='JointPosition'
        parameter_value_pair='{
            "joint_name": "torso_lift_joint", #required
            "goal_position": 0, #required
            "weight": 1, #optional
            "max_velocity": 1 #optional -- rad/s or m/s depending on joint; can not go higher than urdf limit
        }'
        :return:
        """
        current_joint = self.get_joint_position_symbol(self.joint_name)
        max_velocity = w.min(self.max_velocity,
                             self.world.get_joint_velocity_limits(self.joint_name)[1])

        error = w.shortest_angular_distance(current_joint, self.joint_goal)

        if self.hard:
            self.add_constraint(reference_velocity=max_velocity,
                                lower_error=error,
                                upper_error=error,
                                weight=self.weight,
                                expression=current_joint,
                                lower_slack_limit=0,
                                upper_slack_limit=0)
        else:
            self.add_constraint(reference_velocity=max_velocity,
                                lower_error=error,
                                upper_error=error,
                                weight=self.weight,
                                expression=current_joint)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class JointPositionPrismatic(Goal):
    def __init__(self, joint_name: str, goal: float, weight: float = WEIGHT_BELOW_CA, max_velocity: float = 1,
                 hard: bool = False, **kwargs):
        """
        This goal will move a prismatic joint to the goal position
        :param weight: default WEIGHT_BELOW_CA
        :param max_velocity: m/s, default 4535, meaning the urdf/config limits are active
        """
        self.joint_name = joint_name
        self.goal = goal
        self.weight = weight
        self.max_velocity = max_velocity
        self.hard = hard
        super().__init__(**kwargs)
        if not self.world.is_joint_prismatic(joint_name):
            raise ConstraintException(f'{self.__class__.__name__} called with non prismatic joint {joint_name}')

    def make_constraints(self):
        """
        example:
        name='JointPosition'
        parameter_value_pair='{
            "joint_name": "torso_lift_joint", #required
            "goal_position": 0, #required
            "weight": 1, #optional
            "gain": 10, #optional -- error is multiplied with this value
            "max_speed": 1 #optional -- rad/s or m/s depending on joint; can not go higher than urdf limit
        }'
        :return:
        """
        current_joint = self.get_joint_position_symbol(self.joint_name)


        try:
            # if self.world.is_joint_mimic(self.joint_name):
            #     mimed_joint_name = self.world.joints[self.joint_name].mimed_joint_name
            #     mimed_joint_symbol = self.get_joint_position_symbol(mimed_joint_name)
            #     mimied_limit = self.world.joint_limit_expr(self.joint_name, 1)[1]
            #     limit_expr = w.substitute(current_joint, mimed_joint_symbol, mimied_limit)
            # else:
            limit_expr = self.world.get_joint_velocity_limits(self.joint_name)[1]
            max_velocity = w.min(self.max_velocity,
                                 limit_expr)
        except IndexError:
            max_velocity = self.max_velocity

        error = self.goal - current_joint

        if self.hard:
            self.add_constraint(reference_velocity=max_velocity,
                                lower_error=error,
                                upper_error=error,
                                weight=self.weight,
                                expression=current_joint,
                                upper_slack_limit=0,
                                lower_slack_limit=0)
        else:
            self.add_constraint(reference_velocity=max_velocity,
                                lower_error=error,
                                upper_error=error,
                                weight=self.weight,
                                expression=current_joint)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class JointVelocityRevolute(Goal):
    def __init__(self, joint_name: str, weight: float = WEIGHT_BELOW_CA, max_velocity: float = 1,
                 hard: bool = False, **kwargs):
        """
        This goal will move a prismatic joint to the goal position
        :param weight: default WEIGHT_BELOW_CA
        :param max_velocity: m/s, default 4535, meaning the urdf/config limits are active
        """
        self.joint_name = joint_name
        self.weight = weight
        self.max_velocity = max_velocity
        self.hard = hard
        super().__init__(**kwargs)
        if not self.world.is_joint_revolute(joint_name):
            raise ConstraintException(f'{self.__class__.__name__} called with non revolute joint {joint_name}')

    def make_constraints(self):
        current_joint = self.get_joint_position_symbol(self.joint_name)

        # joint_goal = self.get_parameter_as_symbolic_expression('goal')
        # weight = self.get_parameter_as_symbolic_expression('weight')

        try:
            # if self.world.is_joint_mimic(self.joint_name):
            #     mimed_joint_name = self.world.joints[self.joint_name].mimed_joint_name
            #     mimed_joint_symbol = self.get_joint_position_symbol(mimed_joint_name)
            #     mimied_limit = self.world.joint_limit_expr(self.joint_name, 1)[1]
            #     limit_expr = w.substitute(current_joint, mimed_joint_symbol, mimied_limit)
            # else:
            limit_expr = self.world.get_joint_velocity_limits(self.joint_name)[1]
            max_velocity = w.min(self.max_velocity,
                                 limit_expr)
        except IndexError:
            max_velocity = self.max_velocity

        # error = joint_goal - current_joint

        if self.hard:
            self.add_velocity_constraint(lower_velocity_limit=-max_velocity,
                                         upper_velocity_limit=max_velocity,
                                         weight=self.weight,
                                         expression=current_joint,
                                         velocity_limit=max_velocity,
                                         lower_slack_limit=0,
                                         upper_slack_limit=0)
        else:
            self.add_velocity_constraint(lower_velocity_limit=-max_velocity,
                                         upper_velocity_limit=max_velocity,
                                         weight=self.weight,
                                         expression=current_joint,
                                         velocity_limit=max_velocity)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class JointPositionRevolute(Goal):

    def __init__(self, joint_name: str, goal: float, weight: float = WEIGHT_BELOW_CA, max_velocity: float = 1,
                 hard: bool = False, **kwargs):
        """
        This goal will move a revolute joint to the goal position
        :param joint_name: str
        :param goal: float
        :param weight: float, default WEIGHT_BELOW_CA
        :param max_velocity: float, rad/s, default 3451, meaning the urdf/config limits are active
        """
        self.joint_name = joint_name
        self.goal = goal
        self.weight = weight
        self.max_velocity = max_velocity
        self.hard = hard
        super().__init__(**kwargs)
        if not self.world.is_joint_revolute(joint_name):
            raise ConstraintException(f'{self.__class__.__name__} called with non revolute joint {joint_name}')

    def make_constraints(self):
        """
        example:
        name='JointPosition'
        parameter_value_pair='{
            "joint_name": "torso_lift_joint", #required
            "goal_position": 0, #required
            "weight": 1, #optional
            "gain": 10, #optional -- error is multiplied with this value
            "max_speed": 1 #optional -- rad/s or m/s depending on joint; can not go higher than urdf limit
        }'
        :return:
        """
        current_joint = self.get_joint_position_symbol(self.joint_name)

        joint_goal = self.goal
        weight = self.weight

        max_velocity = w.min(self.max_velocity,
                             self.world.get_joint_velocity_limits(self.joint_name)[1])

        error = joint_goal - current_joint
        # self.add_debug_expr('error', error)
        if self.hard:
            self.add_constraint(reference_velocity=max_velocity,
                                lower_error=error,
                                upper_error=error,
                                weight=weight,
                                expression=current_joint,
                                upper_slack_limit=0,
                                lower_slack_limit=0)
        else:
            self.add_constraint(reference_velocity=max_velocity,
                                lower_error=error,
                                upper_error=error,
                                weight=weight,
                                expression=current_joint)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class ShakyJointPositionRevoluteOrPrismatic(Goal):
    def __init__(self, joint_name, goal, frequency, noise_amplitude=1.0, weight=WEIGHT_BELOW_CA,
                 max_velocity=1, **kwargs):
        """
        This goal will move a revolute or prismatic joint to the goal position and shake the joint with the given frequency.
        :param joint_name: str
        :param goal: float
        :param frequency: float
        :param noise_amplitude: float
        :param weight: float, default WEIGHT_BELOW_CA
        :param max_velocity: float, rad/s, default 3451, meaning the urdf/config limits are active
        """
        self.joint_name = joint_name
        super().__init__(**kwargs)
        if not self.world.is_joint_revolute(joint_name) and not self.world.is_joint_prismatic(joint_name):
            raise ConstraintException(
                f'{self.__class__.__name__} called with non revolute/prismatic joint {joint_name}')

        self.goal = goal
        self.frequency = frequency
        self.noise_amplitude = noise_amplitude
        self.weight = weight
        self.max_velocity = max_velocity

    def make_constraints(self):
        """
        example:
        name='ShakyJointPositionRevoluteOrPrismatic'
        parameter_value_pair='{
            "joint_name": "r_wrist_flex_joint", #required
            "goal_position": -1.0, #required
            "frequency": 5.0, #required
            "weight": 1, #optional
            "max_velocity": 1 #optional -- rad/s or m/s depending on joint; can not go higher than urdf limit
        }'
        :return:
        """
        current_joint = self.get_joint_position_symbol(self.joint_name)
        frequency = self.frequency
        noise_amplitude = self.noise_amplitude
        joint_goal = self.goal
        weight = self.weight

        time = self.god_map.to_symbol(identifier.time)
        time_in_secs = self.sample_period * time

        max_velocity = w.min(self.max_velocity,
                             self.world.get_joint_velocity_limits(self.joint_name)[1])

        fun_params = frequency * 2.0 * w.pi * time_in_secs
        err = (joint_goal - current_joint) + noise_amplitude * max_velocity * w.sin(fun_params)
        capped_err = w.limit(err, -noise_amplitude * max_velocity, noise_amplitude * max_velocity)

        self.add_constraint(lower_error=capped_err,
                            upper_error=capped_err,
                            reference_velocity=max_velocity,
                            weight=weight,
                            expression=current_joint)

    def __str__(self):
        s = super(ShakyJointPositionRevoluteOrPrismatic, self).__str__()
        return f'{s}/{self.joint_name}'


class ShakyJointPositionContinuous(Goal):
    def __init__(self, joint_name, goal, frequency, noise_amplitude=10, weight=WEIGHT_BELOW_CA,
                 max_velocity=1, **kwargs):
        """
        This goal will move a continuous joint to the goal position and shake the joint with the given frequency.
        :param joint_name: str
        :param goal: float
        :param frequency: float
        :param noise_amplitude: float
        :param weight: float, default WEIGHT_BELOW_CA
        :param max_velocity: float, rad/s, default 3451, meaning the urdf/config limits are active
        """
        self.joint_name = joint_name
        self.goal = goal
        self.frequency = frequency
        self.noise_amplitude = noise_amplitude
        self.weight = weight
        self.max_velocity = max_velocity
        super().__init__(**kwargs)
        if not self.world.is_joint_continuous(joint_name):
            raise ConstraintException(f'{self.__class__.__name__} called with non continuous joint {joint_name}')

    def make_constraints(self):
        """
        example:
        name='JointPosition'
        parameter_value_pair='{
            "joint_name": "l_wrist_roll_joint", #required
            "goal_position": -5.0, #required
            "frequency": 5.0, #required
            "weight": 1, #optional
            "max_velocity": 1 #optional -- rad/s or m/s depending on joint; can not go higher than urdf limit
        }'
        :return:
        """
        current_joint = self.get_joint_position_symbol(self.joint_name)
        frequency = self.frequency
        noise_amplitude = self.noise_amplitude
        joint_goal = self.goal
        weight = self.weight

        time = self.god_map.to_symbol(identifier.time)
        time_in_secs = self.sample_period * time

        max_velocity = w.min(self.max_velocity,
                             self.world.get_joint_velocity_limits(self.joint_name)[1])

        fun_params = frequency * 2.0 * w.pi * time_in_secs
        err = w.shortest_angular_distance(current_joint, joint_goal) + noise_amplitude * max_velocity * w.sin(
            fun_params)

        capped_err = w.limit(err, -noise_amplitude * max_velocity, noise_amplitude * max_velocity)

        self.add_constraint(lower_error=capped_err,
                            upper_error=capped_err,
                            reference_velocity=max_velocity,
                            weight=weight,
                            expression=current_joint)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class AvoidJointLimitsRevolute(Goal):
    def __init__(self, joint_name, weight=0.1, max_linear_velocity=100, percentage=5, **kwargs):
        """
        This goal will push revolute joints away from their position limits
        :param joint_name: str
        :param weight: float, default WEIGHT_BELOW_CA
        :param max_linear_velocity: float, default 1e9, meaning the urdf/config limit will kick in
        :param percentage: float, default 15, if limits are 0-100, the constraint will push into the 15-85 range
        """
        self.joint_name = joint_name
        self.weight = weight
        self.max_velocity = max_linear_velocity
        self.percentage = percentage
        super().__init__(**kwargs)
        if not self.world.is_joint_revolute(joint_name):
            raise ConstraintException(f'{self.__class__.__name__} called with non prismatic joint {joint_name}')

    def make_constraints(self):
        weight = self.weight
        joint_symbol = self.get_joint_position_symbol(self.joint_name)
        percentage = self.percentage / 100.
        lower_limit, upper_limit = self.world.get_joint_position_limits(self.joint_name)
        max_velocity = self.max_velocity
        max_velocity = w.min(max_velocity,
                             self.world.get_joint_velocity_limits(self.joint_name)[1])

        joint_range = upper_limit - lower_limit
        center = (upper_limit + lower_limit) / 2.

        max_error = joint_range / 2. * percentage

        upper_goal = center + joint_range / 2. * (1 - percentage)
        lower_goal = center - joint_range / 2. * (1 - percentage)

        upper_err = upper_goal - joint_symbol
        lower_err = lower_goal - joint_symbol

        # upper_err_capped = self.limit_velocity(upper_err, max_velocity)
        # lower_err_capped = self.limit_velocity(lower_err, max_velocity)

        error = w.max(w.abs(w.min(upper_err, 0)), w.abs(w.max(lower_err, 0)))
        weight = weight * (error / max_error)

        self.add_constraint(reference_velocity=max_velocity,
                            lower_error=lower_err,
                            upper_error=upper_err,
                            weight=weight,
                            expression=joint_symbol)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class AvoidJointLimitsPrismatic(Goal):
    def __init__(self, joint_name, weight=0.1, max_angular_velocity=100, percentage=5, **kwargs):
        """
        This goal will push prismatic joints away from their position limits
        :param joint_name: str
        :param weight: float, default WEIGHT_BELOW_CA
        :param max_angular_velocity: float, default 1e9, meaning the urdf/config limit will kick in
        :param percentage: float, default 15, if limits are 0-100, the constraint will push into the 15-85 range
        """
        self.joint_name = joint_name
        self.weight = weight
        self.max_velocity = max_angular_velocity
        self.percentage = percentage
        super().__init__(**kwargs)
        if not self.world.is_joint_prismatic(joint_name):
            raise ConstraintException(f'{self.__class__.__name__} called with non prismatic joint {joint_name}')

    def make_constraints(self):
        weight = self.weight
        joint_symbol = self.get_joint_position_symbol(self.joint_name)
        percentage = self.percentage / 100.
        lower_limit, upper_limit = self.world.get_joint_position_limits(self.joint_name)
        max_velocity = self.max_velocity
        max_velocity = w.min(max_velocity,
                             self.world.get_joint_velocity_limits(self.joint_name)[1])

        joint_range = upper_limit - lower_limit
        center = (upper_limit + lower_limit) / 2.

        max_error = joint_range / 2. * percentage

        upper_goal = center + joint_range / 2. * (1 - percentage)
        lower_goal = center - joint_range / 2. * (1 - percentage)

        upper_err = upper_goal - joint_symbol
        lower_err = lower_goal - joint_symbol

        # upper_err_capped = self.limit_velocity(upper_err, max_velocity)
        # lower_err_capped = self.limit_velocity(lower_err, max_velocity)

        error = w.max(w.abs(w.min(upper_err, 0)), w.abs(w.max(lower_err, 0)))
        weight = weight * (error / max_error)

        self.add_constraint(reference_velocity=max_velocity,
                            lower_error=lower_err,
                            upper_error=upper_err,
                            weight=weight,
                            expression=joint_symbol)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class JointPositionList(Goal):
    def __init__(self, goal_state: Dict[str, float], weight: float = None,
                 max_velocity: float = None, hard: bool = False, **kwargs):
        """
        This goal takes a joint state and adds the other JointPosition goals depending on their type
        :param weight: default is the default of the added joint goals
        :param max_velocity: default is the default of the added joint goals
        """
        super().__init__(**kwargs)
        self.joint_names = list(goal_state.keys())
        if len(goal_state) == 0:
            raise ConstraintInitalizationException(f'Can\'t initialize {self} with no joints.')
        for joint_name, goal_position in goal_state.items():
            if not self.world.has_joint(joint_name):
                raise KeyError(f'unknown joint \'{joint_name}\'')
            params = kwargs
            params.update({'joint_name': joint_name,
                           'goal': goal_position})
            if weight is not None:
                params['weight'] = weight
            if max_velocity is not None:
                params['max_velocity'] = max_velocity
            params['hard'] = hard
            self.add_constraints_of_goal(JointPosition(**params))

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_names}'


class JointPosition(Goal):
    def __init__(self, joint_name: str, goal: float, weight: float = WEIGHT_BELOW_CA, max_velocity: float = 100,
                 **kwargs):
        super().__init__(**kwargs)
        self.joint_name = joint_name
        if self.world.is_joint_continuous(joint_name):
            C = JointPositionContinuous
        elif self.world.is_joint_revolute(joint_name):
            C = JointPositionRevolute
        elif self.world.is_joint_prismatic(joint_name):
            C = JointPositionPrismatic
        else:
            raise ConstraintInitalizationException(f'\'{joint_name}\' has to be continuous, revolute or prismatic')
        self.add_constraints_of_goal(C(joint_name=joint_name,
                                       goal=goal,
                                       weight=weight,
                                       max_velocity=max_velocity,
                                       **kwargs))

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'


class AvoidJointLimits(Goal):
    def __init__(self, percentage=15, weight=WEIGHT_BELOW_CA, joint_list: Optional[List[str]] = None, **kwargs):
        """
        This goal will push joints away from their position limits
        :param percentage: float, default 15, if limits are 0-100, the constraint will push into the 15-85 range
        :param weight: float, default WEIGHT_BELOW_CA
        """
        super().__init__(**kwargs)
        if joint_list is None:
            joint_list = self.god_map.get_data(identifier.controlled_joints)
        for joint_name in joint_list:
            if self.world.is_joint_revolute(joint_name):
                self.add_constraints_of_goal(AvoidJointLimitsRevolute(joint_name=joint_name,
                                                                      percentage=percentage,
                                                                      weight=weight, **kwargs))
            elif self.world.is_joint_prismatic(joint_name):
                self.add_constraints_of_goal(AvoidJointLimitsPrismatic(joint_name=joint_name,
                                                                       percentage=percentage,
                                                                       weight=weight, **kwargs))


class JointPositionRange(Goal):
    def __init__(self, joint_name, upper_limit, lower_limit, hard=False, **kwargs):
        super().__init__(**kwargs)
        self.joint_name = joint_name
        if self.world.is_joint_continuous(joint_name):
            raise NotImplementedError(f'Can\'t limit range of continues joint \'{self.joint_name}\'.')
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        self.hard = hard
        if self.hard:
            current_position = self.world.state[self.joint_name].position
            if current_position > self.upper_limit + 2e-3 or current_position < self.lower_limit - 2e-3:
                raise ConstraintInitalizationException(f'{self.joint_name} out of set limits. '
                                                       '{self.lower_limit} <= {current_position} <= {self.upper_limit} '
                                                       'is not true.')

    def make_constraints(self):
        joint_position = self.get_joint_position_symbol(self.joint_name)
        if self.hard:
            self.add_constraint(reference_velocity=self.world.get_joint_velocity_limits(self.joint_name)[1],
                                lower_error=self.lower_limit - joint_position,
                                upper_error=self.upper_limit - joint_position,
                                weight=WEIGHT_BELOW_CA,
                                expression=joint_position,
                                lower_slack_limit=0,
                                upper_slack_limit=0)
        else:
            self.add_constraint(reference_velocity=self.world.get_joint_velocity_limits(self.joint_name)[1],
                                lower_error=self.lower_limit - joint_position,
                                upper_error=self.upper_limit - joint_position,
                                weight=WEIGHT_BELOW_CA,
                                expression=joint_position)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.joint_name}'
