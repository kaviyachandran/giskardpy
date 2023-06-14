from typing import Optional

import numpy as np

from giskardpy.configs.data_types import ControlModes, CollisionCheckerLib, SupportedQPSolver, TfPublishingModes
from giskardpy.configs.default_giskard import Giskard
from giskardpy.my_types import Derivatives


class PR2_Base(Giskard):
    map_name = 'map'
    localization_joint_name = 'localization'
    odom_link_name = 'odom_combined'
    drive_joint_name = 'brumbrum'

    def configure_world(self):
        self.world.set_default_limits({Derivatives.velocity: 1,
                                       Derivatives.acceleration: np.inf,
                                       Derivatives.jerk: 30})
        self.world.add_empty_link(self.map_name)
        self.world.add_empty_link(self.odom_link_name)
        self.world.add_6dof_joint(parent_link=self.map_name, child_link=self.odom_link_name,
                                  joint_name=self.localization_joint_name)
        pr2_group_name = self.world.add_robot_from_parameter_server()
        root_link_name = self.world.get_root_link_of_group(pr2_group_name)
        self.world.add_omni_drive_joint(name=self.drive_joint_name,
                                        parent_link_name=self.odom_link_name,
                                        child_link_name=root_link_name,
                                        translation_limits={
                                            Derivatives.velocity: 0.4,
                                            Derivatives.acceleration: 1,
                                            Derivatives.jerk: 5,
                                        },
                                        rotation_limits={
                                            Derivatives.velocity: 0.2,
                                            Derivatives.acceleration: 1,
                                            Derivatives.jerk: 5
                                        },
                                        robot_group_name=pr2_group_name)

    def configure_collision_avoidance(self):
        # self.collision_avoidance.set_collision_checker(CollisionCheckerLib.none)
        self.collision_avoidance.load_moveit_self_collision_matrix('package://giskardpy/config/pr2.srdf')
        self.collision_avoidance.set_default_external_collision_avoidance(soft_threshold=0.1,
                                                                          hard_threshold=0.0)
        for joint_name in ['r_wrist_roll_joint', 'l_wrist_roll_joint']:
            self.collision_avoidance.overwrite_external_collision_avoidance(joint_name,
                                                                            number_of_repeller=4,
                                                                            soft_threshold=0.05,
                                                                            hard_threshold=0.0,
                                                                            max_velocity=0.2)
        for joint_name in ['r_wrist_flex_joint', 'l_wrist_flex_joint']:
            self.collision_avoidance.overwrite_external_collision_avoidance(joint_name,
                                                                            number_of_repeller=2,
                                                                            soft_threshold=0.05,
                                                                            hard_threshold=0.0,
                                                                            max_velocity=0.2)
        for joint_name in ['r_elbow_flex_joint', 'l_elbow_flex_joint']:
            self.collision_avoidance.overwrite_external_collision_avoidance(joint_name,
                                                                            soft_threshold=0.05,
                                                                            hard_threshold=0.0)
        for joint_name in ['r_forearm_roll_joint', 'l_forearm_roll_joint']:
            self.collision_avoidance.overwrite_external_collision_avoidance(joint_name,
                                                                            soft_threshold=0.025,
                                                                            hard_threshold=0.0)
        self.collision_avoidance.ignore_all_collisions_of_links(['bl_caster_l_wheel_link', 'bl_caster_r_wheel_link',
                                                                 'fl_caster_l_wheel_link', 'fl_caster_r_wheel_link',
                                                                 'br_caster_l_wheel_link', 'br_caster_r_wheel_link',
                                                                 'fr_caster_l_wheel_link', 'fr_caster_r_wheel_link'])
        self.collision_avoidance.fix_joints_for_self_collision_avoidance(['head_pan_joint',
                                                                          'head_tilt_joint',
                                                                          'r_gripper_l_finger_joint',
                                                                          'l_gripper_l_finger_joint'])
        self.collision_avoidance.fix_joints_for_external_collision_avoidance(['r_gripper_l_finger_joint',
                                                                              'l_gripper_l_finger_joint'])
        self.collision_avoidance.overwrite_external_collision_avoidance(self.drive_joint_name,
                                                                        number_of_repeller=2,
                                                                        soft_threshold=0.2,
                                                                        hard_threshold=0.1)

    def configure_robot_interface(self):
        pass


class PR2_Mujoco(PR2_Base):

    def configure_execution(self):
        self.execution.set_control_mode(ControlModes.open_loop)

    def configure_world(self):
        super().configure_world()
        self.world.set_default_visualization_marker_color(1, 1, 1, 0.7)

    def configure_behavior_tree(self):
        super().configure_behavior_tree()
        self.behavior_tree.add_visualization_marker_publisher(add_to_sync=True, add_to_planning=True,
                                                              add_to_control_loop=False)

    def configure_robot_interface(self):
        super().configure_robot_interface()
        self.robot_interface.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                                           tf_parent_frame=self.map_name,
                                                           tf_child_frame=self.odom_link_name)
        self.robot_interface.sync_joint_state_topic('/joint_states')
        self.robot_interface.sync_odometry_topic('/pr2/base_footprint', self.drive_joint_name)
        self.robot_interface.add_follow_joint_trajectory_server(
            namespace='/pr2/whole_body_controller/follow_joint_trajectory',
            state_topic='/pr2/whole_body_controller/state')
        self.robot_interface.add_follow_joint_trajectory_server(
            namespace='/pr2/l_gripper_l_finger_controller/follow_joint_trajectory',
            state_topic='/pr2/l_gripper_l_finger_controller/state')
        self.robot_interface.add_follow_joint_trajectory_server(
            namespace='/pr2/r_gripper_l_finger_controller/follow_joint_trajectory',
            state_topic='/pr2/r_gripper_l_finger_controller/state')
        self.robot_interface.add_base_cmd_velocity(cmd_vel_topic='/pr2/cmd_vel',
                                                   track_only_velocity=True,
                                                   joint_name=self.drive_joint_name)


class PR2_IAI(PR2_Base):
    def configure_execution(self):
        self.execution.set_control_mode(ControlModes.open_loop)

    def configure_world(self):
        super().configure_world()
        self.world.set_default_visualization_marker_color(20 / 255, 27.1 / 255, 80 / 255, 0.2)

    def configure_behavior_tree(self):
        super().configure_behavior_tree()
        self.behavior_tree.add_visualization_marker_publisher(add_to_sync=True, add_to_planning=True,
                                                              add_to_control_loop=False)

    def configure_robot_interface(self):
        super().configure_robot_interface()
        self.robot_interface.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                                           tf_parent_frame=self.map_name,
                                                           tf_child_frame=self.odom_link_name)
        self.robot_interface.sync_joint_state_topic('/joint_states')
        self.robot_interface.sync_odometry_topic('/robot_pose_ekf/odom_combined', self.drive_joint_name)
        fill_velocity_values = False
        self.robot_interface.add_follow_joint_trajectory_server(namespace='/l_arm_controller/follow_joint_trajectory',
                                                                state_topic='/l_arm_controller/state',
                                                                fill_velocity_values=fill_velocity_values)
        self.robot_interface.add_follow_joint_trajectory_server(namespace='/r_arm_controller/follow_joint_trajectory',
                                                                state_topic='/r_arm_controller/state',
                                                                fill_velocity_values=fill_velocity_values)
        self.robot_interface.add_follow_joint_trajectory_server(namespace='/torso_controller/follow_joint_trajectory',
                                                                state_topic='/torso_controller/state',
                                                                fill_velocity_values=fill_velocity_values)
        self.robot_interface.add_follow_joint_trajectory_server(
            namespace='/head_traj_controller/follow_joint_trajectory',
            state_topic='/head_traj_controller/state',
            fill_velocity_values=fill_velocity_values)
        self.robot_interface.add_base_cmd_velocity(cmd_vel_topic='/base_controller/command',
                                                   track_only_velocity=True,
                                                   joint_name=self.drive_joint_name)


class PR2_Unreal(PR2_Base):
    def configure_execution(self):
        self.execution.set_control_mode(ControlModes.open_loop)

    def configure_world(self):
        super().configure_world()
        self.world.set_default_visualization_marker_color(20 / 255, 27.1 / 255, 80 / 255, 0.2)

    def configure_behavior_tree(self):
        super().configure_behavior_tree()
        self.behavior_tree.add_visualization_marker_publisher(add_to_sync=True, add_to_planning=True,
                                                              add_to_control_loop=False)

    def configure_robot_interface(self):
        super().configure_robot_interface()
        self.robot_interface.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                                           tf_parent_frame=self.map_name,
                                                           tf_child_frame=self.odom_link_name)
        self.robot_interface.sync_joint_state_topic('/joint_states')
        self.robot_interface.sync_odometry_topic('/base_odometry/odom', self.drive_joint_name)
        fill_velocity_values = False
        self.robot_interface.add_follow_joint_trajectory_server(
            namespace='/whole_body_controller/follow_joint_trajectory',
            state_topic='/whole_body_controller/state',
            fill_velocity_values=fill_velocity_values)
        self.robot_interface.add_follow_joint_trajectory_server(
            namespace='/head_traj_controller/follow_joint_trajectory',
            state_topic='/head_traj_controller/state',
            fill_velocity_values=fill_velocity_values)
        self.robot_interface.add_base_cmd_velocity(cmd_vel_topic='/base_controller/command',
                                                   track_only_velocity=True,
                                                   joint_name=self.drive_joint_name)


class PR2_StandAlone(PR2_Base):

    def configure_execution(self):
        self.execution.set_control_mode(ControlModes.stand_alone)
        self.execution.set_max_trajectory_length(length=30)

    def configure_world(self):
        super().configure_world()
        self.world.set_default_visualization_marker_color(1, 1, 1, 0.8)

    def configure_robot_interface(self):
        super().configure_robot_interface()
        self.robot_interface.register_controlled_joints([
            'torso_lift_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'r_shoulder_pan_joint',
            'r_shoulder_lift_joint',
            'r_upper_arm_roll_joint',
            'r_forearm_roll_joint',
            'r_elbow_flex_joint',
            'r_wrist_flex_joint',
            'r_wrist_roll_joint',
            'l_shoulder_pan_joint',
            'l_shoulder_lift_joint',
            'l_upper_arm_roll_joint',
            'l_forearm_roll_joint',
            'l_elbow_flex_joint',
            'l_wrist_flex_joint',
            'l_wrist_roll_joint',
            self.drive_joint_name,
        ])

    def configure_behavior_tree(self):
        super().configure_behavior_tree()
        self.behavior_tree.add_visualization_marker_publisher(add_to_sync=True, add_to_planning=False,
                                                              add_to_control_loop=True)
        self.behavior_tree.add_tf_publisher(include_prefix=True, mode=TfPublishingModes.all)
