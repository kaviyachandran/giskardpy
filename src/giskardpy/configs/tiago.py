from giskardpy.configs.default_giskard import Giskard, ControlModes


class TiagoBase(Giskard):
    def __init__(self):
        super().__init__()
        self.set_default_visualization_marker_color(1, 1, 1, 0.7)
        self.load_moveit_self_collision_matrix('package://tiago_dual_moveit_config/config/srdf/tiago.srdf')
        self.overwrite_external_collision_avoidance('brumbrum',
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1)
        self.ignored_collisions = ['wheel_left_link',
                                   'wheel_right_link',
                                   'caster_back_left_2_link',
                                   'caster_back_right_2_link',
                                   'caster_front_left_2_link',
                                   'caster_front_right_2_link']
        self.fix_joints_for_self_collision_avoidance(['head_1_joint',
                                                      'head_2_joint',
                                                      'gripper_left_left_finger_joint',
                                                      'gripper_left_right_finger_joint',
                                                      'gripper_right_left_finger_joint',
                                                      'gripper_right_right_finger_joint'])
        self.fix_joints_for_external_collision_avoidance(['gripper_left_left_finger_joint',
                                                          'gripper_left_right_finger_joint',
                                                          'gripper_right_left_finger_joint',
                                                          'gripper_right_right_finger_joint'])
        self.overwrite_external_collision_avoidance('arm_right_7_joint',
                                                    number_of_repeller=4,
                                                    soft_threshold=0.05,
                                                    hard_threshold=0.0,
                                                    max_velocity=0.2)
        self.overwrite_external_collision_avoidance('arm_left_7_joint',
                                                    number_of_repeller=4,
                                                    soft_threshold=0.05,
                                                    hard_threshold=0.0,
                                                    max_velocity=0.2)
        self.set_default_self_collision_avoidance(hard_threshold=0.04,
                                                  soft_threshold=0.08)
        self.set_default_external_collision_avoidance(hard_threshold=0.03,
                                                      soft_threshold=0.08)


class TiagoMujoco(TiagoBase):
    def __init__(self):
        super().__init__()
        self.add_robot_from_parameter_server()
        self.add_sync_tf_frame('map', 'odom')
        self.add_diff_drive_joint(parent_link_name='odom',
                                  child_link_name='base_footprint',
                                  translation_acceleration_limit=1,
                                  rotation_acceleration_limit=1,
                                  odometry_topic='/tiago/base_footprint')
        self.add_follow_joint_trajectory_server(namespace='/arm_left_controller/follow_joint_trajectory',
                                                state_topic='/arm_left_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/arm_right_controller/follow_joint_trajectory',
                                                state_topic='/arm_right_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/head_controller/follow_joint_trajectory',
                                                state_topic='/head_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/left_gripper_controller/follow_joint_trajectory',
                                                state_topic='/left_gripper_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/right_gripper_controller/follow_joint_trajectory',
                                                state_topic='/right_gripper_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/torso_controller/follow_joint_trajectory',
                                                state_topic='/torso_controller/state')
        self.add_base_cmd_velocity(cmd_vel_topic='/tiago/cmd_vel')
        self._qp_solver_config.joint_weights['velocity']['brumbrum'] = 0.1


class IAI_Tiago(TiagoBase):
    def __init__(self):
        super().__init__()
        self.add_sync_tf_frame('map', 'odom')
        self._add_odometry_topic('/mobile_base_controller/odom')
        self.add_robot_from_parameter_server()
        self.add_follow_joint_trajectory_server(
            namespace='/arm_left_impedance_controller/follow_joint_trajectory',
            state_topic='/arm_left_impedance_controller/state'
            # namespace='/arm_left_controller/follow_joint_trajectory',
            # state_topic='/arm_left_controller/state'
        )
        self.add_follow_joint_trajectory_server(
            namespace='/arm_right_impedance_controller/follow_joint_trajectory',
            state_topic='/arm_right_impedance_controller/state'
            # namespace='/arm_right_controller/follow_joint_trajectory',
            # state_topic='/arm_right_controller/state'
        )
        self.add_follow_joint_trajectory_server(namespace='/gripper_left_controller/follow_joint_trajectory',
                                                state_topic='/gripper_left_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/gripper_right_controller/follow_joint_trajectory',
                                                state_topic='/gripper_right_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/head_controller/follow_joint_trajectory',
                                                state_topic='/head_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/torso_controller/follow_joint_trajectory',
                                                state_topic='/torso_controller/state')
        self.add_diff_drive_interface(cmd_vel_topic='/mobile_base_controller/cmd_vel',
                                      parent_link_name='odom',
                                      child_link_name='base_footprint',
                                      translation_acceleration_limit=0.5,
                                      rotation_acceleration_limit=None)


class Tiago_Standalone(TiagoBase):
    def __init__(self):
        self.add_robot_from_parameter_server()
        super().__init__()
        self.set_default_visualization_marker_color(1, 1, 1, 1)
        self.set_control_mode(ControlModes.stand_alone)
        self.publish_all_tf()
        self.configure_VisualizationBehavior(in_planning_loop=True)
        self.configure_CollisionMarker(in_planning_loop=True)
        self.set_root_link_name('map')
        self.add_fixed_joint(parent_link='map', child_link='odom')
        self.add_diff_drive_joint(parent_link_name='odom',
                                  child_link_name='base_footprint',
                                  name='brumbrum',
                                  translation_velocity_limit=0.19,
                                  rotation_velocity_limit=0.19)
        self.register_controlled_joints(['torso_lift_joint', 'head_1_joint', 'head_2_joint', 'brumbrum'])
        self.register_controlled_joints(['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
                                         'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'])
        self.register_controlled_joints(['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                                         'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                                         'arm_right_7_joint'])
        self.register_controlled_joints(['gripper_right_left_finger_joint', 'gripper_right_right_finger_joint',
                                         'gripper_left_left_finger_joint', 'gripper_left_right_finger_joint'])
