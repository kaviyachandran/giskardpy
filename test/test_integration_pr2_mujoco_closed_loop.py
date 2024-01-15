from copy import deepcopy
from typing import Optional

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_matrix, quaternion_about_axis

import giskardpy.utils.tfwrapper as tf
from giskard_msgs.msg import MoveResult, MoveGoal
from giskardpy.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy.configs.giskard import Giskard
from giskardpy.configs.iai_robots.pr2 import PR2CollisionAvoidance, PR2VelocityMujocoInterface, WorldWithPR2Config
from giskardpy.configs.qp_controller_config import QPControllerConfig
from giskardpy.configs.world_config import WorldWithOmniDriveRobot
from giskardpy.data_types import JointStates
from giskardpy.goals.goal import WEIGHT_BELOW_CA
from test_integration_pr2 import PR2TestWrapper, TestJointGoals, pocky_pose
from utils_for_tests import JointGoalChecker


class PR2TestWrapperMujoco(PR2TestWrapper):
    better_pose = {'r_shoulder_pan_joint': -1.7125,
                   'r_shoulder_lift_joint': -0.25672,
                   'r_upper_arm_roll_joint': -1.46335,
                   'r_elbow_flex_joint': -2.12,
                   'r_forearm_roll_joint': 1.76632,
                   'r_wrist_flex_joint': -0.10001,
                   'r_wrist_roll_joint': 0.05106,
                   'l_shoulder_pan_joint': 1.9652,
                   'l_shoulder_lift_joint': - 0.26499,
                   'l_upper_arm_roll_joint': 1.3837,
                   'l_elbow_flex_joint': -2.12,
                   'l_forearm_roll_joint': 16.99,
                   'l_wrist_flex_joint': - 0.10001,
                   'l_wrist_roll_joint': 0,
                   'torso_lift_joint': 0.2,
                   # 'l_gripper_l_finger_joint': 0.55,
                   # 'r_gripper_l_finger_joint': 0.55,
                   'head_pan_joint': 0,
                   'head_tilt_joint': 0,
                   }

    def __init__(self):
        del self.default_pose['l_gripper_l_finger_joint']
        del self.default_pose['r_gripper_l_finger_joint']
        self.r_tip = 'r_gripper_tool_frame'
        self.l_tip = 'l_gripper_tool_frame'
        self.l_gripper_group = 'l_gripper'
        self.r_gripper_group = 'r_gripper'
        # self.r_gripper = rospy.ServiceProxy('r_gripper_simulator/set_joint_states', SetJointState)
        # self.l_gripper = rospy.ServiceProxy('l_gripper_simulator/set_joint_states', SetJointState)
        self.mujoco_reset = rospy.ServiceProxy('mujoco/reset', Trigger)
        self.odom_root = 'odom_combined'
        giskard = Giskard(world_config=WorldWithPR2Config(),
                          collision_avoidance_config=PR2CollisionAvoidance(),
                          robot_interface_config=PR2VelocityMujocoInterface(),
                          behavior_tree_config=ClosedLoopBTConfig(),
                          qp_controller_config=QPControllerConfig())
        super().__init__(giskard)

    def reset_base(self):
        p = PoseStamped()
        p.header.frame_id = tf.get_tf_root()
        p.pose.orientation.w = 1
        self.set_localization(p)
        self.wait_heartbeats()

    def set_localization(self, map_T_odom: PoseStamped):
        pass

    def teleport_base(self, goal_pose, group_name: Optional[str] = None):
        self.allow_all_collisions()
        self.move_base(goal_pose)

    def reset(self):
        self.mujoco_reset()
        super().reset()


@pytest.fixture(scope='module')
def giskard(request, ros):
    c = PR2TestWrapperMujoco()
    request.addfinalizer(c.tear_down)
    return c


class TestJointGoalsMujoco(TestJointGoals):
    def test_joint_goal(self, zero_pose: PR2TestWrapper):
        js = {
            # 'torso_lift_joint': 0.2999225173357618,
            'head_pan_joint': 0.041880780651479044,
            'head_tilt_joint': -0.37,
            'r_upper_arm_roll_joint': -0.9487714747527726,
            'r_shoulder_pan_joint': -1.0047307505973626,
            'r_shoulder_lift_joint': 0.48736790658811985,
            'r_forearm_roll_joint': -14.895833882874182,
            'r_elbow_flex_joint': -1.392377908925028,
            'r_wrist_flex_joint': -0.4548695149411013,
            'r_wrist_roll_joint': 0.11426798984097819,
            'l_upper_arm_roll_joint': 1.7383062350263658,
            'l_shoulder_pan_joint': 1.8799810286792007,
            'l_shoulder_lift_joint': 0.011627231224188975,
            'l_forearm_roll_joint': 312.67276414458695,
            'l_elbow_flex_joint': -2.0300928925694675,
            'l_wrist_flex_joint': -0.10014623223021513,
            'l_wrist_roll_joint': -6.062015047706399,
        }
        zero_pose.set_joint_goal(js)
        zero_pose.allow_all_collisions()
        zero_pose.plan_and_execute()


class TestMoveBaseGoals:
    def test_left_1m(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'odom_combined'
        base_goal.pose.position.y = 1
        base_goal.pose.orientation.w = 1
        zero_pose.move_base(base_goal)

    def test_left_1cm(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.y = 0.01
        base_goal.pose.orientation.w = 1
        zero_pose.move_base(base_goal)

    def test_forward_left_rotate(self, zero_pose: PR2TestWrapper):
        map_T_odom = PoseStamped()
        map_T_odom.header.frame_id = 'map'
        map_T_odom.pose.position.x = 1
        map_T_odom.pose.position.y = 1
        map_T_odom.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 3, [0, 0, 1]))
        zero_pose.set_localization(map_T_odom)

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.position.y = -1
        # base_goal.pose.orientation.w = 1
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 4, [0, 0, 1]))
        zero_pose.allow_all_collisions()
        zero_pose.move_base(base_goal)

    def test_circle(self, zero_pose: PR2TestWrapper):
        center = PointStamped()
        center.header.frame_id = zero_pose.default_root
        zero_pose.set_json_goal(constraint_type='Circle',
                                center=center,
                                radius=0.3,
                                tip_link='base_footprint',
                                scale=0.1)
        # zero_pose.set_json_goal('PR2CasterConstraints')
        zero_pose.set_max_traj_length(new_length=160)
        zero_pose.allow_all_collisions()
        zero_pose.plan_and_execute()

    def test_stay_put(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = zero_pose.default_root
        # base_goal.pose.position.y = 0.05
        base_goal.pose.orientation.w = 1
        # zero_pose.set_json_goal('PR2CasterConstraints')
        zero_pose.set_joint_goal(zero_pose.better_pose)
        zero_pose.move_base(base_goal)

    def test_forward_1m(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.orientation.w = 1
        zero_pose.allow_all_collisions()
        zero_pose.move_base(base_goal)

    def test_carry_my_bs(self, zero_pose: PR2TestWrapper):
        # zero_pose.set_json_goal('CarryMyBullshit',
        #                         camera_link='head_mount_kinect_rgb_optical_frame',
        #                         laser_topic_name='/laser',
        #                         height_for_camera_target=1.5)
        # zero_pose.allow_all_collisions()
        # zero_pose.plan_and_execute(expected_error_codes=[MoveResult.PREEMPTED], stop_after=10)
        #
        # zero_pose.set_json_goal('CarryMyBullshit',
        #                         camera_link='head_mount_kinect_rgb_optical_frame',
        #                         laser_topic_name='/laser',
        #                         clear_path=True,
        #                         height_for_camera_target=1.5)
        # zero_pose.allow_all_collisions()
        # zero_pose.plan_and_execute(expected_error_codes=[MoveResult.PREEMPTED], stop_after=10)

        zero_pose.set_json_goal('CarryMyBullshit',
                                camera_link='head_mount_kinect_rgb_optical_frame',
                                # point_cloud_laser_topic_name='',
                                laser_frame_id='base_laser_link',
                                height_for_camera_target=1.5)
        zero_pose.allow_all_collisions()
        # zero_pose.plan_and_execute(expected_error_codes=[MoveResult.PREEMPTED], stop_after=30)
        zero_pose.plan_and_execute(expected_error_codes=[MoveResult.ERROR])

        # zero_pose.set_json_goal('CarryMyBullshit',
        #                         camera_link='head_mount_kinect_rgb_optical_frame',
        #                         laser_topic_name='/laser',
        #                         drive_back=True)
        # zero_pose.allow_all_collisions()
        # zero_pose.plan_and_execute(expected_error_codes=[MoveResult.PREEMPTED], stop_after=10)

        zero_pose.set_json_goal('CarryMyBullshit',
                                camera_link='head_mount_kinect_rgb_optical_frame',
                                # laser_topic_name='/laser',
                                laser_frame_id='base_laser_link',
                                drive_back=True)
        zero_pose.allow_all_collisions()
        zero_pose.plan_and_execute()

    def test_wave(self, zero_pose: PR2TestWrapper):
        center = PointStamped()
        center.header.frame_id = zero_pose.default_root
        zero_pose.allow_all_collisions()
        zero_pose.set_json_goal(constraint_type='Wave',
                                center=center,
                                radius=0.05,
                                tip_link='base_footprint',
                                scale=2)
        zero_pose.set_joint_goal(zero_pose.better_pose, check=False)
        zero_pose.plan_and_execute()

    def test_forward_1cm(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 0.01
        base_goal.pose.orientation.w = 1
        zero_pose.move_base(base_goal)

    def test_forward_left_1m_1m(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.position.y = 1
        base_goal.pose.orientation.w = 1
        zero_pose.move_base(base_goal)

    def test_forward_left_1cm_1cm(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 0.01
        base_goal.pose.position.y = 0.01
        base_goal.pose.orientation.w = 1
        zero_pose.move_base(base_goal)

    def test_forward_right_and_rotate(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.position.y = -1
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(-pi / 4, [0, 0, 1]))
        zero_pose.move_base(base_goal)

    def test_forward_then_left(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(-pi / 4, [0, 0, 1]))
        zero_pose.move_base(base_goal)
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 0
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(pi / 3, [0, 0, 1]))
        zero_pose.move_base(base_goal)

    def test_rotate_pi_half(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(-pi / 2, [0, 0, 1]))
        zero_pose.allow_all_collisions()
        zero_pose.move_base(base_goal)

    def test_rotate_pi(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(pi, [0, 0, 1]))
        zero_pose.move_base(base_goal)

    def test_rotate_0_001_rad(self, zero_pose: PR2TestWrapper):
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(0.001, [0, 0, 1]))
        zero_pose.move_base(base_goal)


class TestWorldManipulation:
    def test_add_urdf_body(self, kitchen_setup: PR2TestWrapper):
        joint_goal = 0.2
        object_name = kitchen_setup.kitchen_name
        kitchen_setup.set_kitchen_js({'sink_area_left_middle_drawer_main_joint': joint_goal})
        joint_state = rospy.wait_for_message('/kitchen/joint_states', JointState, rospy.Duration(1))
        joint_state = JointStates.from_msg(joint_state)
        assert joint_state['sink_area_left_middle_drawer_main_joint'].position == joint_goal
        kitchen_setup.clear_world()
        try:
            kitchen_setup.set_object_joint_state(object_name, {})
        except KeyError:
            pass
        else:
            raise 'expected error'
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = 1
        p.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
        if kitchen_setup.is_standalone():
            js_topic = ''
            set_js_topic = ''
        else:
            js_topic = '/kitchen/joint_states'
            set_js_topic = '/kitchen/cram_joint_states'
        kitchen_setup.add_urdf(name=object_name,
                               urdf=rospy.get_param('kitchen_description'),
                               pose=p,
                               js_topic=js_topic,
                               set_js_topic=set_js_topic)
        kitchen_setup.wait_heartbeats(1)
        joint_state = kitchen_setup.get_group_info(object_name).joint_state
        joint_state = JointStates.from_msg(joint_state)
        assert joint_state['iai_kitchen/sink_area_left_middle_drawer_main_joint'].position == joint_goal

        joint_goal = 0.1
        kitchen_setup.set_kitchen_js({'sink_area_left_middle_drawer_main_joint': joint_goal})
        kitchen_setup.remove_group(object_name)
        try:
            kitchen_setup.set_object_joint_state(object_name, {})
        except KeyError:
            pass
        else:
            raise 'expected error'
        kitchen_setup.add_urdf(name=object_name,
                               urdf=rospy.get_param('kitchen_description'),
                               pose=p,
                               js_topic=js_topic,
                               set_js_topic=set_js_topic)
        kitchen_setup.wait_heartbeats(1)
        joint_state = kitchen_setup.get_group_info(object_name).joint_state
        joint_state = JointStates.from_msg(joint_state)
        assert joint_state['iai_kitchen/sink_area_left_middle_drawer_main_joint'].position == joint_goal


class TestConstraints:

    def test_SetSeedConfiguration(self, zero_pose: PR2TestWrapper):
        zero_pose.set_seed_configuration(seed_configuration=zero_pose.better_pose)
        zero_pose.set_joint_goal(zero_pose.default_pose)
        zero_pose.plan_and_execute(expected_error_codes=[MoveResult.CONSTRAINT_INITIALIZATION_ERROR])

    def test_bowl_and_cup(self, kitchen_setup: PR2TestWrapper):
        # FIXME
        # kernprof -lv py.test -s test/test_integration_pr2.py::TestCollisionAvoidanceGoals::test_bowl_and_cup
        bowl_name = 'bowl'
        cup_name = 'cup'
        percentage = 50
        drawer_handle = 'sink_area_left_middle_drawer_handle'
        drawer_joint = 'sink_area_left_middle_drawer_main_joint'
        # spawn cup
        cup_pose = PoseStamped()
        cup_pose.header.frame_id = 'iai_kitchen/sink_area_left_middle_drawer_main'
        # cup_pose.header.stamp = rospy.get_rostime() + rospy.Duration(0.5)
        cup_pose.pose.position = Point(0.1, 0.2, -.05)
        cup_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        kitchen_setup.add_cylinder(name=cup_name, height=0.07, radius=0.04, pose=cup_pose,
                                   parent_link='sink_area_left_middle_drawer_main')

        # spawn bowl
        bowl_pose = PoseStamped()
        bowl_pose.header.frame_id = 'iai_kitchen/sink_area_left_middle_drawer_main'
        bowl_pose.pose.position = Point(0.1, -0.2, -.05)
        bowl_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        kitchen_setup.add_cylinder(name=bowl_name, height=0.05, radius=0.07, pose=bowl_pose,
                                   parent_link='sink_area_left_middle_drawer_main')

        # grasp drawer handle
        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = drawer_handle
        bar_axis.vector.y = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = drawer_handle

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = kitchen_setup.l_tip
        tip_grasp_axis.vector.z = 1

        kitchen_setup.set_grasp_bar_goal(bar_center=bar_center,
                                         bar_axis=bar_axis,
                                         bar_length=0.4,
                                         tip_link=kitchen_setup.l_tip,
                                         tip_grasp_axis=tip_grasp_axis,
                                         root_link=kitchen_setup.default_root)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = kitchen_setup.l_tip
        x_gripper.vector.x = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = drawer_handle
        x_goal.vector.x = -1

        kitchen_setup.set_align_planes_goal(tip_link=kitchen_setup.l_tip,
                                            tip_normal=x_gripper,
                                            root_link=kitchen_setup.default_root,
                                            goal_normal=x_goal)
        # kitchen_setup.allow_all_collisions()
        kitchen_setup.plan_and_execute()

        # open drawer
        kitchen_setup.set_open_container_goal(tip_link=kitchen_setup.l_tip,
                                              environment_link=drawer_handle)
        kitchen_setup.plan_and_execute()
        kitchen_setup.set_kitchen_js({drawer_joint: 0.48})

        kitchen_setup.set_joint_goal(kitchen_setup.better_pose)
        base_pose = PoseStamped()
        base_pose.header.frame_id = 'map'
        base_pose.pose.position.y = 1
        base_pose.pose.position.x = .1
        base_pose.pose.orientation.w = 1
        kitchen_setup.move_base(base_pose)

        # grasp bowl
        l_goal = deepcopy(bowl_pose)
        l_goal.header.frame_id = 'iai_kitchen/sink_area_left_middle_drawer_main'
        l_goal.pose.position.z += .2
        l_goal.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 1, 0, 0],
                                                                      [0, 0, -1, 0],
                                                                      [-1, 0, 0, 0],
                                                                      [0, 0, 0, 1]]))
        kitchen_setup.set_cart_goal(goal_pose=l_goal,
                                    tip_link=kitchen_setup.l_tip,
                                    root_link=kitchen_setup.default_root)
        kitchen_setup.allow_collision(kitchen_setup.l_gripper_group, bowl_name)

        # grasp cup
        r_goal = deepcopy(cup_pose)
        r_goal.header.frame_id = 'iai_kitchen/sink_area_left_middle_drawer_main'
        r_goal.pose.position.z += .2
        r_goal.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 1, 0, 0],
                                                                      [0, 0, -1, 0],
                                                                      [-1, 0, 0, 0],
                                                                      [0, 0, 0, 1]]))
        kitchen_setup.set_avoid_joint_limits_goal(percentage=percentage)
        kitchen_setup.set_cart_goal(goal_pose=r_goal,
                                    tip_link=kitchen_setup.r_tip,
                                    root_link=kitchen_setup.default_root,
                                    weight=WEIGHT_BELOW_CA)
        kitchen_setup.plan_and_execute()

        l_goal.pose.position.z -= .2
        r_goal.pose.position.z -= .2
        kitchen_setup.set_cart_goal(goal_pose=l_goal,
                                    tip_link=kitchen_setup.l_tip,
                                    root_link=kitchen_setup.default_root)
        kitchen_setup.set_cart_goal(goal_pose=r_goal,
                                    tip_link=kitchen_setup.r_tip,
                                    root_link=kitchen_setup.default_root)
        kitchen_setup.set_avoid_joint_limits_goal(percentage=percentage)
        kitchen_setup.avoid_all_collisions(0.05)
        kitchen_setup.allow_collision(group1=kitchen_setup.robot_name, group2=bowl_name)
        kitchen_setup.allow_collision(group1=kitchen_setup.robot_name, group2=cup_name)
        kitchen_setup.plan_and_execute()

        kitchen_setup.update_parent_link_of_group(name=bowl_name, parent_link=kitchen_setup.l_tip)
        kitchen_setup.update_parent_link_of_group(name=cup_name, parent_link=kitchen_setup.r_tip)

        kitchen_setup.set_joint_goal(kitchen_setup.better_pose)
        kitchen_setup.plan_and_execute()
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'base_footprint'
        base_goal.pose.position.x = -.1
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
        kitchen_setup.move_base(base_goal)

        # place bowl and cup
        bowl_goal = PoseStamped()
        bowl_goal.header.frame_id = 'kitchen_island_surface'
        bowl_goal.pose.position = Point(.2, 0, .05)
        bowl_goal.pose.orientation = Quaternion(0, 0, 0, 1)

        cup_goal = PoseStamped()
        cup_goal.header.frame_id = 'kitchen_island_surface'
        cup_goal.pose.position = Point(.15, 0.25, .07)
        cup_goal.pose.orientation = Quaternion(0, 0, 0, 1)

        kitchen_setup.set_cart_goal(goal_pose=bowl_goal, tip_link=bowl_name, root_link=kitchen_setup.default_root)
        kitchen_setup.set_cart_goal(goal_pose=cup_goal, tip_link=cup_name, root_link=kitchen_setup.default_root)
        kitchen_setup.set_avoid_joint_limits_goal(percentage=percentage)
        kitchen_setup.avoid_all_collisions(0.05)
        kitchen_setup.plan_and_execute()

        kitchen_setup.detach_group(name=bowl_name)
        kitchen_setup.detach_group(name=cup_name)
        kitchen_setup.allow_collision(group1=kitchen_setup.robot_name, group2=cup_name)
        kitchen_setup.allow_collision(group1=kitchen_setup.robot_name, group2=bowl_name)
        kitchen_setup.set_joint_goal(kitchen_setup.better_pose)
        kitchen_setup.plan_and_execute()

    def test_open_close_dishwasher(self, zero_pose: PR2TestWrapper):
        def pub_gripper_goals(goal: str):
            if goal == 'open':
                pass

        # def mujoco_pose_to_posestamped(position, quaternion, frame_id : str = "odom"): # quaternion in mujoco is [w, x, y, z]
        #     p = PoseStamped()
        #     p.header.frame_id = frame_id
        #     p.pose.position.x = position[0]
        #     p.pose.position.y = position[1]
        #     p.pose.position.z = position[2]
        #     p.pose.orientation.w = quaternion[0]
        #     p.pose.orientation.x = quaternion[1]
        #     p.pose.orientation.y = quaternion[2]
        #     p.pose.orientation.z = quaternion[3]
        #     return p
        # def mujoco_pose_to_posestamped(position, quaternion, frame_id : str = "odom"): # quaternion in mujoco is [w, x, y, z]
        world_root_pose = tf.lookup_pose("map", "world")
        zero_pose.kitchen_name = 'iai_door'
        zero_pose.add_urdf(name=zero_pose.kitchen_name,
                           urdf=rospy.get_param('door_description'),
                           pose=world_root_pose,
                           js_topic='/mujoco/world_joint_states')
        ### Add all objects to the world
        handle_name = 'sink_area_dish_washer_door_handle'
        door_name = 'sink_area_dish_washer_door'
        door_joint = 'sink_area_dish_washer_door_joint'
        # dishwasher = 'sink_area_dish_washer_main'

        # ### Sink

        # zero_pose.add_box('sink_area_bottom', size=(0.295*2, 1.07*2, 0.038*2), pose= mujoco_pose_to_posestamped(
        # frame_id = "odom", position=(1.565, 0.3, 0.045), quaternion=(0, 0, 0, 1))) zero_pose.add_box(
        # 'sink_area_top', size=(0.295*2, 1.07*2, 0.02*2), pose= mujoco_pose_to_posestamped(frame_id = "odom",
        # position=(1.565, 0.3, 0.8), quaternion=(0, 0, 0, 1))) zero_pose.add_box('sink_area_right', size=(0.295*2,
        # 0.02*2, 0.449*2), pose= mujoco_pose_to_posestamped(frame_id = "odom", position=(1.565, -0.782, 0.37),
        # quaternion=(0, 0, 0, 1))) zero_pose.add_box('sink_area_left', size=(0.295*2, 0.02*2, 0.43*2),
        # pose= mujoco_pose_to_posestamped(frame_id = "odom", position=(1.565, 1.35, 0.35), quaternion=(0, 0, 0, 1)))

        # ### Dishwasher zero_pose.add_mesh(dishwasher,
        # mesh='package://mujoco_sim/model/partial_kitchen/assets/dishwasher_rot.stl',
        # pose=mujoco_pose_to_posestamped(position=(1.555, 0.2, 0.42), quaternion=(0, 0, 0, 1))) zero_pose.add_mesh(
        # door_name, mesh='package://mujoco_sim/model/partial_kitchen/assets/dishwasher_door.stl',
        # pose=mujoco_pose_to_posestamped(frame_id = dishwasher, position=(0.28, 0, -0.305), quaternion=(0.707,
        # -0.707107, 0, 0))) zero_pose.add_mesh(handle_name,
        # mesh='package://mujoco_sim/model/partial_kitchen/assets/Handle60.stl', pose=mujoco_pose_to_posestamped(
        # frame_id = door_name, position=(0.073, -0.58, 0), quaternion=(0.707, -0.707107, 0, 0)))

        p = PoseStamped()
        p.header.frame_id = 'odom_combined'
        p.pose.orientation.w = 1
        p.pose.position.x = -0.2
        p.pose.position.y = 0.2
        zero_pose.teleport_base(p)

        hand = zero_pose.r_tip
        door_obj = "door"
        open_goal_angle = np.pi / 5
        goal_angle = np.pi / 2.5
        # handle_frame_id = 'sink_area_dish_washer_door_handle'
        zero_pose.register_group(door_obj,
                                 zero_pose.kitchen_name,
                                 door_name)  # root link of the objects to avoid collision
        root_Pose_door = zero_pose.world.compute_fk_pose('map', door_name)  # door pose b4 rotation
        door_Pose_root = zero_pose.world.compute_fk_pose('sink_area_dish_washer_door', 'map')
        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_name
        bar_axis.vector.y = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_name

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = hand
        tip_grasp_axis.vector.z = 1

        zero_pose.set_json_goal('GraspBar',
                                root_link=zero_pose.default_root,
                                tip_link=hand,
                                tip_grasp_axis=tip_grasp_axis,
                                bar_center=bar_center,
                                bar_axis=bar_axis,
                                bar_length=.3)
        # zero_pose.allow_collision([], 'kitchen', [handle_name])
        # zero_pose.allow_all_collisions()

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = hand
        x_gripper.vector.x = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_name
        x_goal.vector.x = -1
        zero_pose.set_align_planes_goal(tip_link=hand,
                                        tip_normal=x_gripper,
                                        goal_normal=x_goal)
        # # zero_pose.allow_all_collisions()
        #
        zero_pose.plan_and_execute()
        # TODO : insert grasping routine here
        #
        # zero_pose.set_json_goal('Open',
        #                         tip_link=hand,
        #                         environment_link=handle_name,
        #                         goal_joint_state=open_goal_angle)
        # zero_pose.allow_all_collisions()
        zero_pose.allow_collision(group1=zero_pose.kitchen_name, group2=zero_pose.r_gripper_group)
        zero_pose.plan_and_execute()
        # zero_pose.set_kitchen_js({'sink_area_dish_washer_door_joint': goal_angle})
        print("world state ", zero_pose.world.state.to_position_dict())
        # tip_grasp_axis = Vector3Stamped()
        # tip_grasp_axis.header.frame_id = hand
        # tip_grasp_axis.vector.y = 1
        #
        # object_rotation_axis = Vector3Stamped()
        # object_rotation_axis.header.frame_id = door_name
        # object_rotation_axis.vector.z = 1
        # print("root_P_door b4 rot", root_Pose_door)
        # print("root_P_door after rot", zero_pose.world.compute_fk_pose('map', 'sink_area_dish_washer_door'))
        # ### Testingg
        # root_Pose_door = zero_pose.world.compute_fk_pose('map', 'sink_area_dish_washer_door')
        # door_Pose_root = zero_pose.world.compute_fk_pose('sink_area_dish_washer_door', 'map')
        # door_T_root = quaternion_matrix(np.array([door_Pose_root.pose.orientation.x,
        #                                           door_Pose_root.pose.orientation.y,
        #                                           door_Pose_root.pose.orientation.z,
        #                                           door_Pose_root.pose.orientation.w]))
        # door_T_root[:, 3] = np.array([door_Pose_root.pose.position.x, door_Pose_root.pose.position.y,
        #                               door_Pose_root.pose.position.z, 1])
        #
        # root_T_door = np.linalg.inv(door_T_root)
        #
        # root_Pose_intermediate_pt = PoseStamped()
        # root_Pose_intermediate_pt.header.frame_id = 'map'
        # root_Pose_intermediate_pt.pose.position.x = root_Pose_door.pose.position.x
        # root_Pose_intermediate_pt.pose.position.y = root_Pose_door.pose.position.y
        # root_Pose_intermediate_pt.pose.position.z = root_Pose_door.pose.position.z + 0.565
        #
        # ### 1. get the rotated pose
        # root_P_goal = np.dot(root_T_door, np.array([root_Pose_intermediate_pt.pose.position.x,
        #                                             root_Pose_intermediate_pt.pose.position.y,
        #                                             root_Pose_intermediate_pt.pose.position.z,
        #                                             1]))
        # print("goal _point : ", root_P_goal)
        # ### 2
        # door_P_goal = np.dot(door_T_root, root_P_goal)
        #
        # ### half unrotate??
        # r = rotation_matrix(goal_angle / 2, -1 * np.array([object_rotation_axis.vector.x,
        #                                                    object_rotation_axis.vector.y,
        #                                                    object_rotation_axis.vector.z]))
        #
        # door_P_r_goal = np.dot(r, door_P_goal)
        #
        # ## back to root
        # root_P_r_goal = np.dot(root_T_door, door_P_r_goal)
        # print("unrotated_pt :", root_P_r_goal)
        #
        # print("door_pose_root :", door_Pose_root)
        # root_P_intermediate_pt = np.array([[root_Pose_intermediate_pt.pose.position.x,
        #                                     root_Pose_intermediate_pt.pose.position.y,
        #                                     root_Pose_intermediate_pt.pose.position.z,
        #                                     1]])
        # door_P_intermediate_pt = np.dot(door_T_root, root_P_intermediate_pt.T)
        # print("door_P_intermediate_pt : ", door_P_intermediate_pt)
        # rot_mat = rotation_matrix(goal_angle / 2, np.array([object_rotation_axis.vector.x,
        #                                                     object_rotation_axis.vector.y,
        #                                                     object_rotation_axis.vector.z]))
        # door_P_rotated_pt = np.dot(rot_mat, door_P_intermediate_pt)
        # print("door_P_rotated_pt : ", door_P_rotated_pt)
        # root_P_rotated_pt = np.dot(root_T_door, door_P_rotated_pt)
        #
        # print("root_P_rotated_pt : ", root_P_rotated_pt)
        # # #
        # # test_point = PoseStamped()
        # # test_point.header.frame_id = 'map'
        # # test_point.pose.position.x = root_P_rotated_pt[0, 0]
        # # test_point.pose.position.y = root_P_rotated_pt[1, 0]
        # # test_point.pose.position.z = root_P_rotated_pt[2, 0]
        # # test_point.pose.orientation = Quaternion(0, 0.7071, 0, 0.7071)
        # # zero_pose.set_cart_goal(test_point, tip_link=hand, root_link='map')
        # #
        # # zero_pose.plan_and_execute()
        #
        # zero_pose.set_json_goal('AlignTipToPushObject',
        #                         root_link=zero_pose.default_root,
        #                         tip_link=hand,
        #                         door_object=door_name,
        #                         door_height=0.565,
        #                         # door_pose_before_rotation=root_Pose_door,
        #                         object_joint_name=door_joint,
        #                         # door_length=0.42,
        #                         tip_gripper_axis=tip_grasp_axis,
        #                         object_rotation_axis=object_rotation_axis)
        #
        # zero_pose.plan_and_execute()
        # object_normal = Vector3Stamped()
        # object_normal.header.frame_id = door_name
        # object_normal.vector.x = 1
        #
        # # handle = zero_pose.world.compute_fk_pose('map', 'sink_area_dish_washer_door_handle')
        #
        # # # Move to the opposite side of the handle
        # # behind_handle = PoseStamped()
        # # behind_handle.header.frame_id = 'map'
        # # behind_handle.pose.position.x = 1.285 - 0.445 / 2  # 1.285 - 0.445/2  # 1.285 - 0.84 = 0.4445
        # # behind_handle.pose.position.y = 0.05
        # # behind_handle.pose.position.z = 0.76
        # # # Align along the axis of rotation of the object.
        # # behind_handle.pose.orientation = Quaternion(0, 0.7071, 0, 0.7071)  # rotate along y axis as the door opens
        # # # along the y axis
        #
        # # zero_pose.set_cart_goal(behind_handle, tip_link=hand, root_link='map')
        #
        # # zero_pose.plan_and_execute()
        #
        # # # # close the gripper
        # zero_pose.set_kitchen_js({'r_gripper_l_finger_joint': 0.0})
        #
        # # # dishwasher dimensions - 0.0200, 0.5950, 1.365
        # # # from rviz length = 0.42, height = 0.565
        # # # C-------D
        # # # |       |
        # # # A-------B
        # # # lOW HEIGHT - 0.263 , Max LENGTH - 0.38  Min LENGTH - 0.0222, max height - 0.65
        # # A = np.array([[1.285, 0.42328, 0.3703]])
        # # B = np.array([[1.285, 0.0167, 0.3703]])
        # # C = np.array([[1.285, 0.4238, 0.6]])
        # # D = np.array([[1.285, 0.0167, 0.6]])
        #
        # # # l = A[0, 1] - B[0, 1]
        # # # h = C[0, 2] - A[0, 2]
        #
        # # ab = B - A
        # # ac = C - A
        # # n = np.cross(ab, ac)
        # # n_unit = n / np.linalg.norm(n)
        # #### starts here
        # hand_pose = zero_pose.world.compute_fk_pose('map', hand)
        # P = np.array([[hand_pose.pose.position.x, hand_pose.pose.position.y, hand_pose.pose.position.z]])
        # door_height = 0.7
        # door_length = 0.42
        #
        # min_y = 1 / 4
        # max_y = 3 / 4
        # min_z = -1 / 2
        # max_z = 1 / 2
        #
        # A = np.array([[root_Pose_door.pose.position.x, root_Pose_door.pose.position.y + door_length * max_y,
        #                root_Pose_door.pose.position.z + door_height * min_z]])
        #
        # B = np.array([[root_Pose_door.pose.position.x, root_Pose_door.pose.position.y + door_length * min_y,
        #                root_Pose_door.pose.position.z + door_height * min_z]])
        #
        # C = np.array([[root_Pose_door.pose.position.x, root_Pose_door.pose.position.y + door_length * max_y,
        #                root_Pose_door.pose.position.z + door_height * max_z]])
        #
        # ab = B - A
        # ac = C - A
        # ap = P - A
        # ab_len = np.linalg.norm(ab)
        # ac_len = np.linalg.norm(ac)
        # ab_unit = ab / ab_len
        # ac_unit = ac / ac_len
        #
        # proj_pt = np.matmul(np.vstack((ab_unit, ac_unit)).reshape(2, 3), ap.reshape(3, 1))
        # proj_pt_scaled = [proj_pt[0] / ab_len, proj_pt[1] / ac_len]
        # proj_pt_scaled = np.clip(proj_pt_scaled, 0.0, 1.0)
        # nearest = A + proj_pt_scaled[0] * ab + proj_pt_scaled[1] * ac
        #
        # # # Projection
        # # p = hand_array - np.dot(hand_array - A, n_unit.T) * n_unit
        # # print("before clipping ", p)
        # # p[0, 1] = np.clip(p[0, 1], B[0, 1], A[0, 1])
        # # p[0, 2] = np.clip(p[0, 2], A[0, 2], C[0, 2])
        #
        # point_m_p = np.hstack((nearest, np.ones((1, 1))))
        # print("computed point", point_m_p)
        #
        # # # compute point_door_p
        # tf_m_d = quaternion_matrix(np.array([root_Pose_door.pose.orientation.x, root_Pose_door.pose.orientation.y,
        #                                      root_Pose_door.pose.orientation.z, root_Pose_door.pose.orientation.w]))
        # tf_m_d[:, 3] = np.hstack((np.array([[root_Pose_door.pose.position.x, root_Pose_door.pose.position.y,
        #                                      root_Pose_door.pose.position.z]]), np.ones((1, 1))))
        #
        # tf_d_m = np.linalg.inv(tf_m_d)
        # point_door_p = np.matmul(tf_d_m, point_m_p.T)
        # print("point d_p: ", point_door_p)
        #
        # # # 2. rotate the point
        # rot_door = rotation_matrix(goal_angle, np.array([0, 0, 1]))  # z is the axis of rotation in the local frame
        # point_rotated_door_p = np.matmul(rot_door, point_door_p)
        # print("rotated point", point_rotated_door_p)
        #
        # # # 3. compute point_rotated_m_p
        # print("shapes: ", tf_m_d.shape, point_rotated_door_p.shape)
        # point_rotated_m_p = np.matmul(tf_m_d, point_rotated_door_p)
        # print("point_rotated_m_p", point_rotated_m_p[0], point_rotated_m_p[1], point_rotated_m_p[2])
        # # # 4. compute the goal pose
        #
        # desired_pose = PoseStamped()
        # desired_pose.header.frame_id = 'map'
        # desired_pose.pose.position.x = point_rotated_m_p[0, 0]  # 1.285*np.cos(np.pi/4)
        # desired_pose.pose.position.y = point_rotated_m_p[1, 0]
        # desired_pose.pose.position.z = point_rotated_m_p[2, 0]  # 0.6*np.sin(np.pi/4)
        # hand_pose_orientation = hand_pose.pose.orientation
        # desired_pose.pose.orientation = Quaternion(hand_pose_orientation.x, hand_pose_orientation.y,
        #                                            hand_pose_orientation.z, hand_pose_orientation.w)
        #
        # root_V_object_rotation_axis = Vector3Stamped()
        # root_V_object_rotation_axis.header.frame_id = "map"
        # root_V_object_rotation_axis.vector.y = -1
        #
        # zero_pose.set_json_goal('PushDoor',
        #                         root_link=zero_pose.default_root,
        #                         tip_link=hand,
        #                         door_object=door_name,
        #                         door_height=0.565,
        #                         door_length=0.42,
        #                         tip_gripper_axis=tip_grasp_axis,
        #                         root_V_object_rotation_axis=root_V_object_rotation_axis,
        #                         object_joint_name=door_joint,
        #                         root_V_object_normal=object_normal)
        #
        # zero_pose.allow_collision(group1=door_obj, group2=zero_pose.r_gripper_group)
        # zero_pose.plan_and_execute()
        #
        # # zero_pose.set_cart_goal(desired_pose, tip_link=hand, root_link='map')
        # # zero_pose.allow_collision(group1=door_obj, group2=zero_pose.r_gripper_group)
        # # zero_pose.plan_and_execute()
        # hand_pose_now = zero_pose.world.compute_fk_pose('map', hand)
        # pose_to_push_down = PoseStamped()
        # pose_to_push_down.header.frame_id = hand_pose_now.header.frame_id
        # pose_to_push_down.pose.position.x = hand_pose_now.pose.position.x
        # pose_to_push_down.pose.position.y = hand_pose_now.pose.position.y
        # pose_to_push_down.pose.position.z = hand_pose.pose.position.z - 0.15
        # pose_to_push_down.pose.orientation = hand_pose.pose.orientation
        #
        # # zero_pose.set_json_goal('CartesianPose',
        # #                             root_link='map',
        # #                             tip_link=hand,
        # #                             goal_pose=pose_to_push_down)
        #
        # # Qn to Simon: There is a minor difference compared to the normal drawer scenario, here the robot tip does not have to move
        # # with the handle. The force applied by the gripper can just move the object further
        # # (if the force is sufficient)
        # zero_pose.set_json_goal(constraint_type='Open',
        #                         tip_link=hand,
        #                         environment_link=handle_name,
        #                         goal_joint_state=1.3217)
        #
        # zero_pose.allow_collision(group1=door_obj, group2=zero_pose.r_gripper_group)
        # zero_pose.plan_and_execute()
        #
        # # test_pose = PoseStamped()
        # # test_pose.pose.position.x = 1.0
        # # test_pose.pose.position.y = -0.04
        # # test_pose.pose.position.z = 0.675
        # # test_pose.pose.orientation = Quaternion(0, 0.7071068, 0, 0.7071068)
        # #
        # # zero_pose.set_cart_goal(test_pose, tip_link=hand, root_link='map')
        # # zero_pose.allow_all_collisions()
        # # # zero_pose.allow_collision(group1=zero_pose.kitchen_name, group2=zero_pose.r_gripper_group)
        # # zero_pose.plan_and_execute()
        # #
        # # # align the gripper on the side opposite to the handle
        # # door_frame_id = 'sink_area_dish_washer_door'
        # #
        # # x_gripper = Vector3Stamped()
        # # x_gripper.header.frame_id = hand
        # # x_gripper.vector.x = 1
        # #
        # # x_goal = Vector3Stamped()
        # # x_goal.header.frame_id = door_frame_id
        # # x_goal.vector.x = 1
        # # zero_pose.set_align_planes_goal(tip_link=hand,
        # #                                     tip_normal=x_gripper,
        # #                                     goal_normal=x_goal)
        # # zero_pose.plan_and_execute()
        #
        # # zero_pose.set_json_goal('Open',
        # #                             tip_link=hand,
        # #                             environment_link=handle_name,
        # #                             goal_joint_state=0)
        # # zero_pose.allow_all_collisions()
        # # zero_pose.plan_and_execute()
        # # zero_pose.set_kitchen_js({'sink_area_dish_washer_door_joint': 0})


class TestActionServerEvents:
    def test_interrupt_way_points1(self, zero_pose: PR2TestWrapper):
        p = PoseStamped()
        p.header.frame_id = 'base_footprint'
        p.pose.position = Point(0, 0, 0)
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        zero_pose.set_cart_goal(deepcopy(p), 'base_footprint')
        zero_pose.add_cmd()
        p.pose.position.x += 10
        zero_pose.set_cart_goal(deepcopy(p), 'base_footprint')
        zero_pose.add_cmd()
        p.pose.position.x += 10
        zero_pose.set_cart_goal(p, 'base_footprint')
        zero_pose.plan_and_execute(expected_error_codes=[MoveResult.SUCCESS,
                                                         MoveResult.PREEMPTED,
                                                         MoveResult.PREEMPTED],
                                   stop_after=2)

        p = PoseStamped()
        p.header.frame_id = zero_pose.r_tip
        p.header.stamp = rospy.get_rostime()
        p.pose.position = Point(-0.1, 0, 0)
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        zero_pose.set_cart_goal(p, zero_pose.r_tip, zero_pose.default_root)

        zero_pose.add_cmd()
        p = PoseStamped()
        p.header.frame_id = zero_pose.r_tip
        p.header.stamp = rospy.get_rostime()
        p.pose.position = Point(0.0, -0.1, -0.1)
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        zero_pose.set_cart_goal(p, zero_pose.r_tip, zero_pose.default_root)

        zero_pose.add_cmd()
        p = PoseStamped()
        p.header.frame_id = zero_pose.r_tip
        p.header.stamp = rospy.get_rostime()
        p.pose.position = Point(0.1, 0.1, 0.1)
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        zero_pose.set_cart_goal(p, zero_pose.r_tip, zero_pose.default_root)

        zero_pose.plan_and_execute()

    def test_interrupt1(self, zero_pose: PR2TestWrapper):
        p = PoseStamped()
        p.header.frame_id = 'base_footprint'
        p.pose.position = Point(1, 0, 0)
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        zero_pose.set_cart_goal(p, 'base_footprint')
        zero_pose.allow_all_collisions()
        zero_pose.plan_and_execute(expected_error_codes=[MoveResult.PREEMPTED], stop_after=1)

    def test_interrupt2(self, zero_pose: PR2TestWrapper):
        p = PoseStamped()
        p.header.frame_id = 'base_footprint'
        p.pose.position = Point(2, 0, 0)
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        zero_pose.set_cart_goal(p, 'base_footprint')
        zero_pose.allow_all_collisions()
        zero_pose.plan_and_execute(expected_error_codes=[MoveResult.PREEMPTED], stop_after=6)

    def test_undefined_type(self, zero_pose: PR2TestWrapper):
        zero_pose.allow_all_collisions()
        zero_pose.send_goal(goal_type=MoveGoal.UNDEFINED,
                            expected_error_codes=[MoveResult.INVALID_GOAL])

    def test_empty_goal(self, zero_pose: PR2TestWrapper):
        zero_pose.cmd_seq = []
        zero_pose.plan_and_execute(expected_error_codes=[MoveResult.INVALID_GOAL])

    def test_plan_only(self, zero_pose: PR2TestWrapper):
        zero_pose.allow_self_collision()
        zero_pose.set_joint_goal(pocky_pose, check=False)
        zero_pose.add_goal_check(JointGoalChecker(zero_pose, zero_pose.default_pose))
        zero_pose.send_goal(goal_type=MoveGoal.PLAN_ONLY)

# kernprof -lv py.test -s test/test_integration_pr2.py
# time: [1-9][1-9]*.[1-9]* s
# import pytest
# pytest.main(['-s', __file__ + '::TestJointGoals::test_joint_goal2'])
# pytest.main(['-s', __file__ + '::TestConstraints::test_open_dishwasher_apartment'])
# pytest.main(['-s', __file__ + '::TestConstraints::test_bowl_and_cup'])
# pytest.main(['-s', __file__ + '::TestCollisionAvoidanceGoals::test_avoid_collision_go_around_corner'])
# pytest.main(['-s', __file__ + '::TestCollisionAvoidanceGoals::test_avoid_collision_box_between_boxes'])
# pytest.main(['-s', __file__ + '::TestCollisionAvoidanceGoals::test_avoid_self_collision'])
# pytest.main(['-s', __file__ + '::TestCollisionAvoidanceGoals::test_avoid_collision_at_kitchen_corner'])
# pytest.main(['-s', __file__ + '::TestWayPoints::test_waypoints2'])
# pytest.main(['-s', __file__ + '::TestCartGoals::test_keep_position3'])
