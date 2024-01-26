#!/usr/bin/env python

# Todo : Check isgrasp differently
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped
from mujoco_msgs.srv import ResetObject
from sensor_msgs.msg import JointState

import numpy as np
import giskardpy.utils.tfwrapper as tf
# from giskardpy.model.world import WorldTree
from giskardpy.python_interface import GiskardWrapper


class OpenDoor:
	"""  1. Plan the base
	2. Position the gripper to grasp the object
	3. Write a service to publish effort to ros gripper controller
	"""

	def __init__(self):
		self.giskard = GiskardWrapper()
		self.r_tip = 'r_gripper_tool_frame'
		self.l_tip = 'l_gripper_tool_frame'
		self.l_gripper_joint = 'l_gripper_l_finger_joint'
		self.root = 'odom_combined'
		self.default_joint_pose = {
			'r_elbow_flex_joint': -0.15,
			'r_forearm_roll_joint': 0,
			'r_shoulder_lift_joint': 0,
			'r_shoulder_pan_joint': 0,
			'r_upper_arm_roll_joint': 0,
			'r_wrist_flex_joint': -0.10001,
			'r_wrist_roll_joint': 0,
			'l_elbow_flex_joint': -0.15,
			'l_forearm_roll_joint': 0,
			'l_shoulder_lift_joint': 0,
			'l_shoulder_pan_joint': 0,
			'l_upper_arm_roll_joint': 0,
			'l_wrist_flex_joint': -0.10001,
			'l_wrist_roll_joint': 0,
			# 'torso_lift_joint': 0.2,
			'head_pan_joint': 0,
			'head_tilt_joint': 0,
			'l_gripper_l_finger_joint': 0.55,
			'r_gripper_l_finger_joint': 0.55
		}

		self.better_pose = {
			'r_shoulder_pan_joint': -1.7125,
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
			'l_gripper_l_finger_joint': 0.55,
			'r_gripper_l_finger_joint': 0.55,
			'head_pan_joint': 0,
			'head_tilt_joint': 0,
		}
		# self.grasp_joint_goal = 0.15
		self.elbow = "l_forearm_link"

		self.handle_name = 'sink_area_dish_washer_door_handle'
		self.door_name = 'sink_area_dish_washer_door'
		self.dishwasher_container = "sink_area_dish_washer_main"
		self.door_joint = 'sink_area_dish_washer_door_joint'

		self.robot_name = "pr2"
		self.world_name = "iai_door"
		self.handle_collision_group = "iai_dishwasher_door"
		self.gripper_collision_group = "l_gripper"
		self.gripper_publisher = rospy.Publisher(self.robot_name + "/l_gripper_controller/command",
		                                         Float64,
		                                         queue_size=50)

		self.is_gripper_closed = False

	def reset_joints(self):
		self.giskard.set_joint_goal(self.better_pose)
		self.giskard.plan_and_execute()
		print("joints reset")

	def reset_base_pose(self):
		print("base goal")
		base_pose = PoseStamped()
		base_pose.header.frame_id = 'base_footprint'
		base_pose.pose.position.x = -0.2
		base_pose.pose.position.y = 0.2
		base_pose.pose.orientation.w = 1
		self.giskard.set_cart_goal(goal_pose=base_pose, tip_link='base_footprint', root_link='map')
		self.giskard.plan_and_execute()

	def init_world(self):
		world_root_pose = tf.lookup_pose("map", "world")

		if self.world_name not in self.giskard.get_group_names():
			self.giskard.add_urdf(name=self.world_name,
			                      urdf=rospy.get_param('door_description'),
			                      pose=world_root_pose,
			                      js_topic='/mujoco/world_joint_states')

		if self.handle_collision_group not in self.giskard.get_group_names():
			self.giskard.register_group(new_group_name=self.handle_collision_group,
			                            root_link_group_name=self.world_name,
			                            root_link_name=self.dishwasher_container)  # root link of the objects to avoid collision

		if self.gripper_collision_group not in self.giskard.get_group_names():
			self.giskard.register_group(new_group_name=self.gripper_collision_group,
			                            root_link_group_name=self.robot_name,
			                            root_link_name='l_wrist_roll_link')

	def send_gripper_goal(self, state: str):
		# Below is for GripperController

		# rate = rospy.Rate(20)
		gripper_goal = 0
		if state == "close":
			gripper_goal = -200
		elif state == "open":
			gripper_goal = 200
		print("here--------------------")
		# ns = rospy.get_namespace()
		# print("ns ", ns)

		# rospy.wait_for_service("l_gripper_l_finger_controller/command")
		# print("wait over")
		rospy.sleep(1)
		self.gripper_publisher.publish(gripper_goal)

	# rate.sleep()

	def align_to_grasp(self):
		""" Add the urdf to giskard world """

		self.send_gripper_goal(state="open")
		print("open gripper")
		print("group names - align", self.giskard.get_group_names())
		# print("gripper group ", self.giskard.)

		# This is the axis of the handle along which the gripper has to be aligned with the handle
		bar_axis = Vector3Stamped()
		bar_axis.header.frame_id = self.handle_name
		bar_axis.vector.y = 1

		bar_center = PointStamped()
		bar_center.header.frame_id = self.handle_name

		tip_grasp_axis = Vector3Stamped()
		tip_grasp_axis.header.frame_id = self.l_tip
		tip_grasp_axis.vector.z = 1

		self.giskard.set_grasp_bar_goal(root_link=self.root,
		                                tip_link=self.l_tip,
		                                tip_grasp_axis=tip_grasp_axis,
		                                bar_center=bar_center,
		                                bar_axis=bar_axis,
		                                bar_length=.40)

		x_gripper = Vector3Stamped()
		x_gripper.header.frame_id = self.l_tip
		x_gripper.vector.x = 1

		x_goal = Vector3Stamped()
		x_goal.header.frame_id = self.handle_name
		x_goal.vector.x = -1
		self.giskard.set_align_planes_goal(root_link=self.root,
		                                   tip_link=self.l_tip,
		                                   tip_normal=x_gripper,
		                                   goal_normal=x_goal)

		self.giskard.allow_collision(self.handle_collision_group, self.gripper_collision_group)
		self.giskard.plan_and_execute()

		# hack - create an enclosure around the object so that the object does not slip away
		l_gripper_pose = tf.lookup_pose('map', self.l_tip)
		l_gripper_pose.pose.position.x = l_gripper_pose.pose.position.x + 0.025
		# l_gripper_pose.pose.position.y = l_gripper_pose.pose.position.y+0.1
		self.giskard.set_cart_goal(l_gripper_pose, tip_link=self.l_tip, root_link='base_link')

		self.giskard.allow_collision(self.handle_collision_group, self.gripper_collision_group)
		# self.giskard.allow_all_collisions()
		self.giskard.plan_and_execute()

		self.send_gripper_goal(state="close")
		print("close gripper")
		print("Done with grasp bar goal ...")

	def _grasp_check_callback(self, data):
		for (ind, val) in enumerate(data.name):
			if val == self.l_gripper_joint:
				print("pos ", data.position[ind])
				if data.position[ind] <= 0.15:  # and val.velocity[ind] < 0.05:
					self.is_gripper_closed = True
				break

	def pull_handle(self):
		obj_goal_angle = np.pi/4

		obj_joint_states = rospy.wait_for_message('/mujoco/world_joint_states', JointState)
		current_door_joint_state = self.get_door_joint_state(obj_joint_states)
		while current_door_joint_state <= obj_goal_angle:
			goal_angle = (obj_goal_angle - current_door_joint_state)/2 + current_door_joint_state
			# self.send_gripper_goal(state="close")

		# sub = rospy.Subscriber("/mujoco/robot_joint_states", JointState, self._grasp_check_callback)
			data = rospy.wait_for_message("/mujoco/robot_joint_states", JointState, timeout=5)
			gripper_pose = tf.lookup_pose('map', self.l_tip) # TODO: Check if lookup_transform is faster
			handle_pose = tf.lookup_pose('map', self.handle_name)
			distance_between_gripper_handle = np.linalg.norm(np.array([gripper_pose.pose.position.x,
                                                                       gripper_pose.pose.position.y]) -
			                                                 np.array([handle_pose.pose.position.x,
			                                                           handle_pose.pose.position.y]))
			self._grasp_check_callback(data)
			if self.is_gripper_closed and distance_between_gripper_handle <= 0.4/2:  # 0,43 - length of the handle
				self.giskard.set_json_goal('Open',
				                           tip_link=self.l_tip,
				                           environment_link=self.handle_name,
				                           goal_joint_state=goal_angle,
				                           max_velocity=2)  # 2 rad/s and 1.46 m/s

				self.giskard.allow_collision(self.handle_collision_group, self.gripper_collision_group)
				self.giskard.plan_and_execute()  # send goal to Giskard
			else:
				self.align_to_grasp()

			obj_joint_states = rospy.wait_for_message('/mujoco/world_joint_states', JointState, timeout=2)
			if obj_joint_states:
				current_door_joint_state = self.get_door_joint_state(obj_joint_states)

	def push_door(self):
		# Get the goal state of the door
		# if it is grater than 45 degrees then align to push and then push the door
		self.send_gripper_goal(state="open")

		obj_joint_states = rospy.wait_for_message('/mujoco/world_joint_states', JointState)
		door_joint_value = 0
		if obj_joint_states:
			door_joint_value = self.get_door_joint_state(obj_joint_states)

		if door_joint_value >= np.pi / 6: # ToDo: update this it should be close to the goal angle set
			tip_grasp_axis = Vector3Stamped()
			tip_grasp_axis.header.frame_id = self.l_tip
			tip_grasp_axis.vector.y = 1

			# Todo: One can get this from giskard as the urdf is added to giskards world?
			object_rotation_axis = Vector3Stamped()
			object_rotation_axis.header.frame_id = self.door_name
			object_rotation_axis.vector.z = 1
			self.giskard.set_json_goal('AlignTipToPushObject',
			                           root_link=self.root,
			                           tip_link=self.l_tip,
			                           door_object=self.door_name,
			                           door_height=0.73,
			                           # door_pose_before_rotation=root_Pose_door,
			                           object_joint_name=self.door_joint,
			                           # door_length=0.42,
			                           tip_gripper_axis=tip_grasp_axis,
			                           object_rotation_axis=object_rotation_axis)

			self.giskard.allow_collision(self.handle_collision_group, self.gripper_collision_group)
			self.giskard.plan_and_execute()  # send goal to Giskard

			# ToDo: This can be computed. So if it rotates along z axis w.r.t dishwasher main then compute the value
			# w.r.t map
			root_V_object_rotation_axis = Vector3Stamped()
			root_V_object_rotation_axis.header.frame_id = "map"
			root_V_object_rotation_axis.vector.y = -1

			object_normal = Vector3Stamped()
			object_normal.header.frame_id = self.door_name
			object_normal.vector.x = 1

			self.giskard.set_json_goal('PushDoor',
			                           root_link=self.root,
			                           tip_link=self.l_tip,
			                           door_object=self.door_name,
			                           door_height=0.73,
			                           door_length=0.54,
			                           tip_gripper_axis=tip_grasp_axis,
			                           root_V_object_rotation_axis=root_V_object_rotation_axis,
			                           object_joint_name=self.door_joint,
			                           root_V_object_normal=object_normal)

			self.giskard.allow_collision(self.handle_collision_group, self.gripper_collision_group)
			self.giskard.plan_and_execute()  # send goal to Giskard

			self.giskard.set_json_goal(constraint_type='Open',
			                           tip_link=self.l_tip,
			                           environment_link=self.handle_name,
			                           goal_joint_state=1.3217)

			self.giskard.allow_collision(self.handle_collision_group, self.gripper_collision_group)
			self.giskard.plan_and_execute()  # send goal to Giskard

	def get_door_joint_state(self, data) -> float:
		for ind, val in enumerate(data.name):
			if val == self.door_joint:
				return data.position[ind]

	def reset_object_state(self):
		obj_joint_states = rospy.wait_for_message('/mujoco/world_joint_states', JointState)
		joint_value = 0
		if obj_joint_states:
			door_joint_value = self.get_door_joint_state(obj_joint_states)
			if door_joint_value != 0:
				rospy.wait_for_service('/mujoco/reset_object_joint_state')
				try:
					print("resetting object joints ....")
					reset_object_joint = rospy.ServiceProxy('/mujoco/reset_object_joint_state', ResetObject)
					return reset_object_joint(self.door_joint, joint_value)
				except rospy.ServiceException as e:
					print("Service call failed: %s" % e)


if __name__ == "__main__":
	""" Note to self : launch giskard as you create an instance of the python wrapper."""
	rospy.init_node("open_door")
	open_door = OpenDoor()
	# """ Wait for message on the topic and if the message is received, shut down """

	# print('joint states:{}'.format(obj_joint_states))
	# open_door.reset_object_state()
	# open_door.init_world()
	# open_door.reset_base_pose()
	open_door.reset_joints()
	open_door.align_to_grasp()
	open_door.pull_handle()
	open_door.push_door()

# rospy.spin()
