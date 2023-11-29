from abc import ABC
from typing import Optional
import numpy as np

from geometry_msgs.msg import Vector3Stamped, PointStamped, Quaternion, PoseStamped
from tf.transformations import rotation_matrix, quaternion_matrix, quaternion_about_axis

from giskardpy import casadi_wrapper as cas
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA
from giskardpy.utils import tfwrapper as tf


### ToDo:
# 1. First objective is to get to the closest point on the plane
# 2. a. Get the intermediate point (At the mid of opening)?
#    b.Second objective is to enforce a rotation constraint for the intermediate point
# 3. Find the jacobian matrix of the stretched out arm and then compute the det or svd
# (This should be the trigger to use a different maneuver)

class AlignTipToPushObject(Goal):
    # door object pose in dishwasher is along the hinge. The following goal is written with that as a template
    def __init__(self,
                 root_link: str,
                 tip_link: str,
                 door_object: str,
                 # you will need the intermediate point in the direction the object opens. In dishwasher along z axis
                 # in door along y axis?
                 door_height: float,  #
                 object_joint_name: str,
                 # door_length: float,
                 # door_pose_before_rotation: PoseStamped,
                 tip_gripper_axis: Vector3Stamped,
                 object_rotation_axis: Vector3Stamped,
                 # object_rotation_angle: float,
                 root_group: Optional[str] = None,
                 tip_group: Optional[str] = None,
                 reference_linear_velocity: float = 0.1,
                 reference_angular_velocity: float = 0.5,
                 weight: float = WEIGHT_BELOW_CA):
        """
        The objective is to reach an intermediate point before pushing the door
        """
        super().__init__()
        self.root = self.world.search_for_link_name(root_link, root_group)
        self.tip = self.world.search_for_link_name(tip_link, tip_group)
        self.door_object = self.world.search_for_link_name(door_object)
        self.object_joint_angle = \
            self.world.state.to_position_dict()[self.world.search_for_joint_name(object_joint_name)]

        tip_gripper_axis.header.frame_id = self.tip
        tip_gripper_axis.vector = tf.normalize(tip_gripper_axis.vector)
        object_rotation_axis.header.frame_id = self.door_object
        object_rotation_axis.vector = tf.normalize(object_rotation_axis.vector)

        self.tip_gripper_axis = tip_gripper_axis
        self.object_rotation_axis = object_rotation_axis
        self.root_P_door_object = PointStamped()
        self.root_P_door_object.header.frame_id = self.root

        # Pose of the articulated object is at the center of the axis along which it rotates and not at the center of
        # the object.
        # self.root_T_door = self.world.compute_fk_pose(self.root, self.door_object)
        # self.root_T_door = door_pose_before_rotation
        # self.root_T_door.header.frame_id = self.root
        # self.root_P_door_object.point = self.root_T_door.pose.position
        self.door_height = door_height
        # self.door_length = door_length

        # self.object_rotation_angle = object_rotation_angle
        self.reference_linear_velocity = reference_linear_velocity
        self.reference_angular_velocity = reference_angular_velocity
        self.weight = weight

    def make_constraints(self):
        root_T_tip = self.get_fk(self.root, self.tip)
        root_T_object = self.get_fk(self.root, self.door_object)
        object_V_object_rotation_axis = cas.Vector3(self.object_rotation_axis)
        tip_V_tip_grasp_axis = cas.Vector3(self.tip_gripper_axis)
        root_V_object_rotation_axis = cas.dot(root_T_object, object_V_object_rotation_axis)
        root_V_tip_grasp_axis = cas.dot(root_T_tip, tip_V_tip_grasp_axis)

        root_P_object = self.world.compute_fk_pose(self.root, self.door_object)

        root_P_intermediate_point = PointStamped()
        root_P_intermediate_point.header.frame_id = self.root
        root_P_intermediate_point.point.x = root_P_object.pose.position.x
        root_P_intermediate_point.point.y = root_P_object.pose.position.y
        root_P_intermediate_point.point.z = root_P_object.pose.position.z + self.door_height

        door_T_root = self.world.compute_fk_pose(self.door_object, self.root)

        # point w.r.t door
        door_P_intermediate_point = cas.dot(cas.TransMatrix(door_T_root), cas.Point3(root_P_intermediate_point))
        desired_angle = self.object_joint_angle * 0.5  # just chose 1/2 of the goal angle

        # find rotated point in local frame

        q = Quaternion()
        rot_vector = np.array([self.object_rotation_axis.vector.x, self.object_rotation_axis.vector.y,
                               self.object_rotation_axis.vector.z])
        q.x = rot_vector[0] * np.sin(desired_angle / 2),
        q.y = rot_vector[1] * np.sin(desired_angle / 2),
        q.z = rot_vector[2] * np.sin(desired_angle / 2),
        q.w = np.cos(desired_angle / 2)
        rot_mat = cas.RotationMatrix(q)

        door_P_rotated_point = cas.dot(rot_mat, cas.Point3(door_P_intermediate_point))

        root_P_rotated_point = cas.dot(cas.TransMatrix(root_T_object), cas.Point3(door_P_rotated_point))

        self.add_debug_expr('goal_point', cas.Point3(root_P_rotated_point))
        self.add_point_goal_constraints(frame_P_current=root_T_tip.to_position(),
                                        frame_P_goal=cas.Point3(root_P_rotated_point),
                                        reference_velocity=self.reference_linear_velocity,
                                        weight=self.weight)

        self.add_debug_expr('root_V_grasp_axis', root_V_tip_grasp_axis)
        self.add_debug_expr('root_V_object_axis', root_V_object_rotation_axis)
        self.add_vector_goal_constraints(frame_V_current=cas.Vector3(root_V_tip_grasp_axis),
                                         frame_V_goal=cas.Vector3(root_V_object_rotation_axis),
                                         reference_velocity=self.reference_angular_velocity,
                                         weight=self.weight)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.root}/{self.tip}'
