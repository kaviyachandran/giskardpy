from abc import ABC
from typing import Optional
import numpy as np

from geometry_msgs.msg import Vector3Stamped, PointStamped, Quaternion, PoseStamped
from tf.transformations import rotation_matrix, quaternion_matrix, quaternion_about_axis

from giskardpy import casadi_wrapper as cas
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA
from giskardpy.utils import tfwrapper as tf


class PushDoor(Goal):

    def __init__(self,
                 root_link: str,
                 tip_link: str,
                 door_object: str,
                 door_height: float,
                 door_length: float,
                 door_pose: PoseStamped,
                 tip_gripper_axis: Vector3Stamped,
                 object_rotation_axis: Vector3Stamped,
                 # When the normal is along x axis, then the plane is located along y-z axis
                 object_normal: Vector3Stamped,
                 object_rotation_angle: float,
                 # desired_goal_angle: float,
                 root_group: Optional[str] = None,
                 tip_group: Optional[str] = None,
                 reference_linear_velocity: float = 0.1,
                 reference_angular_velocity: float = 0.5,
                 weight: float = WEIGHT_BELOW_CA):
        """
            The objective is to push the object until desired rotation is reached
            """
        super().__init__()
        self.root = self.world.search_for_link_name(root_link, root_group)
        self.tip = self.world.search_for_link_name(tip_link, tip_group)
        self.door_object = self.world.search_for_link_name(door_object)

        tip_gripper_axis.header.frame_id = self.tip
        tip_gripper_axis.vector = tf.normalize(tip_gripper_axis.vector)
        object_rotation_axis.header.frame_id = self.door_object
        object_rotation_axis.vector = tf.normalize(object_rotation_axis.vector)

        self.tip_gripper_axis = tip_gripper_axis
        self.object_rotation_axis = object_rotation_axis
        self.object_rotated_angle = object_rotation_angle
        self.root_P_door_object = PointStamped()
        self.root_P_door_object.header.frame_id = self.root

        # Pose of the articulated object is at the center of the axis along which it rotates and not at the
        # center of the object. self.root_T_door = self.world.compute_fk_pose(self.root, self.door_object)
        self.root_T_door = door_pose
        self.root_T_door.header.frame_id = self.root
        self.root_P_door_object.point = self.root_T_door.pose.position
        self.door_height = door_height
        self.door_length = door_length
        self.object_normal = object_normal
        self.object_normal.header.frame_id = self.door_object

        self.reference_linear_velocity = reference_linear_velocity
        self.reference_angular_velocity = reference_angular_velocity
        self.weight = weight

        self.rotation_axis = np.argmax([self.object_rotation_axis.vector.x, self.object_rotation_axis.vector.y,
                                        self.object_rotation_axis.vector.z])
        self.normal_axis = np.argmax([self.object_normal.vector.x,
                                      self.object_normal.vector.y,
                                      self.object_normal.vector.z])

        self.axis = {0: "x", 1: "y", 2: "z"}

    def make_constraints(self):

        def point_array_to_pointstamped(point, frameid):
            p = PointStamped()
            p.header.frame_id = frameid
            p.point.x = point[0]
            p.point.y = point[1]
            p.point.z = point[2]
            return p

        # Goal to reach a point on the plane
        root_T_tip = self.get_fk(self.root, self.tip)
        root_P_bottom_left = PointStamped()  # A
        root_P_bottom_right = PointStamped()  # B
        root_P_top_left = PointStamped()  # C
        min_y = 0
        max_y = 0
        min_z = 0
        max_z = 0
        if self.axis[int(self.normal_axis)] == 'x':

            # Plane lies in Y-Z axis

            if self.axis[int(self.rotation_axis)] == 'y':
                min_y = -1 / 2
                max_y = 1 / 2
                min_z = 1 / 4
                max_z = 3 / 4
            elif self.axis[int(self.rotation_axis)] == 'z':
                min_y = 1 / 4
                max_y = 3 / 4
                min_z = -1 / 2
                max_z = 1 / 2

            root_P_bottom_left.header.frame_id = self.root
            root_P_bottom_left.point.x = self.root_P_door_object.point.x
            root_P_bottom_left.point.y = self.root_P_door_object.point.y + self.door_length * max_y
            root_P_bottom_left.point.z = self.root_P_door_object.point.z + self.door_height * min_z

            root_P_bottom_right.header.frame_id = self.root
            root_P_bottom_right.point.x = self.root_P_door_object.point.x
            root_P_bottom_right.point.y = self.root_P_door_object.point.y + self.door_length * min_y
            root_P_bottom_right.point.z = self.root_P_door_object.point.z + self.door_height * min_z

            root_P_top_left.header.frame_id = self.root
            root_P_top_left.point.x = self.root_P_door_object.point.x
            root_P_top_left.point.y = self.root_P_door_object.point.y + self.door_length * max_y
            root_P_top_left.point.z = self.root_P_door_object.point.z + self.door_height * max_z

        # dishwasher dimensions - 0.0200, 0.42, 0.565
        # C-------D
        # |       |
        # A-------B
        root_P_tip = root_T_tip.to_position()
        dist, nearest = cas.distance_point_to_rectangular_surface(root_P_tip, root_P_bottom_left,
                                                                  root_P_bottom_right,
                                                                  root_P_top_left)
        # root_P_nearest = PointStamped()
        # root_P_nearest.header.frame_id = self.root
        # root_P_nearest.point.x = nearest[0]
        # root_P_nearest.point.y = nearest[1]
        # root_P_nearest.point.z = nearest[2]

        door_T_root = self.get_fk(self.door_object, self.root)
        root_T_door = self.get_fk(self.root, self.door_object)
        door_P_nearest = cas.dot(door_T_root, nearest)

        # half_angle = self.object_rotated_angle / 2
        q = Quaternion()
        q.x = self.object_rotation_axis.vector.x * np.sin(self.object_rotated_angle),
        q.y = self.object_rotation_axis.vector.y * np.sin(self.object_rotated_angle),
        q.z = self.object_rotation_axis.vector.z * np.sin(self.object_rotated_angle),
        q.w = np.cos(self.object_rotated_angle)
        # q = cas.from_axis_angle(self.object_rotation_axis, self.goal_angle)
        rot_mat = cas.RotationMatrix(q)
        door_P_rotated_point = cas.dot(rot_mat, door_P_nearest)

        root_P_rotated_point = cas.dot(root_T_door, door_P_rotated_point)
        # door_P_rotated_point = point_array_to_pointstamped(door_P_rotated_point, self.door_object)
        # root_P_rotated_point = tf.transform_point(self.root, door_P_rotated_point)

        self.add_debug_expr('goal_point_on_plane', cas.Point3(root_P_rotated_point))
        self.add_point_goal_constraints(frame_P_current=root_P_tip,
                                        frame_P_goal=cas.Point3(root_P_rotated_point),
                                        reference_velocity=self.reference_linear_velocity,
                                        weight=self.weight)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.tip}/{self.door_object}'
