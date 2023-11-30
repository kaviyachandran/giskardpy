from typing import Optional
import numpy as np

from geometry_msgs.msg import Vector3Stamped, PointStamped, Quaternion

from giskardpy import casadi_wrapper as cas
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA
from giskardpy.utils import tfwrapper as tf


# Goal to reach the closest point on the plane
# Note : Pose of the articulated object is at the center of the axis of rotation (hinge) and not at the
# center of the object.
class PushDoor(Goal):

    def __init__(self,
                 root_link: str,
                 tip_link: str,
                 door_object: str,
                 door_height: float,
                 door_length: float,
                 tip_gripper_axis: Vector3Stamped,
                 root_V_object_rotation_axis: Vector3Stamped,
                 # normal is along x axis, plane is located along y-z axis
                 root_V_object_normal: Vector3Stamped,
                 object_joint_name: str,
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
        self.object_joint_angle = \
            self.world.state.to_position_dict()[self.world.search_for_joint_name(object_joint_name)]

        tip_gripper_axis.header.frame_id = self.tip
        tip_gripper_axis.vector = tf.normalize(tip_gripper_axis.vector)
        root_V_object_rotation_axis.header.frame_id = self.root
        root_V_object_rotation_axis.vector = tf.normalize(root_V_object_rotation_axis.vector)
        root_V_object_normal.header.frame_id = self.root
        root_V_object_normal.vector = tf.normalize(root_V_object_normal.vector)

        self.tip_gripper_axis = tip_gripper_axis
        self.object_rotation_axis = root_V_object_rotation_axis
        self.root_P_door_object = PointStamped()
        self.root_P_door_object.header.frame_id = self.root

        self.door_height = door_height
        self.door_length = door_length

        self.reference_linear_velocity = reference_linear_velocity
        self.reference_angular_velocity = reference_angular_velocity
        self.weight = weight

        self.axis = {0: "x", 1: "y", 2: "z"}

        self.rotation_axis = np.argmax(np.abs([root_V_object_rotation_axis.vector.x,
                                               root_V_object_rotation_axis.vector.y,
                                               root_V_object_rotation_axis.vector.z]))
        self.normal_axis = np.argmax([root_V_object_normal.vector.x,
                                      root_V_object_normal.vector.y,
                                      root_V_object_normal.vector.z])

    def make_constraints(self):
        root_T_tip = self.get_fk(self.root, self.tip)
        root_T_door = self.get_fk(self.root, self.door_object)

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

            root_T_door = self.world.compute_fk_pose(self.root, self.door_object)
            root_P_bottom_left.header.frame_id = self.root
            root_P_bottom_left.point.x = root_T_door.pose.position.x
            root_P_bottom_left.point.y = root_T_door.pose.position.y + self.door_length * max_y
            root_P_bottom_left.point.z = root_T_door.pose.position.z + self.door_height * min_z

            root_P_bottom_right.header.frame_id = self.root
            root_P_bottom_right.point.x = root_T_door.pose.position.x
            root_P_bottom_right.point.y = root_T_door.pose.position.y + self.door_length * min_y
            root_P_bottom_right.point.z = root_T_door.pose.position.z + self.door_height * min_z

            root_P_top_left.header.frame_id = self.root
            root_P_top_left.point.x = root_T_door.pose.position.x
            root_P_top_left.point.y = root_T_door.pose.position.y + self.door_length * max_y
            root_P_top_left.point.z = root_T_door.pose.position.z + self.door_height * max_z

        # dishwasher dimensions - 0.0200, 0.42, 0.565
        # C-------D
        # |       |
        # A-------B
        root_Pose_tip = self.world.compute_fk_pose(self.root, self.tip)
        root_P_tip = PointStamped()
        root_P_tip.header.frame_id = self.root
        root_P_tip.point.x = root_Pose_tip.pose.position.x
        root_P_tip.point.y = root_Pose_tip.pose.position.y
        root_P_tip.point.z = root_Pose_tip.pose.position.z
        dist, root_P_nearest = cas.distance_point_to_rectangular_surface(cas.Point3(root_P_tip),
                                                                         cas.Point3(root_P_bottom_left),
                                                                         cas.Point3(root_P_bottom_right),
                                                                         cas.Point3(root_P_top_left))

        door_T_root = self.world.compute_fk_pose(self.door_object, self.root)
        door_P_nearest = cas.dot(cas.TransMatrix(door_T_root), root_P_nearest)

        root_V_object_rotation_axis = cas.Vector3(self.object_rotation_axis)
        object_V_object_rotation_axis = cas.dot(cas.TransMatrix(door_T_root), root_V_object_rotation_axis)

        rot_mat = cas.RotationMatrix.from_axis_angle(cas.Vector3(object_V_object_rotation_axis),
                                                     self.object_joint_angle)
        door_P_rotated_point = cas.dot(rot_mat, door_P_nearest)

        root_P_rotated_point = cas.dot(cas.TransMatrix(root_T_door), cas.Point3(door_P_rotated_point))

        self.add_debug_expr('goal_point_on_plane', cas.Point3(root_P_rotated_point))
        self.add_debug_expr('A', cas.Point3(root_P_bottom_left))
        self.add_debug_expr('B', cas.Point3(root_P_bottom_right))
        self.add_debug_expr('C', cas.Point3(root_P_top_left))
        self.add_point_goal_constraints(frame_P_current=root_T_tip.to_position(),
                                        frame_P_goal=cas.Point3(root_P_rotated_point),
                                        reference_velocity=self.reference_linear_velocity,
                                        weight=self.weight)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.tip}/{self.door_object}'
