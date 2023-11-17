from typing import Optional
import numpy as np

from geometry_msgs.msg import Vector3Stamped, PointStamped

from giskardpy import casadi_wrapper as cas
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA
from giskardpy.utils import tfwrapper as tf


### ToDo:
# 1. First objective is to get to the closest point on the plane
# 2. a. Get the intermediate point (At the mid of opening)?
#    b.Second objective is to enforce a rotation constraint for the intermediate point
# 3. Find the jacobian matrix of the stretched out arm and then compute the det or svd
# (This should be the trigger to use a different maneuver)

class PushDoor(Goal):
    def __init__(self,
                 root_link: str,
                 tip_link: str,
                 door_object: str,
                 door_height: float,
                 door_length: float,
                 object_normal: Vector3Stamped,
                 tip_gripper_axis: Vector3Stamped,
                 object_rotation_axis: Vector3Stamped,
                 root_group: Optional[str] = None,
                 tip_group: Optional[str] = None,
                 reference_linear_velocity: float = 0.1,
                 reference_angular_velocity: float = 0.5,
                 weight: float = WEIGHT_BELOW_CA):
        """
        The objective is to reach a point on the object that is closest to the tip.
        """
        super().__init__()
        self.root = root_link
        self.tip = tip_link
        self.door_object = door_object

        tip_gripper_axis = self.transform_msg(self.tip, tip_gripper_axis)
        tip_gripper_axis.vector = tf.normalize(tip_gripper_axis.vector)

        object_rotation_axis = self.transform_msg(self.root, object_rotation_axis)
        object_rotation_axis.vector = tf.normalize(object_rotation_axis.vector)

        self.tip_gripper_axis = tip_gripper_axis
        self.object_rotation_axis = object_rotation_axis
        self.root_P_door_object = PointStamped()
        self.root_P_door_object.header.frame_id = self.root

        # Pose of the articulated object is at the center of the axis along which it rotates and not at the center of
        # the object.
        self.root_P_door_object.point = tf.lookup_pose(self.root, self.door_object).pose.position
        self.door_height = door_height
        self.door_length = door_length

        self.object_normal = object_normal
        self.object_normal.vector = tf.normalize(object_normal.vector)

        self.reference_linear_velocity = reference_linear_velocity
        self.reference_angular_velocity = reference_angular_velocity
        self.weight = weight

    def make_constraints(self):
        root_T_tip = self.get_fk(self.root, self.tip)
        root_P_tip = root_T_tip.to_position()

        # use the normal to find where the plane lies
        # axis = np.argmax([self.object_normal.vector.x, self.object_normal.vector.y, self.object_normal.vector.z])
        # Lets assume that the door is always in the y-z plane and the normal is along the x axis by the object axis
        # convention in the wiki
            #

        rotation_axis = np.argmax([self.object_rotation_axis.vector.x, self.object_rotation_axis.vector.y, self.object_rotation_axis.vector.z])
        min_y = 0
        max_y = 0
        min_z = 0
        max_z = 0

        if rotation_axis == 1:
            min_y = -1/2
            max_y = 1/2
            min_z = 1/4
            max_z = 3/4
        elif rotation_axis == 2:
            min_y = 1/4
            max_y = 3/4
            min_z = -1/2
            max_z = 1/2

        root_P_bottom_left = PointStamped()  # A
        root_P_bottom_left.header.frame_id = self.root
        root_P_bottom_left.point.x = self.root_P_door_object.point.x
        root_P_bottom_left.point.y = self.root_P_door_object.point.y+self.door_length*max_y
        root_P_bottom_left.point.z = self.root_P_door_object.point.z+self.door_height*min_z

        root_P_bottom_right = PointStamped()  # B
        root_P_bottom_right.header.frame_id = self.root
        root_P_bottom_right.point.x = self.root_P_door_object.point.x
        root_P_bottom_right.point.y = self.root_P_door_object.point.y+self.door_length*min_y
        root_P_bottom_right.point.z = self.root_P_door_object.point.z+self.door_height*min_z

        root_P_top_left = PointStamped()  # C
        root_P_top_left.header.frame_id = self.root
        root_P_top_left.point.x = self.root_P_door_object.point.x
        root_P_top_left.point.y = self.root_P_door_object.point.y+self.door_length*max_y
        root_P_top_left.point.z = self.root_P_door_object.point.z+self.door_height*max_z

        # dishwasher dimensions - 0.0200, 0.42, 0.565
        # C-------D
        # |       |
        # A-------B
        dist, nearest = cas.distance_point_to_plane(root_P_tip, root_P_bottom_left, root_P_bottom_right,
                                                           root_P_top_left)

        self.add_point_goal_constraints(frame_P_current=root_P_tip,
                                        frame_P_goal=nearest,
                                        reference_velocity=self.reference_linear_velocity,
                                        weight=self.weight)

    def __str__(self):
        s = super().__str__(self)
        return f'{s}/{self.root}/{self.tip}'
