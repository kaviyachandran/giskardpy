from queue import Queue, Empty
from typing import Optional

import rospy
import rostopic
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from py_trees import Status
from pybullet import getAxisAngleFromQuaternion
from rospy import ROSException, AnyMsg

import giskardpy.utils.tfwrapper as tf
from giskardpy.casadi_wrapper import rotation_matrix_from_rpy
from giskardpy.data_types import JointStates
from giskardpy.exceptions import GiskardException
from giskardpy.model.joints import OmniDrive
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils import logging
from giskardpy.utils.math import quaternion_from_axis_angle, rpy_from_matrix, rpy_from_quaternion, quaternion_from_rpy
from giskardpy.utils.utils import catch_and_raise_to_blackboard


class SyncOdometry(GiskardBehavior):

    @profile
    def __init__(self, odometry_topic: str):
        self.odometry_topic = odometry_topic
        super().__init__(str(self))
        self.last_msg = None
        self.lock = Queue(maxsize=1)

    def __str__(self):
        return f'{super().__str__()} ({self.odometry_topic})'

    @catch_and_raise_to_blackboard
    @profile
    def setup(self, timeout=0.0):
        msg: Optional[Odometry] = None
        odom = True
        while msg is None and not rospy.is_shutdown():
            try:
                try:
                    msg = rospy.wait_for_message(self.odometry_topic, Odometry, rospy.Duration(1))
                except:
                    msg = rospy.wait_for_message(self.odometry_topic, PoseWithCovarianceStamped, rospy.Duration(1))
                    odom = False
                self.lock.put(msg)
            except ROSException as e:
                logging.logwarn(f'Waiting for topic \'{self.odometry_topic}\' to appear.')
        root_link = msg.header.frame_id
        try:
            child_link = msg.child_frame_id
        except:
            child_link = 'base_footprint'
        joints = self.world.compute_chain(root_link, child_link, True, False, True, True)
        if len(joints) != 1:
            raise GiskardException(f'Chain between {root_link} and {child_link} should be one joint, but its {joints}')
        self.brumbrum = joints[0]
        if odom:
            self.odometry_sub = rospy.Subscriber(self.odometry_topic, Odometry, self.cb, queue_size=1)
        else:
            self.odometry_sub = rospy.Subscriber(self.odometry_topic, PoseWithCovarianceStamped, self.cb, queue_size=1)

        return super().setup(timeout)

    def cb(self, data: Odometry):
        try:
            self.lock.get_nowait()
        except Empty:
            pass
        self.lock.put(data)

    @catch_and_raise_to_blackboard
    @profile
    def update(self):
        joint: OmniDrive = self.world.joints[self.brumbrum]
        try:
            odometry: Odometry = self.lock.get()
            pose = odometry.pose.pose
            self.last_msg = JointStates()
            self.world.state[joint.x_name].position = pose.position.x
            self.world.state[joint.y_name].position = pose.position.y
            self.world.state[joint.z_name].position = pose.position.z
            roll, pitch, yaw = rpy_from_quaternion(pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w)
            self.world.state[joint.roll_name].position = roll
            self.world.state[joint.pitch_name].position = pitch
            self.world.state[joint.yaw_name].position = yaw
            # q = quaternion_from_rpy(roll, pitch, 0)
            # self.world.state[joint.qx_name].position = q[0]
            # self.world.state[joint.qy_name].position = q[1]
            # self.world.state[joint.qz_name].position = q[2]
            # self.world.state[joint.qw_name].position = q[3]

        except Empty:
            pass
        self.world.notify_state_change()
        return Status.RUNNING
