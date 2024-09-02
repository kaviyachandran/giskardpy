import numpy as np
import rospy

from geometry_msgs.msg import TransformStamped, QuaternionStamped, PointStamped, PoseStamped, Vector3Stamped, Point
from std_msgs.msg import String

from giskardpy.python_interface.python_interface import GiskardWrapper
from giskardpy.goals.pouring import AdaptivePouring
from giskardpy.tasks.task import Task
from giskardpy.utils.tfwrapper import lookup_transform, lookup_pose
from giskard_msgs.msg import MoveResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# obj_goal = PoseStamped()
# obj_goal.header.frame_id = self.source
# obj_goal.pose.position.x = -0.2
# obj_goal.pose.position.y = 0.8
# obj_goal.pose.position.z = 0.5
# obj_goal.pose.orientation.w = 1
# self.giskard.motion_goals.add_cartesian_pose(goal_pose=obj_goal,
#                                         tip_link=self.source,
#                                         root_link='map')
#
# self.giskard.add_default_end_motion_conditions()  # it was not executing without this line
# rospy.loginfo('Send cartesian goal for box.')
# result: MoveResult = self.giskard.execute()
#
# if isinstance(result, MoveResult):
#     print("there is something: ", result.error.code)
#     for i in result.trajectory.points:
#         print("ha ", i.positions)
# rospy.loginfo('Goal reached')

rospy.init_node("test_giskard")
giskard = GiskardWrapper()

source = 'free_cup'
dest = 'free_cup2'
root = 'map'


def adaptive_tilt():
    print("in adaptive tilt")
    dest_pose = PoseStamped()
    dest_dim = (0.06, 0.06, 0.18)

    dest_pose.header.frame_id = 'free_cup2'
    dest_pose = lookup_pose(root, dest)
    dest_pose.pose.position.x = dest_pose.pose.position.x + dest_dim[0]/2
    dest_pose.pose.position.y = dest_pose.pose.position.y + dest_dim[1]*2
    dest_pose.pose.position.z = dest_pose.pose.position.z + dest_dim[2]/2 + 0.03

    src_pose = PoseStamped()

    src_pose.header.frame_id = 'free_cup'
    src_pose = lookup_pose(root, source)
    src_pose.pose.position.z = src_pose.pose.position.z + 0.09

    # goal_pose.pose.position.x = 1.95
    # goal_pose.pose.position.y = -0.4
    # goal_pose.pose.position.z = 0.49
    # goal_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
    #                                                                  [0, -1, 0, 0],
    #                                                                  [1, 0, 0, 0],
    #                                                                  [0, 0, 0, 1]]))

    map_P_dest_side_toward_src = PointStamped()
    map_P_dest_side_toward_src.point.x = dest_pose.pose.position.x
    map_P_dest_side_toward_src.point.y = dest_pose.pose.position.y - 0.03 / 2
    map_P_dest_side_toward_src.point.z = dest_pose.pose.position.z

    map_P_src_side_toward_dest = PointStamped()
    map_P_src_side_toward_dest.point.x = src_pose.pose.position.x
    map_P_src_side_toward_dest.point.y = src_pose.pose.position.y + 0.03 / 2
    map_P_src_side_toward_dest.point.z = src_pose.pose.position.z

    dest_V_src = [map_P_dest_side_toward_src.point.x - map_P_src_side_toward_dest.point.x,
                  map_P_dest_side_toward_src.point.y - map_P_src_side_toward_dest.point.y,
                  map_P_dest_side_toward_src.point.y - map_P_src_side_toward_dest.point.z]
    tilt_axis = Vector3Stamped()
    tilt_axis.header.frame_id = 'free_cup'
    tilt_axis.vector.x = 0.7071
    tilt_axis.vector.y = -0.7071
    tilt_axis.vector.z = 0

    goal_point: PointStamped = PointStamped()
    goal_point.header.frame_id = 'map'
    goal_point.point.x = dest_pose.pose.position.x
    goal_point.point.y = dest_pose.pose.position.y + 0.03
    goal_point.point.z = dest_pose.pose.position.z + 0.09

    # giskard.motion_goals.add_cartesian_position(goal_point=goal_point,
    #                                             tip_link=source,
    #                                             root_link=root)

    giskard.motion_goals.add_motion_goal(motion_goal_class=AdaptivePouring.__name__,
                                         name='pouring',
                                         tip_link='free_cup',
                                         root_link='map',
                                         tilt_angle=1,
                                         pouring_pose=dest_pose,
                                         tilt_axis=tilt_axis,
                                         pre_tilt=True)
    giskard.execute()
    print("Done")


adaptive_tilt()

# class GiskardListener(object):
#
#     def __init__(self):
#         # rospy.Subscriber('/reasoner/concluded_behaviors', String, self.reasoner_callback)
#
#         self.change_in_orientation = (5 / 180) * 3.14  # 5 degrees
#         self.change_in_position = 0.05  # 5 cm
#         self.max_rotation = np.deg2rad(135)
#
#         self.giskard = GiskardWrapper()
#
#         print('Instantiating Giskard wrapper.')
#         self.source = 'free_cup'
#         self.dest = 'free_cup2'
#         self.root = 'odom'
#
#     def adaptive_tilt(self):
#         print("in adaptive tilt")
#         p = PoseStamped()
#
#         p.header.frame_id = 'free_cup2'
#         p = lookup_pose(self.root, self.dest)
#         p.pose.position.z = p.pose.position.z + 0.05
#
#         # goal_pose.pose.position.x = 1.95
#         # goal_pose.pose.position.y = -0.4
#         # goal_pose.pose.position.z = 0.49
#         # goal_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
#         #                                                                  [0, -1, 0, 0],
#         #                                                                  [1, 0, 0, 0],
#         #                                                                  [0, 0, 0, 1]]))
#         tilt_axis = Vector3Stamped()
#         tilt_axis.header.frame_id = 'free_cup'
#         tilt_axis.vector.x = 1
#
#         self.giskard.motion_goals.add_motion_goal(goal=PouringAdaptiveTilt.__name__,
#                                                   name='pouring',
#                                                   tip='free_cup',
#                                                   root='map',
#                                                   tilt_angle=1,
#                                                   pouring_pose=p,
#                                                   tilt_axis=tilt_axis)
#         self.giskard.execute()
#
#     def tilt(self, pose, change: str, rotation_dir: np.array = np.array([])):
#         print(f"self.dest pose: {pose.transform.translation, pose.transform.rotation}")
#         # + 5 degrees along the rotation dir
#         euler_angles_radians = list(euler_from_quaternion([pose.transform.rotation.x, pose.transform.rotation.y,
#                                                            pose.transform.rotation.z, pose.transform.rotation.w]))
#         a_max = 0
#         a_min = 0
#         goal_rotation = QuaternionStamped()
#         angle_update_radians = np.zeros(3)
#
#         if rotation_dir.size == 0:
#             if change == "decrease":  # until 0 degrees
#                 if euler_angles_radians[0] > euler_angles_radians[1]:
#                     a_max = euler_angles_radians[0]
#                     angle_update_radians[0] = -self.change_in_orientation
#                 elif euler_angles_radians[0] == euler_angles_radians[1]:
#                     a_max = euler_angles_radians[0]
#                     angle_update_radians[0] = -self.change_in_orientation
#                 else:
#                     a_max = euler_angles_radians[1]
#                     angle_update_radians[1] = -self.change_in_orientation
#                 print(f"before clipping: {euler_angles_radians[0], a_max, a_min}")
#                 euler_angles_radians[0:2] = np.clip(euler_angles_radians[0:2], a_max=self.max_rotation, a_min=a_min)
#
#             elif change == "increase":  # until 135 degrees
#                 if euler_angles_radians[0] > euler_angles_radians[1]:
#                     a_min = euler_angles_radians[0]
#                     angle_update_radians[0] = self.change_in_orientation
#                 elif euler_angles_radians[0] == euler_angles_radians[1]:
#                     a_min = euler_angles_radians[0]
#                     angle_update_radians[0] = self.change_in_orientation
#                 else:
#                     a_min = euler_angles_radians[1]
#                     angle_update_radians[1] = self.change_in_orientation
#
#                 euler_angles_radians[0:2] = np.clip(euler_angles_radians[0:2], a_min=a_min, a_max=self.max_rotation)
#
#             goal_rotation.header.frame_id = self.source
#             quat = quaternion_from_euler(angle_update_radians[0], angle_update_radians[1],
#                                          angle_update_radians[2], 'rxyz')
#             print(f'quat: {quat}')
#             goal_rotation.quaternion.x = quat[0]
#             goal_rotation.quaternion.y = quat[1]
#             goal_rotation.quaternion.z = quat[2]
#             goal_rotation.quaternion.w = quat[3]
#
#         self.giskard.motion_goals.add_cartesian_orientation(goal_orientation=goal_rotation,
#                                                             tip_link=self.source,
#                                                             root_link=self.dest)
#
#     def move_up(self):
#         goal_position = PointStamped()
#         goal_position.header.frame_id = self.source
#         goal_position.point.z = self.change_in_position
#
#         self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
#                                                          tip_link=self.source,
#                                                          root_link=self.dest)
#
#     def move_down(self):
#         goal_position = PointStamped()
#         goal_position.header.frame_id = self.source
#         goal_position.point.z = -self.change_in_position
#
#         self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
#                                                          tip_link=self.source,
#                                                          root_link=self.dest)
#
#     def move_right(self):
#         goal_position = PointStamped()
#         goal_position.header.frame_id = self.source
#         goal_position.point.y = -self.change_in_position
#
#         self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
#                                                          tip_link=self.source,
#                                                          root_link=self.dest)
#
#     def move_left(self):
#         goal_position = PointStamped()
#         goal_position.header.frame_id = self.source
#         goal_position.point.y = self.change_in_position
#
#         self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
#                                                          tip_link=self.source,
#                                                          root_link=self.dest)
#
#     def move_forward(self):
#         goal_position = PointStamped()
#         goal_position.header.frame_id = self.source
#         goal_position.point.x = self.change_in_position
#
#         self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
#                                                          tip_link=self.source,
#                                                          root_link=self.dest)
#
#     def move_back(self):
#         goal_position = PointStamped()
#         goal_position.header.frame_id = self.source
#         goal_position.point.x = -self.change_in_position
#
#         self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
#                                                          tip_link=self.source,
#                                                          root_link=self.dest)
#
#     def move_towards(self):
#         self.giskard.motion_goals.add_cartesian_pose(goal_pose=lookup_pose(target_frame="map", source_frame=self.dest),
#                                                      tip_link=self.source,
#                                                      root_link='map')
#
#     def rotate_towards(self, frame_id):
#         pass
#
#     def reasoner_callback(self, msg):
#         if msg.data:
#             print(f"data: {msg.data}")
#             command = set(msg.data.split(":"))
#             # move_commands = set(command.values())
#             print(type(command))
#             self.destination_move(command, [])
#
#     def destination_move(self, commands: set, rotation_dir: np.array):
#         # self.source = commands.pop().split('(')[1].strip(')')
#         # ToDo: What happens if you receive a goal before you reach the current goal?
#         # current_pose = lookup_transform(self.dest, self.source)
#         for command in commands:
#
#             if "decreaseTilting" in command:
#                 # self.tilt(current_pose, "decrease", rotation_dir)
#                 pass
#             elif "increaseTilting" in command:
#                 # self.tilt(current_pose, "increase", rotation_dir)
#                 pass
#             elif "moveTowards" in command:
#                 self.move_towards()
#
#             elif "moveUp" in command:
#                 self.move_up()
#
#             elif "moveDown" in command:
#                 self.move_down()
#
#             elif "moveRight" in command:
#                 self.move_right()
#
#             elif "moveLeft" in command:
#                 self.move_left()
#
#             elif "moveForward" in command:
#                 self.move_forward()
#
#             elif "moveBack" in command:
#                 self.move_back()
#
#             elif "rotateTowards" in command:
#                 self.rotate_towards(frame_id=self.dest)
#
#         result: MoveResult = self.giskard.execute()
#
#
# if __name__ == "__main__":
#     rospy.init_node('giskard_listener')
#     listener = GiskardListener()
#     listener.adaptive_tilt()
#
#     # rate = rospy.Rate(10)  # 10hz
#     # while not rospy.is_shutdown():
#     #     rate.sleep()
#
# # rospy.loginfo('Set a Cartesian goal for the box')
#
# # self.destination_move("increaseTilting")
