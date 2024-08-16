import numpy as np
import rospy

from geometry_msgs.msg import TransformStamped, QuaternionStamped, PointStamped, PoseStamped, Vector3Stamped
from std_msgs.msg import String

from giskardpy.python_interface.python_interface import GiskardWrapper
from giskardpy.goals.adaptive_goals import PouringAdaptiveTilt
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

class GiskardListener(object):

    def __init__(self):
        # rospy.Subscriber('/reasoner/concluded_behaviors', String, self.reasoner_callback)

        self.change_in_orientation = (5 / 180) * 3.14  # 5 degrees
        self.change_in_position = 0.05  # 5 cm
        self.max_rotation = np.deg2rad(135)

        self.giskard = GiskardWrapper()
        self.source = 'free_cup'
        self.dest = 'free_cup2'
        self.root = 'map'

    def adaptive_tilt(self):

        p = PoseStamped()

        p.header.frame_id = 'free_cup2'
        p = lookup_pose(self.root, self.dest)
        p.pose.position.z = p.pose.position.z + 0.05

        # goal_pose.pose.position.x = 1.95
        # goal_pose.pose.position.y = -0.4
        # goal_pose.pose.position.z = 0.49
        # goal_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
        #                                                                  [0, -1, 0, 0],
        #                                                                  [1, 0, 0, 0],
        #                                                                  [0, 0, 0, 1]]))
        tilt_axis = Vector3Stamped()
        tilt_axis.header.frame_id = 'free_cup'
        tilt_axis.vector.x = 1

        self.giskard.motion_goals.add_motion_goal(goal=PouringAdaptiveTilt(),
                                                  name='pouring',
                                                  tip='free_cup',
                                                  root='map',
                                                  tilt_angle=1,
                                                  pouring_pose=p,
                                                  tilt_axis=tilt_axis)
        self.giskard.execute()

    def tilt(self, pose, change: str, rotation_dir: np.array = np.array([])):
        print(f"self.dest pose: {pose.transform.translation, pose.transform.rotation}")
        # + 5 degrees along the rotation dir
        euler_angles_radians = list(euler_from_quaternion([pose.transform.rotation.x, pose.transform.rotation.y,
                                                           pose.transform.rotation.z, pose.transform.rotation.w]))
        a_max = 0
        a_min = 0
        goal_rotation = QuaternionStamped()
        angle_update_radians = np.zeros(3)

        if rotation_dir.size == 0:
            if change == "decrease":  # until 0 degrees
                if euler_angles_radians[0] > euler_angles_radians[1]:
                    a_max = euler_angles_radians[0]
                    angle_update_radians[0] = -self.change_in_orientation
                elif euler_angles_radians[0] == euler_angles_radians[1]:
                    a_max = euler_angles_radians[0]
                    angle_update_radians[0] = -self.change_in_orientation
                else:
                    a_max = euler_angles_radians[1]
                    angle_update_radians[1] = -self.change_in_orientation
                print(f"before clipping: {euler_angles_radians[0], a_max, a_min}")
                euler_angles_radians[0:2] = np.clip(euler_angles_radians[0:2], a_max=self.max_rotation, a_min=a_min)

            elif change == "increase":  # until 135 degrees
                if euler_angles_radians[0] > euler_angles_radians[1]:
                    a_min = euler_angles_radians[0]
                    angle_update_radians[0] = self.change_in_orientation
                elif euler_angles_radians[0] == euler_angles_radians[1]:
                    a_min = euler_angles_radians[0]
                    angle_update_radians[0] = self.change_in_orientation
                else:
                    a_min = euler_angles_radians[1]
                    angle_update_radians[1] = self.change_in_orientation

                euler_angles_radians[0:2] = np.clip(euler_angles_radians[0:2], a_min=a_min, a_max=self.max_rotation)

            goal_rotation.header.frame_id = self.source
            quat = quaternion_from_euler(angle_update_radians[0], angle_update_radians[1],
                                         angle_update_radians[2], 'rxyz')
            print(f'quat: {quat}')
            goal_rotation.quaternion.x = quat[0]
            goal_rotation.quaternion.y = quat[1]
            goal_rotation.quaternion.z = quat[2]
            goal_rotation.quaternion.w = quat[3]

        self.giskard.motion_goals.add_cartesian_orientation(goal_orientation=goal_rotation,
                                                            tip_link=self.source,
                                                            root_link=self.dest)

    def move_up(self):
        goal_position = PointStamped()
        goal_position.header.frame_id = self.source
        goal_position.point.z = self.change_in_position

        self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
                                                         tip_link=self.source,
                                                         root_link=self.dest)

    def move_down(self):
        goal_position = PointStamped()
        goal_position.header.frame_id = self.source
        goal_position.point.z = -self.change_in_position

        self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
                                                         tip_link=self.source,
                                                         root_link=self.dest)

    def move_right(self):
        goal_position = PointStamped()
        goal_position.header.frame_id = self.source
        goal_position.point.y = -self.change_in_position

        self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
                                                         tip_link=self.source,
                                                         root_link=self.dest)

    def move_left(self):
        goal_position = PointStamped()
        goal_position.header.frame_id = self.source
        goal_position.point.y = self.change_in_position

        self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
                                                         tip_link=self.source,
                                                         root_link=self.dest)

    def move_forward(self):
        goal_position = PointStamped()
        goal_position.header.frame_id = self.source
        goal_position.point.x = self.change_in_position

        self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
                                                         tip_link=self.source,
                                                         root_link=self.dest)

    def move_back(self):
        goal_position = PointStamped()
        goal_position.header.frame_id = self.source
        goal_position.point.x = -self.change_in_position

        self.giskard.motion_goals.add_cartesian_position(goal_point=goal_position,
                                                         tip_link=self.source,
                                                         root_link=self.dest)

    def move_towards(self):
        self.giskard.motion_goals.add_cartesian_pose(goal_pose=lookup_pose(target_frame="map", source_frame=self.dest),
                                                     tip_link=self.source,
                                                     root_link='map')

    def rotate_towards(self, frame_id):
        pass

    def reasoner_callback(self, msg):
        if msg.data:
            print(f"data: {msg.data}")
            command = set(msg.data.split(":"))
            # move_commands = set(command.values())
            print(type(command))
            self.destination_move(command, [])

    def destination_move(self, commands: set, rotation_dir: np.array):
        # self.source = commands.pop().split('(')[1].strip(')')
        # ToDo: What happens if you receive a goal before you reach the current goal?
        # current_pose = lookup_transform(self.dest, self.source)
        for command in commands:

            if "decreaseTilting" in command:
                # self.tilt(current_pose, "decrease", rotation_dir)
                pass
            elif "increaseTilting" in command:
                # self.tilt(current_pose, "increase", rotation_dir)
                pass
            elif "moveTowards" in command:
                self.move_towards()

            elif "moveUp" in command:
                self.move_up()

            elif "moveDown" in command:
                self.move_down()

            elif "moveRight" in command:
                self.move_right()

            elif "moveLeft" in command:
                self.move_left()

            elif "moveForward" in command:
                self.move_forward()

            elif "moveBack" in command:
                self.move_back()

            elif "rotateTowards" in command:
                self.rotate_towards(frame_id=self.dest)

        result: MoveResult = self.giskard.execute()


if __name__ == "__main__":
    rospy.init_node('giskard_listener')
    listener = GiskardListener()
    listener.adaptive_tilt()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

# rospy.loginfo('Set a Cartesian goal for the box')

# self.destination_move("increaseTilting")
