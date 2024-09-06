import rospy
from geometry_msgs.msg import PointStamped

from giskardpy.python_interface.python_interface import GiskardWrapper
from giskardpy.god_map import god_map
from giskardpy.utils.tfwrapper import lookup_transform, lookup_pose, lookup_point
from giskard_msgs.msg import MoveResult

try:
    # This case works in ros1 environments
    from knowrob.kb import *
except ImportError:
    # If the import fails, import the knowrob.so directly
    from knowrob import *


class GiskardReasoner(RDFGoalReasoner):
    def __init__(self):
        super(GiskardReasoner, self).__init__()
        rospy.init_node("giskard_reasoner")
        self.giskard = GiskardWrapper()
        self.reachable = IRIAtom("http://knowrob.org/kb/knowrob.owl#reachability")
        self.robot_part = IRIAtom("http://knowrob.org/kb/knowrob.owl#l_gripper_tool_frame")
        self.object_part = IRIAtom("http://knowrob.org/kb/knowrob.owl#iai_fridge_door_handle")
        self.defineRelation(self.reachable)

    def initializeReasoner(self, config: PropertyTree):
        return True

    def evaluate(self, goal: RDFGoal):
        literal = goal.rdfLiterals()[0]
        s: Term = literal.subjectTerm()
        p = literal.propertyTerm()
        o: Term = literal.objectTerm()
        result: bool = False
        if "reachability" in str(p).split(":")[-1].strip("'"):
            rospy.loginfo("Checking reachability")
            result = self.reachability(str(s).strip("'"), str(o).strip("'"))

        if result:
            key = Variable("Reachable")
            value = Atom.Tabled("True")
            bindings = Bindings({key : value})
            goal.push(bindings)

        return True


    def reset_base(self):
        root_link = "map"
        base_position = PointStamped()
        base_position.header.frame_id = root_link
        base_position.point.x = -0.5

        self.giskard.motion_goals.add_cartesian_position(goal_point=base_position,
                                                         tip_link="base_footprint",
                                                         root_link=root_link,
                                                         weight=1.0,
                                                         name="reset")
        self.giskard.add_default_end_motion_conditions()
        self.giskard.execute()

    # reachable("", "")
    def reachability(self, object_link: str, end_effector_link: str, root_link: str = "map"):
        # root_link is the expressed in frame
        # end_effector_link = self.giskard.world.search_for_link_name(end_effector_link)
        # object_link = object_link.split("#")[-1]
        # end_effector_link = end_effector_link.split("#")[-1]

        # self.reset_base()
        object_point = lookup_point(root_link, object_link)

        self.giskard.motion_goals.add_cartesian_position(goal_point=object_point,
                                                         tip_link=end_effector_link,
                                                         root_link=root_link,
                                                         weight=1.0,
                                                         name="reachability")

        self.giskard.add_default_end_motion_conditions()

        result: MoveResult = self.giskard.projection()

        if result.error.code == 0:
            rospy.loginfo("Projection successful.")
            return True
        elif result.error.code == 102:
            rospy.loginfo("out_of_joint_limits")
            return False
        else:
            rospy.loginfo("Projection failed.")
            return False

# if __name__ == "__main__":
#     reasoner = GiskardReasoner()
#     reasoner.reachability("iai_fridge_door_handle", "l_gripper_tool_frame")
#
#     rate = rospy.Rate(10)  # 10hz
#     while not rospy.is_shutdown():
#         rate.sleep()
