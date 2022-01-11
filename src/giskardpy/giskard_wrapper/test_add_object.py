import rospy

from giskard_msgs.srv import UpdateWorld
from geometry_msgs.msg import PoseStamped
from giskard_msgs.msg import WorldBody


# Args: operation body rigidly_attached pose
def add_object():
    rospy.init_node("add_object_client")
    rospy.wait_for_service('/giskard/update_world')
    try:
        add_object_client = rospy.ServiceProxy('/giskard/update_world', UpdateWorld)
        operation = 1
        body = WorldBody()
        body.type = 1
        body.name = "cuboid_cup"
        body.shape.type = 1
        body.shape.dimensions = [0.1, 0.1, 0.2]
        #body.shape.dimensions[1] = 1.8
        #body.shape.dimensions[2] = 1.0

        rigidly_attached = False
        pose_w_object = PoseStamped()
        pose_w_object.header.stamp = rospy.Time.now()
        pose_w_object.header.frame_id = "map"

        pose_w_object.pose.position.x = 1.5
        pose_w_object.pose.position.y = 1.5
        pose_w_object.pose.position.z = 1.5

        pose_w_object.pose.orientation.x = 0
        pose_w_object.pose.orientation.y = 0
        pose_w_object.pose.orientation.z = 0
        pose_w_object.pose.orientation.w = 1

        response = add_object_client(operation, body, rigidly_attached, pose_w_object)
        print("response : ", response.error_codes)
    except rospy.ServiceException as e:
        print("Service call failed %s", e)


if __name__ == "__main__":
    add_object()
