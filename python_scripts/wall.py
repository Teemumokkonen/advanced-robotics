
from re import X
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
import rospy
from geometry_msgs.msg import Pose
rospy.init_node('insert_object',log_level=rospy.INFO)
import tf
import rospy
from std_msgs.msg import String
from math import*

def talker():

    # spawn tag to world
    pose = Pose()

    pose.position.x = 0.0
    pose.position.y = -1.2
    pose.position.z = 0.5

    quaternion = tf.transformations.quaternion_from_euler(0, 1.571, 1.571)


    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
    model_name='obs_wall',
        model_xml=open('models/wall.sdf', 'r').read(),
        robot_namespace='/foo',
        initial_pose=pose,
        reference_frame='world'
    )

    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

