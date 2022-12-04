
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
    pose.position.y = -0.4
    pose.position.z = 0.7

    quaternion = tf.transformations.quaternion_from_euler(0, 1.571, 1.571)


    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
    model_name='aruco_marker',
        model_xml=open('/home/teemu/.gazebo/models/aruco_visual_marker_0/model.sdf', 'r').read(),
        robot_namespace='/foo',
        initial_pose=pose,
        reference_frame='world'
    )

    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    #rospy.init_node('tag_mover', anonymous=True)
    rate = rospy.Rate(1000) # 10hz
    msg = ModelState()
    msg.pose = pose
    while not rospy.is_shutdown():
        roll = 0
        pitch = 1.571
        yaw = 1.571
        msg.model_name = "aruco_marker"
        if msg.pose.position.x < 0.25:
            msg.pose.position.x += 0.00005 
        
        elif msg.pose.position.z > 0.4:
            msg.pose.position.z -= 0.00005
            msg.pose.position.y += 0.000005
            yaw += 0.5
            pitch -= 0.2
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]

        else:
            break
        
        #elif msg.pose.position.y < -0.2:
        #     msg.pose.position.y += 0.0001
        #     msg.pose.position.z += 0.0001

        pub.publish(msg)
        rate.sleep()

    while not rospy.is_shutdown():
        roll = 0
        pitch = 1.571
        yaw = 1.571
        msg.model_name = "aruco_marker"
        if msg.pose.position.x > -0.10:
            msg.pose.position.x -= 0.00005 
            msg.pose.position.z += 0.000005
            msg.pose.position.y -= 0.000005
        
        #elif msg.pose.position.z > 0.4:
        #    msg.pose.position.z -= 0.0001
        #    msg.pose.position.y += 0.00005
        #    yaw += 0.5
        #    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        #    msg.pose.orientation.x = quaternion[0]
        #    msg.pose.orientation.y = quaternion[1]
        #    msg.pose.orientation.z = quaternion[2]
        #    msg.pose.orientation.w = quaternion[3]

        else:
            break
        
        #elif msg.pose.position.y < -0.2:
        #     msg.pose.position.y += 0.0001
        #     msg.pose.position.z += 0.0001

        pub.publish(msg)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

