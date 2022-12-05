#!/usr/bin/env python
from __future__ import print_function
# license removed for brevity
import kdl_parser_py.urdf as kdl_parser
import PyKDL as kdl
import rospy
import numpy as np
import speech_recognition as sr
from trajectory_planners.srv import tag_init, tag_initRequest, tag_initResponse
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from time import sleep

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl

class SpeechCommandServer():
    def __init__(self):
        self.pose_subs = rospy.Subscriber("elfin/cvc/q", Float64MultiArray, self.pose_callback)
        self.pose_pub = rospy.Publisher('target_pose', TransformStamped, queue_size=10)
        (status, tree) = kdl_parser.treeFromParam("robot_description")
        print("\n *** Successfully parsed urdf file and constructed kdl tree *** \n" if status else "Failed to parse urdf file to kdl tree")
        chain = tree.getChain("world", "elfin_link6")
        num_joints = chain.getNrOfJoints()
        print("\n*** This robot has %s joints *** \n" % num_joints)
        self.fk_pos_solver = kdl.ChainFkSolverPos_recursive(chain)
        print(" \n *** solver has been created *** \n")
        self.frame = kdl.Frame()
        self.x = 0
        self.y = 0 
        self.z = 0
        self.w = 1
        self.steps_size = 0.05


    def send_command(self):
        msg = TransformStamped()
        msg.transform.translation.x = self.frame.p[0]
        msg.transform.translation.y = self.frame.p[1]
        msg.transform.translation.z = self.frame.p[2]
        #print(self.frame.M)
        msg.transform.rotation.x = self.x
        msg.transform.rotation.y = self.y
        msg.transform.rotation.z = self.z
        msg.transform.rotation.w = self.w
        print("sending command")
        self.pose_pub.publish(msg)

    def tag_following_on(self, following_status):
        rospy.wait_for_service('init_tag_following')
        if following_status:
            print("initing tag following")

        else:
            print("turn off tag following")
        try:
            tag_init_service= rospy.ServiceProxy('init_tag_following', tag_init)
            req = tag_initRequest()
            req.is_tag_following = following_status
            resp1 = tag_init_service(req)
            return resp1.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def pose_callback(self, msg):
        "parse the joint states to end-effector pos"
        q = []
        for i in range(len(msg.data)):
            q.append(msg.data[i])

        theta = joint_list_to_kdl(q)
        self.fk_pos_solver.JntToCart(theta, self.frame)

    def change_tag_id(self, id):
        print("setting the marker id to: {}".format(id))
        rospy.set_param("aruco_single/marker_id", id)
        rospy.set_param("/aruco_single_world/marker_id", id)

    def command_parser(self, command):
        print(command.split(" ")[0])
        if command == "move down": 
            self.frame.p[2] -= self.steps_size
            self.send_command()

        elif command == "move up":
            self.frame.p[2] += self.steps_size
            self.send_command()

        elif command == "move left":
            self.frame.p[0] += self.steps_size
            self.send_command()

        elif command == "move right":
            self.frame.p[0] -= self.steps_size
            self.send_command()

        elif command == "move forward":
            self.frame.p[1] -= self.steps_size
            self.send_command()

        elif command == "move back":
            self.frame.p[1] += self.steps_size
            self.send_command()
            
        elif command == "down":
            self.frame.M.RotX(1.57)
            self.send_command()

        elif command == "look up":
            self.frame.M.RotX(-1.57)
            self.send_command()
            
        elif command == "rotate left":
            self.frame.M.DoRotZ(-1.57)
            self.send_command()

        elif command == "rotate right":
            self.frame.M.DoRotZ(1.57)
            self.send_command()

        elif command == "rotate down":
            self.frame.M.DoRotY(-1.57)
            self.send_command()

        elif command == "rotate up":
            self.frame.M.DoRotY(1.57)
            self.send_command()

        elif command == "Step Up":
            self.steps_size += 0.05
            print("step size is {}".format(self.steps_size))
            
        elif command == "step down":
            self.steps_size -= 0.05
            print("step size is {}".format(self.steps_size))
        
        elif command == "back on":
            self.tag_following_on(True)
        
        elif command == "back off":
            self.tag_following_on(False)

        elif command.split(" ")[0] == "marker":
            self.change_tag_id(int(command.split(" ")[1]))

        
            

if __name__ == '__main__':
    rospy.init_node('speech_command_node', anonymous=True)
    r = sr.Recognizer()
    mic = sr.Microphone()
    command = ""
    sp_server = SpeechCommandServer()
    while not rospy.is_shutdown():
        sleep(1)
        with mic as source:
            r.adjust_for_ambient_noise(source)
            try: 
                print("state your command to robot")
                audio = r.listen(source, 1, 5)
                print("trying to make up command")
                command = r.recognize_google(audio)
                print(command)
                sp_server.command_parser(command)
            except Exception as e:
                print(e)

