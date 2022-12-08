#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import random
import numpy as np
import PyKDL

if __name__ == '__main__':
    rospy.init_node("line_planner")
    rospy.loginfo("line_planner node started!")

    des_pub = rospy.Publisher("/planner/des_frame", Float64MultiArray, queue_size=10)
    rate = rospy.Rate(5)
#    while not rospy.is_shutdown():

    msg_des = Float64MultiArray()
    msg_des.data.clear()
    msg_des.data.append(0)
    msg_des.data.append(-0.32)
    msg_des.data.append(0.56)
    msg_des.data.append(0 * np.pi /180)
    msg_des.data.append(0)
    msg_des.data.append(-90 * np.pi/180)
    for x in range(12):
        msg_des.data.append(0)
    des_pub.publish(msg_des)
    rospy.sleep(1)
    des_pub.publish(msg_des)
    rospy.sleep(5)

    pa = PyKDL.Vector(0,-0.32,0.56)
    pb = PyKDL.Vector(0,0,0.56)
    rot_a = PyKDL.Rotation()
    rot_a.DoRotZ(-90*np.pi/180)
    rot_b = PyKDL.Rotation()
    rot_b.DoRotZ(-90*np.pi/180)
    a = PyKDL.Frame(rot_a, pa)
    b = PyKDL.Frame(rot_b, pb)
    c = PyKDL.diff(a,b, 20)
    d = a
    for i in range(200):
        msg_des.data.clear()
        (rr, pp, yy) = d.M.GetRPY()
        x = d.p.x()
        y = d.p.y()
        z = d.p.z()
        msg_des.data.append(x)
        msg_des.data.append(y)
        msg_des.data.append(z)
        msg_des.data.append(rr)
        msg_des.data.append(pp)
        msg_des.data.append(yy)
        msg_des.data.append(c.vel.x())
        msg_des.data.append(c.vel.y())
        msg_des.data.append(c.vel.z())
        msg_des.data.append(c.rot.x())
        msg_des.data.append(c.rot.y())
        msg_des.data.append(c.rot.z())
        for x in range(6):
            msg_des.data.append(0)
        des_pub.publish(msg_des)
        d = PyKDL.addDelta(d, c, 0.1)
        rospy.sleep(0.1)
    msg_des.data.clear()
    (rr, pp, yy) = d.M.GetRPY()
    x = d.p.x()
    y = d.p.y()
    z = d.p.z()
    msg_des.data.append(x)
    msg_des.data.append(y)
    msg_des.data.append(z)
    msg_des.data.append(rr)
    msg_des.data.append(pp)
    msg_des.data.append(yy)
    for x in range(12):
        msg_des.data.append(0)
    d = PyKDL.addDelta(d, c, 0.1)
    des_pub.publish(msg_des)
    #rate.sleep()