import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Point


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [] , []
        self.x = 0
        self.y = 0

    def plot_init(self):
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(0, 1)
        self.ax.set_aspect('equal', 'box')
        return self.ln

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw   

    def odom_callback(self, msg):
        #yaw_angle = self.getYaw(msg.pose.pose)
        self.y_data.append(msg.force.y)
        #x_index = len(self.x_data)
        self.x_data.append(msg.force.x)
        #rospy.sleep(0.005)

    def force_callback(self, msg):
        ok = True
        if msg.force.y > 0.1:
            for i in range(len(self.x_data)):
                if pow(self.x - self.x_data[i], 2) + pow(self.y - self.y_data[i], 2) < 0.00001:
                    ok = False

            if ok:
                self.y_data.append(self.y)
                #x_index = len(self.x_data)
                self.x_data.append(self.x)
                rospy.loginfo("Point Added!")

    def pos_callback(self, msg):
        self.x = msg.y
        self.y = msg.z

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln


rospy.init_node("plotter")
#rospy.init_node('lidar_visual_node')
vis = Visualiser()
#sub = rospy.Subscriber('/dji_sdk/odometry', Odometry, vis.odom_callback)

pose_sub = rospy.Subscriber("/elfin/cvc/ee_point", Point, callback=vis.pos_callback) 
force_sub = rospy.Subscriber("/elfin/cvc/sensor_filtered", Wrench, callback=vis.force_callback) 
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)


