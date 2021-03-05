#!/usr/bin/env python
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from bicycle_control.msg import MotionPrimitive
import pdb
import sys

first_plot = True 

def plot_trajectory(data):
    global first_plot
    if not first_plot:
        return
    first_plot = False


    rospy.loginfo(rospy.get_caller_id() + "I heard a message")
    print(type(data))
    x = np.array(data.x)
    y = np.array(data.y)
    v = np.array(data.v)
    yaw = np.array(data.yaw)
    points = np.stack((x,y))
    points = points.T
    plt.scatter(x, y)
    plt.show()

def listener(graph=False):
    rospy.init_node('trajectory_plotter', anonymous=True)
    rospy.Subscriber("motion_primitive", MotionPrimitive, plot_trajectory)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
