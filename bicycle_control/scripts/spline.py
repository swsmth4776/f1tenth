#!/usr/bin/env python
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from bicycle_control.msg import MotionPrimitive

def talker(graph=False):
    # Define some points:
    points = np.array([[10,9.8,9,6,2,0],
                        [0,2,6,9,9.8,10]]).T  # a (nbre_points x nbre_dim) array

    # Linear length along the line:
    distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
    distance = np.insert(distance, 0, 0)/distance[-1]

    # Interpolation for different methods:
    interpolations_methods = ['slinear', 'quadratic', 'cubic']
    alpha = np.linspace(0, 1, 75)

    interpolated_points = {}
    for method in interpolations_methods:
        interpolator =  interp1d(distance, points, kind=method, axis=0)
        interpolated_points[method] = interpolator(alpha)

    if graph:
        # Graph:
        plt.figure(figsize=(7,7))
        for method_name, curve in interpolated_points.items():
            plt.plot(curve[:,0], curve[:,1], '-', label=method_name);
            plt.plot(points[:,0], points[:,1], 'ok', label='original points');
            plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')
        plt.show()

    data = interpolated_points['cubic']
    # Goal: write data through a topic
    pub = rospy.Publisher('motion_primitive', MotionPrimitive, queue_size=10)
    rospy.init_node('planner', anonymous=True)
    rate = rospy.Rate(10) # 10Hz
    msg = MotionPrimitive()
    
    # 20 time steps = 2 seconds
    # v = t*a + v_initial
    # so if a = 6, v_init = 0, then v = 12 m/s = 26.8 mph
    # at the end of the turn the car will be traveling at 26.8 mph
    a = 6 #m/s^2

    msg.x
    msg.y
    msg.v
    msg.yaw

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
