#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import time
import random 
import matplotlib
import matplotlib.pyplot as plt
import nav
import numpy as np
import bikeState
import mapModel
from matplotlib import collections  as mc
from std_msgs.msg import Int32MultiArray


def update_graph(data):
    d = data.data
    new_bike.xB = d[0]
    new_bike.yB = d[1]
    
def path_parse(data):
    d = np.array(data.data).reshape(len(data.data)/4, 2, 2)
    map_model.paths = d
    paths.unregister()
    lc = mc.LineCollection(map_model.paths, linewidths=2,
        color = "black")
    ax.add_collection(lc)
    plt.plot(d[:,:,0], d[:,:,1], 'ro')

def listener():
    rospy.init_node('sim', anonymous=True)
    rospy.Subscriber("bike_state", Float32MultiArray, update_graph)
    global paths
    paths = rospy.Subscriber("paths", Int32MultiArray, path_parse)
    rate = rospy.Rate(100)
    rospy.loginfo(rospy.is_shutdown())
    while not rospy.is_shutdown():
        plt.scatter(new_bike.xB, new_bike.yB)
        plt.show()
        plt.pause(0.000000001)
    
if __name__ == '__main__':
    new_bike = bikeState.Bike(0, -10, 0.1, np.pi/3, 0, 0, 3.57)
    waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
    map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
    plt.ion() # enables interactive plotting
    paths = map_model.paths
    fig = plt.figure()
    fig.set_dpi(100) #dots per inch
    ax = plt.axes(xlim=(0, 40), ylim=(-20, 20)) 
    lc = mc.LineCollection(paths, linewidths=2, color = "black")
    ax.add_collection(lc)
    plt.show() 
    listener()

