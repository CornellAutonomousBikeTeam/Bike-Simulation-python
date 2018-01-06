#!/usr/bin/env python
"""
This file is simply simulator_node.py, but it reads from the GPS ROS stream
and updates its internal state based on that.
"""
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import time
import random 
import bikeState
import bikeSim
import mapModel
import requestHandler
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

# This variable stores the old set of GPS data
old_gps_set = ()
def update_bike_from_gps(data):
    """Takes the incoming data from the GPS and updates our state with it."""

    # Only update the state if the incoming data is different from the last set
    # of GPS data we received

    # These are the four variables we use below
    global old_gps_set
    curr_gps_set = (data.data[0], data.data[1], data.data[7], data.data[8])
    if curr_gps_set == old_gps_set:
        rospy.loginfo("Rejecting data from gps")
        return
    old_gps_set = curr_gps_set

    latitude = data.data[0] # In degrees
    longitude = data.data[1]
    psi = data.data[7] # psi = heading in radians
    velocity = data.data[8]

    xy_point = requestHandler.math_convert(latitude, longitude)

    new_bike.psi = psi
    new_bike.v = velocity
    new_bike.xB = xy_point[0]
    new_bike.yB = xy_point[1]
    rospy.loginfo("Incoming data from GPS: ({:.4f}, {:.4f}), heading {:.2f} deg, velocity {:.2f} m/s".format(new_bike.xB, new_bike.yB, new_bike.psi * 180 / np.pi, new_bike.v))
    rospy.loginfo("INCOMING DATA")

def update_graph(data):
    new_bike.update(bikeSim.new_state(new_bike, data.data))
    
def path_parse(data):
    d = np.array(data.data).reshape(len(data.data)/4, 2, 2)
    map_model.paths = d

def listener():
    pub = rospy.Publisher('bike_state', Float32MultiArray, queue_size=10)
    rospy.init_node('simulator', anonymous=True)
    rospy.Subscriber("nav_instr", Float32, update_graph)
    rospy.Subscriber("paths", Float32MultiArray, path_parse)
    rospy.Subscriber("gps", Float32MultiArray, update_bike_from_gps)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        dim = [MultiArrayDimension('data', 8, 8)]
        layout = MultiArrayLayout(dim, 0)
        # This needs to publish the bike state from the Arduino 
        l = [new_bike.xB, new_bike.yB, new_bike.phi, new_bike.psi, new_bike.delta, new_bike.w_r, new_bike.v, new_bike.turning_r]
        rospy.loginfo("({:.4f}, {:.4f}), heading {:.4f}, velocity {:.4f}, lean angle/rate {:.4f}/{:.4f}, steering {:.4f} [gps_assisted_simulator_node]".format(new_bike.xB, new_bike.yB, new_bike.psi, new_bike.v, new_bike.phi, new_bike.w_r, new_bike.delta))
        rospy.loginfo(l)
        pub.publish(layout, l)
        rate.sleep()
    

if __name__ == '__main__':
    new_bike = bikeState.Bike(0, -10, 0.1, np.pi/3, 0, 0, 3.57)
    waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
    map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
    listener()
