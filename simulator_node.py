#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import time
import random 
import bikeState
import bikeSim
import mapModel
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension


def update_graph(data):
    new_bike.update(bikeSim.new_state(new_bike, data.data))
    
def path_parse(data):
    d = np.array(data.data).reshape(len(data.data)/4, 2, 2)
    map_model.paths = d

def listener():
    pub = rospy.Publisher('bike_state', Float32MultiArray, queue_size=10)
    rospy.init_node('simulator', anonymous=True)
    rospy.Subscriber("nav_instr", Int32, update_graph)
    rospy.Subscriber("paths", Int32MultiArray, path_parse)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        dim = [MultiArrayDimension('data', 8, 8)]
        layout = MultiArrayLayout(dim, 0)
        # This needs to publish the bike state from the Arduino 
        l = [new_bike.xB, new_bike.yB, new_bike.phi, new_bike.psi, new_bike.delta, new_bike.w_r, new_bike.v, new_bike.turning_r]
        rospy.loginfo(l)
        pub.publish(layout, l)
        rate.sleep()
    

if __name__ == '__main__':
    new_bike = bikeState.Bike(0, -10, 0.1, np.pi/3, 0, 0, 3.57)
    waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
    map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
    listener()
