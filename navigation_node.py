#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import nav
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import bikeState
import mapModel
import requestHandler


def callback(data):
    new_nav.map_model.bike.xy_coord = (data.x, data.y)
    new_nav.map_model.bike.direction = data.theta

def path_parse(data):
    d = np.array(data.data).reshape(len(data.data)/4, 2, 2)
    new_map.paths = d

def update_bike_state(data):
    d = data.data
    #new_bike.xB = d[0]
    #new_bike.yB = d[1]
    new_bike.phi = d[5]
    new_bike.delta = d[2]
    new_bike.w_r = d[4]
    # new_bike.v = d[] #gps or 6
    # new_bike.turning_r = d[7] LOOKEDUP

# This variable stores the old set of GPS data
old_gps_set = ()
def update_bike_xy(data):
    """Takes the incoming data from the GPS and updates our state with it."""

    # Only update the state if the incoming data is different from the last set
    # of GPS data we received

    # These are the four variables we use below
    curr_gps_set = (data.data[0], data.data[1], data.data[7], data.data[8])
    if curr_gps_set == old_gps_set:
        rospy.loginfo("Rejecting data from gps")
        return
    old_gps_set = curr_gps_set

#    if data.data[0] == 0 and data.data[1] == 0:
#       not_ready = True
#    else:
    if (True):
        not_ready = False
        latitude = data.data[0] # In degrees
        longitude = data.data[1]
        psi = data.data[7] # psi = heading in radians
        velocity = data.data[8]

    #if data.data[0] == 0 and data.data[1] == 0:
     #   not_ready = True
    #else:
        #time_since_last = data.data[5]
        #EPSILON = 25
        #if time_since_last > EPSILON:
            #not_ready = False
            #lat = data.data[0] # In degrees 
            #lon = data.data[1]
            #psi = data.data[7] # This needs to be in rads
            #velocity = data.data[8]
        xy_point = requestHandler.math_convert(latitude, longitude)


            #d_psi = float(psi - old_psi)/old_time_since_last
        new_bike.psi = psi
        new_bike.v = velocity
            #Update old velocity and angle for extrapolation
            #old_v = velocity
            #old_psi = psi
        new_bike.xB = xy_point[0]
        new_bike.yB = xy_point[1]
        #else:
         #   new_psi = d_psi*time_since_last + old_psi
          #  new_x = velocity*cos(new_psi)
           # new_y = velocity*sin(new_psi)
        #old_time_since_last = data.data[5]


def keyboard_update(data):
    #rospy.loginfo(data) 
    x = data.linear.x
    z = data.angular.z
    v = .5
    if x>0 and z == 0: #up
        new_bike.yB += v 
    if x<0 and z == 0: #down
        new_bike.yB -= v 
    if z>0 and x == 0: #left
        new_bike.xB -=v        
    if z<0 and x == 0: #right
        new_bike.xB +=v
    if x>0 and z > 0: #ccw
        psi = new_bike.psi
        new_bike.psi = (psi + np.pi/12)%(2*np.pi) 
    if x>0 and z < 0: #cw
        psi = new_bike.psi
        new_bike.psi = (psi + np.pi/12)%(2*np.pi)

def talker():
    pub = rospy.Publisher('nav_instr', Float32, queue_size=10)
    rospy.init_node('navigation', anonymous=True)
    # Subscribe to topic "bike_state" to get data and then call update_bike_state with data
    rospy.Subscriber("cmd_vel", Twist, keyboard_update)
    rospy.Subscriber("bike_state", Float32MultiArray, update_bike_state) 
    rospy.Subscriber("gps", Float32MultiArray, update_bike_xy)
    rospy.Subscriber("paths", Float32MultiArray, path_parse) 
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        new_map = new_nav.map_model
        #if not_ready:
         #   pub.publish(3)
        #else:
        if True:
            #rospy.loginfo((new_bike.xB, new_bike.yB, new_bike.psi, new_nav.direction_to_turn()))
            pub.publish(new_nav.get_steering_angle())
        rate.sleep()

if __name__ == '__main__':
    try:
        old_psi = 0
        old_v = 0
        d_psi = 0
        not_ready = True
        new_bike = bikeState.Bike(0, -10, 0.1, np.pi/3, 0, 0, 3.57)
        waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
        new_map = mapModel.Map_Model(new_bike, waypoints, [], [])
        new_nav = nav.Nav(new_map)
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('here')
        pass
