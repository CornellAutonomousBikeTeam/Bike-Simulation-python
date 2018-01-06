# constants.py
import math

######### mainNavigation constants

# properties of the p struct in matlab
G = 9.81
L = 1.02
B = 0.3
H = 0.9
# trail is zero
C = 0 

# gains
K1 = 71.
K2 = 21.
K3 = -20.

# time
TIMESTEP = 1.0/100
PAUSE = TIMESTEP

# used in converting output of nav to steerD
MAX_STEER = math.pi/6.0

## Nav algo constants

# PID with cutting
MAX_ACCEPTABLE_ANGLE_DIFF = math.pi / 4.0
PID_DIST_GAIN = 1.0
PID_ANGLE_GAIN = 2.0
RAD_TO_DEG = 180.0 / math.pi
TURN_LOOKAHEAD_DIST = 7
NEXT_TURN_GAIN = 1

# Quintic
BIKE_LENGTH = 1
QUINTIC_LOOKAHEAD = 6
QUINTIC_SAMPLE_LENGTH = 2 # The quintic gives us a steering angle for any distance along the path

# Lookahead
BASE_LOOKAHEAD = 3
LOOKAHEAD_ANGLE_GAIN = 4
MIN_TURN_RADIUS = 7.0 # determined with the simulation by setting steering angle to MAX_STEER

## Visualization constants
ANIM_INTERVAL = 0 # milliseconds between blit frames
