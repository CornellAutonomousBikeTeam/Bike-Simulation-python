from constants import *
import bikeState

def new_state(bike, steerD):
	""" Returns new bike state object after it passes through bike dynamics """
	# Get navigation command
	# steerD = nav.command(bike.xB, bike.yB, bike.psi, bike.v, waypoints)
	rhs_result = bike.rhs(steerD)
	u = rhs_result[0]
	values = rhs_result[1]
	new_xB = bike.xB + values[0]*TIMESTEP
	new_yB = bike.yB + values[1]*TIMESTEP
	new_phi = bike.phi + values[2]*TIMESTEP
	new_psi = bike.psi+ values[3]*TIMESTEP
	new_delta = bike.delta + values[4]*TIMESTEP
	new_w_r = bike.w_r + values[5]*TIMESTEP
	new_v = bike.v + values[6]*TIMESTEP
	new_state = bikeState.Bike(new_xB, new_yB, new_phi, new_psi, new_delta, new_w_r, new_v)
	return new_state