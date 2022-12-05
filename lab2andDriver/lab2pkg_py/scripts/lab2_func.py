#!/usr/bin/env python3
from math import cos, sin, pi, asin, acos, sqrt, atan2
import numpy as np
from scipy.linalg import expm
from lab2_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = [[0, -1, 0, 0.39],[0, 0, -1, 0.401],[1, 0, 0, 0.2155],[0, 0, 0, 1]]

	S = [
			[
				[0, -1, 0, 0.15],
				[1, 0, 0, 0.15],
				[0, 0, 0, 0],
				[0, 0, 0, 0]
			],
			[
				[0, 0, 1, -0.162],
				[0, 0, 0, 0],
				[-1, 0, 0, -0.15],
				[0, 0, 0, 0]
			],
			[
				[0, 0, 1, -0.162],
				[0, 0, 0, 0],
				[-1, 0, 0, 0.094],
				[0, 0, 0, 0]
			],
			[
				[0, 0, 1, -0.162],
				[0, 0, 0, 0],
				[-1, 0, 0, 0.307],
				[0, 0, 0, 0]
			],
			[
				[0, 0, 0, 0],
				[0, 0, -1, 0.162],
				[0, 1, 0, -0.26],
				[0, 0, 0, 0]
			],
			[
				[0, 0, 1, -0.162],
				[0, 0, 0, 0],
				[-1, 0, 0, 0.39],
				[0, 0, 0, 0]
			]
	]

	S = np.array(S)




	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):


	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========

	# =================== Your code starts here ====================#
	M, S = Get_MS()

	# theta1 = theta1*PI/180.0
	# theta2 = theta2*PI/180.0
	# theta3 = theta3*PI/180.0
	# theta4 = theta4*PI/180.0
	# theta5 = theta5*PI/180.0
	# theta6 = theta6*PI/180.0

	t1 = expm(theta1*S[0])
	t2 = expm(theta2*S[1])
	t3 = expm(theta3*S[2])
	t4 = expm(theta4*S[3])
	t5 = expm(theta5*S[4])
	t6 = expm(theta6*S[5])

	T = np.dot(t1, np.dot(t1, np.dot(t3, np.dot(t4, np.dot(t5, np.dot(t6, M))))))

	# ==============================================================#

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	yaw_WgripRad = yaw_WgripDegree / 180 * pi

	xgrip = xWgrip + 0.15
	ygrip = yWgrip - 0.15
	zgrip = zWgrip - 0.01

	xcen = xgrip - 0.0535*cos(yaw_WgripRad)
	ycen = ygrip - 0.0535*sin(yaw_WgripRad)
	zcen = zgrip	

	theta1 = pi / 2 - asin(0.11/abs(sqrt(xcen**2 + ycen**2))) - atan2(xcen, ycen)
	theta6 = pi / 2 - yaw_WgripRad + theta1

	x3end = xcen - 0.083*cos(theta1) + 0.11*sin(theta1)
	y3end = ycen - 0.083*sin(theta1) - 0.11*cos(theta1)
	z3end = zcen + 0.141

	theta2 = acos((0.244**2 + (x3end**2 + y3end**2 + (z3end-0.152)**2) - 0.213**2) / (2*0.244*sqrt((x3end**2 + y3end**2 + (z3end-0.152)**2)))) + asin((z3end-0.152) / sqrt((x3end**2 + y3end**2 + (z3end-0.152)**2)))
	theta3 = pi - acos((0.244**2 + 0.213**2 - (x3end**2 + y3end**2 + (z3end-0.152)**2))/(2*0.244*0.213))
	theta4 = theta3 - theta2
	theta5 = -pi / 2

	theta2 = -theta2
	theta4 = -theta4
	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
