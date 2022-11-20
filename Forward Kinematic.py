#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from numpy import matmul
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

q1 = np.array([-150, 150, 10])
q2 = np.array([-150, 270, 162])
q3 = np.array([94, 270, 162])
q4 = np.array([307, 177, 162])
q5 = np.array([307, 260, 162])
q6 = np.array([390, 260, 162])
## MAY NEED CHANGE
q7 = np.array([0,0,0])
q8 = np.array([0,0,0])
## MAY NEED CHANGE

w1 = np.array([0, 0, 1])
w2 = np.array([0, 1, 0])
w3 = np.array([0, 1, 0])
w4 = np.array([0, 1, 0])
w5 = np.array([1, 0, 0])
w6 = np.array([0, 1, 0])
## MAY NEED CHANGE
w7 = np.array([0,0,0])
w8 = np.array([0,0,0])
## MAY NEED CHANGE

v1 = np.cross(-w1, q1)
v2 = np.cross(-w2, q2)
v3 = np.cross(-w3, q3)
v4 = np.cross(-w4, q4)
v5 = np.cross(-w5, q5)
v6 = np.cross(-w6, q6)
v7 = np.cross(-w7, q7)
v8 = np.cross(-w8, q8)

def bracket_S(w):
	## MAY NEED CHANGE
	return np.array([[0, -w[2], w[1], w[3]],
					[w[2], 0, -w[0], w[4]],
					[-w[1], w[0], 0, w[5]],
					[0, 	0,	0,		0]])
	## MAY NEED CHANGE

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	
	## MAY NEED CHANGE
	M = np.array([[0,0,1,390],
				 [-1,0,0,399],
				 [0,-1,0,215.5],
				 [0,0,0,1]])
	## MAY NEED CHANGE

	s1 = np.array([w1,v1]).flatten()
	s2 = np.array([w2,v2]).flatten()
	s3 = np.array([w3,v3]).flatten()
	s4 = np.array([w4,v4]).flatten()
	s5 = np.array([w5,v5]).flatten()
	s6 = np.array([w6,v6]).flatten()
	s7 = np.array([w7,v7]).flatten()
	s8 = np.array([w8,v8]).flatten()

	S=np.array([s1,s2,s3,s4,s5,s6,s7,s8])
	
	# ==============================================================#
	
	return M, S

"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6,theta7,theta8):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")
	M, S = Get_MS()
	# =================== Your code starts here ====================#

	t1 = expm(theta1*bracket_S(S[0]))
	t2 = expm(theta2*bracket_S(S[1]))
	t3 = expm(theta3*bracket_S(S[2]))
	t4 = expm(theta4*bracket_S(S[3]))
	t5 = expm(theta5*bracket_S(S[4]))
	t6 = expm(theta6*bracket_S(S[5]))
	t7 = expm(theta6*bracket_S(S[6]))
	t8 = expm(theta6*bracket_S(S[7]))

	T = matmul(matmul(matmul(matmul(t1,t2),matmul(t3,t4)),matuml(t5,t6)),matmul(matmul(t7,t8),M))
	# ==============================================================#

	print(str(T) + "\n")
	## MAY NEED CHANGE
	return_value[0] = theta1
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4
	return_value[4] = theta5
	return_value[5] = theta6
	return_value[6] = theta7
	return_value[7] = theta8
	## MAY NEED CHANGE
	return return_value