#default imports
import time
import random

#math imports
import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter:
	"""
	The Kalman filter is multi dimensional.

	This class calulates the gain, estimate and estimate error, while updating the estimate, estimate error and measurment error.
	
	MEA = measurement
	E_MEA = error in the measurement
	EST = estimate
	E_EST = error in the estimate
	KG = kalman gain
	"""

	def __init__(self, delta_T, state, control_u):
		# time 
		self.delta_T = delta_T
		
		# Kalman Gain
		

		# States
		self.state = state
		
		self.control_u = control_u
		
		# Measurments
		
	def newStateCalc(self):
		# Calculate new state
		A = np.array([[1, 0, 0, self.delta_T,            0,            0],
			      [0, 1, 0,            0, self.delta_T,            0],
			      [0, 0, 1,            0,            0, self.delta_T],
			      [0, 0, 0,            1,            0,            0],
			      [0, 0, 0,            0,            1,            0],
			      [0, 0, 0,            0,            0,            1]])

		B = np.array([[0.5 * self.delta_T **2,                   0,                   0],
			      [                     0, 0.5*self.delta_T**2,                   0],
			      [                     0,                   0, 0.5*self.delta_T**2],
			      [          self.delta_T,                   0,                   0],
			      [                     0,        self.delta_T,                   0],
			      [                     0,                   0,        self.delta_T]])
		
		self.state = np.matmul(A, self.state) + np.matmul(B, self.control_u)
		self.delta_T = self.delta_T + 1
		return self.state

	def measurementCalc(self, measurement):
		C = np.array([1, 0]) # position measurment only
		# C = np.array([1, 0],
		#	       [0, 1]) # position and velocity measurement
		Y = np.matmul(C, measurement)
	
def main():
	
	# Starting the kalman filter
	# Initial estimate = 29, initial est error= 3, initial mea error= 2, initial kalman gain = 0.5
	
	# Initial parameters for falling object t = 0, fall from 50m with initial velocity of 5m/s
	

	statePosX = []
	statePosY = []
	statePosZ = []
	stateVelX = []
	stateVelY = []
	stateVelZ = []
	time = []
	
	delta_T = 0

	initial_state = np.array([[3], 
				  [2], 
				  [1],
				  [0.5],
				  [  1],
				  [ -1]]) # x, y, z, xdot, ydot, zdot

	control_u = np.array([[  1],
			      [ -2],
			      [0.2]]) # ax, ay, az	
	
	kalmanfilter = KalmanFilter(delta_T, initial_state, control_u)
	i = 0		
	while i < 10:
		
		state = kalmanfilter.newStateCalc()
		statePosX.append(state[0])
		statePosY.append(state[1])
		statePosZ.append(state[2])
		stateVelX.append(state[3])
		stateVelY.append(state[4])
		stateVelZ.append(state[5])
		time.append(i)
		
		i = i + 1	
	
	# PLOTTING	
	fig, axs = plt.subplots(3)
	
	axs[0].plot(time, statePosX, label="Pos X")
	axs[0].plot(time, statePosY, label="Pos Y")
	axs[0].plot(time, statePosZ, label="Pos Z")
	axs[0].set_title("State Position")
	axs[0].set_xlabel("Time (seconds)")
	axs[0].set_ylabel("Distance (meters)")
	axs[0].legend()
	
	axs[1].plot(time, stateVelX, label="Vel X")
	axs[1].plot(time, stateVelY, label="Vel Y")
	axs[1].plot(time, stateVelZ, label="Vel Z")
	axs[1].set_title("State Velocity")
	axs[1].set_xlabel("Time (seconds)")
	axs[1].set_ylabel("Velocity (meters per second)")
	axs[1].legend()
	
	axs[2].plot(statePosX, statePosY)
	axs[2].set_title("Trajectory")
	axs[2].set_xlabel("X pos")
	axs[2].set_ylabel("Y pos")
	fig.tight_layout()
	plt.legend()
	plt.show()




	
if __name__=="__main__":
	main()
