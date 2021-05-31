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
		A = np.array([[1, self.delta_T],
			      [0,            1]])
		B = np.array([[0.5 * self.delta_T],
			      [self.delta_T]])
		
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
	

	statePos = []
	stateVel = []
	time = []
	
	initial_state = np.array([[50], [5]])
	control_u = np.array([[-9.81]])	
	
	kalmanfilter = KalmanFilter(0, initial_state, control_u)
	i = 0		
	while i < 10:
		
		state = kalmanfilter.newStateCalc()
		statePos.append(state[0])
		stateVel.append(state[1])
		time.append(i)
		
		i = i + 1	
	
	fig, axs = plt.subplots(2)
	
	axs[0].plot(time, statePos)
	axs[0].set_title("State Position")
	axs[0].set_xlabel("Time (seconds)")
	axs[0].set_ylabel("Distance (meters)")
	axs[1].plot(time, stateVel)
	axs[1].set_title("State Velocity")
	axs[1].set_xlabel("Time (seconds)")
	axs[1].set_ylabel("Velocity (meters per second)")
	
	fig.tight_layout()
	plt.legend()
	plt.show()




	
if __name__=="__main__":
	main()
