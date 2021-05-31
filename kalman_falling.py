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
		

	#def gainCalc(self):
		# Calculate Kalman Gain (KG)
		#self.KG = float(self.E_EST)/(self.E_EST + self.E_MEA)
		#return self.KG

	def newStateCalc(self):
		# Calculate new state
		A = np.array([[1, self.delta_T], [0, 1]])
		B = np.array([[0.5 * self.delta_T], [self.delta_T]])
		
		self.state = np.matmul(A, self.state) + np.matmul(B, self.control_u)
		self.delta_T = self.delta_T + 1
		return self.state

	#def updateMeasurement(self):
		# update measurement matrix with new measurement

		#C = np.array([1, 0])
		
		

	#def errorCalc(self):
		# Calulate new error in the new estimate
		#self.E_EST = (1-self.KG)*(self.EST)		
		#return self.E_EST
	
def main():
	
	# Starting the kalman filter
	# Initial estimate = 29, initial est error= 3, initial mea error= 2, initial kalman gain = 0.5
	
	# Initial parameters for falling object t = 0, fall from 50m with initial velocity of 5m/s
	
	initial_state = np.array([[50], [5]])
	control_u = np.array([[-9.81]])	
	
	kalmanfilter = KalmanFilter(0, initial_state, control_u)
	i = 0		
	while i < 10:
		
		state = kalmanfilter.newStateCalc()
		# statelist.append(state)
		print("state: ", state)
		
		i = i + 1	
	
if __name__=="__main__":
	main()
