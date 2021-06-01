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

	def __init__(self, delta_T, state, P, control_u):
		# time 
		self.delta_T = delta_T
		
		# Kalman Gain
		

		# States
		self.state = state
		self.P = P # process covariance matrix
		self.control_u = control_u
		
		# Measurments
		
	def newStateCalc(self):
		# Calculate new state
		A = np.array([[1, self.delta_T],
			      [0,            1]])
		
		B = np.array([[0.5 * self.delta_T],
			      [      self.delta_T]])
		
		self.state = np.matmul(A, self.state) + np.matmul(B, self.control_u)
		self.delta_T = self.delta_T + 1
		return self.state

	def measurementCalc(self, measurement):
		# C = np.array([1, 0]) # position measurment only
		C = np.array([1, 0],
	                     [0, 1]) # position and velocity measurement
		Y = np.matmul(C, measurement)

	def currentStateCalc(self, Y)
		# Calculating the current state
		H = np.identity(2)
		self.state = self.state + self.K * (Y- np.matmul(H,self.state))

	def stateCovarianceCalc(self):
		A = np.array([[1, self.delta_T],
			      [0,            1]])
		# State Covariance Matrix
		self.P = np.matmul(np.matmul(A, self.P), A.transpose())
		
	def kalmanGainCalc(self):
		# Kalman Gain
		H = np.identity(2)
		
		R = np.array([[25**2,    0],
			      [    0, 6**2]])
		self.K = np.matmul(self.P, H.transpose()) / (np.matmul(np.matmul(H, self.P), H.transpose()) + R)

			
def main():
	
	# Starting the kalman filter
	# Initial estimate = 29, initial est error= 3, initial mea error= 2, initial kalman gain = 0.5
	
	# Initial parameters for falling object t = 0, fall from 50m with initial velocity of 5m/s
	
	# Observations
	Pos_obs = [4000, 4260, 4550, 4860, 5110]
	Vel_obs = [ 280,  282,  285,  286,  290]	

	statePos = []
	stateVel = []
	time = []
	
	initial_state = np.array([[Pos_obs[0]],
				  [Vel_obs[0]]])
	
	# Initial process errors matrix 
	initial_P = np.array([[20**2, 20*5],
			      [ 5*20, 5**2]]) # delta_x = 20 delta_Vx = 5

	control_u = np.array([[-9.81]])	
	
	kalmanfilter = KalmanFilter(0, initial_state, initial_P, control_u)
	i = 0		
	while i in len(range(Pos_obs)):
		
		state = kalmanfilter.newStateCalc()
		statePos.append(state[0])
		stateVel.append(state[1])
		time.append(i)
		
		i = i + 1	
	
	# PLOTTING	
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
