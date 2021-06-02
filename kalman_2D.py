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

	def __init__(self, delta_T, K, state, P, control_u, H):
		# time 
		self.delta_T = delta_T
		
		# Kalman Gain
		self.K = K

		# States
		self.state = state
		self.P = P
		
		# control matrix u
		self.control_u = control_u
	
		# H matrix
		self.H = H	
		
	def statePCalc(self):
		"Calculate new predicted state"
		A = np.array([[1, 0, self.delta_T,            0],
			      [0, 1,            0, self.delta_T],
			      [0, 0,            1,            0],
			      [0, 0,            0,            1]])

		B = np.array([[0.5 * self.delta_T **2,                   0],
			      [0                     , 0.5*self.delta_T**2],
			      [self.delta_T          ,                   0],
			      [0                     ,        self.delta_T]])
		
		self.stateP = np.matmul(A, self.state) + np.matmul(B, self.control_u)
		self.delta_T = self.delta_T + 1
		return self.stateP

	def statePCovarianceCalc(self, A):
		"Calculate devation matrix a and then the predicted covariance matrix "
		unityMat = np.array([[1, 1],
				     [1, 1]])
		a = A - np.dot(unityMat, A)* 0.2
		self.PP = np.matmul(a.transpose(), a)
		return self.PP

	def kalmanGainCalc(self):
		"calculate kalman gain"
		
		#update parameters
		self.PP = statePCovariancCalc()
		self.R = 

		self.K = np.matmul(self.PP, self.H.transpose()) / (np.matmul(np.matmul(self.H, self.PP), self.H.transpose()) + self.R)
		return self.K

	def measurementCalc(self, measurement):
		"calculate new measurement"
		C = np.array([1, 0]) # position measurment only
		# C = np.array([1, 0],
		#	       [0, 1]) # position and velocity measurement
		Y = np.matmul(C, measurement)

	def newStateCalc(self, measurement):
		"calculate new state"
		
		# update parameters
		self.stateP = statePCalc()
		self.K = kalmanGainCalc()
		self.Y = measurementCalc(measurement)
	
		self.state = self.stateP + self.K * (self.Y - np.matmul(self.H, self.state)


	def newCovarianceCalc(self):
		"calculate new covariance matrix"
		self.P = np.matmul((np.identity(2) - self.K*self.H), self.PP)

	
def main():
	
	# Starting the kalman filter for tracking an aeroplane
	# Initial estimate: [x0=4000, y0=3000, x0dot=280, y0dot=120]

	# Initial conditions: ax=2, vx=280, delta_t=1, delta_x=25

	# Process errors: delta_Px=20, delta_PVx=6

	# Observation errors: delta_x=25, delta_Vx=6
	# Observations in the x, Pos and Vel
	
	Pos_obs = [4000, 4260, 4550, 4860, 5110]
	Vel_obs = [ 280,  282,  285,  286,  290]


	statePosX = []
	statePosY = []
	stateVelX = []
	stateVelY = []
	time = []
	
	delta_T = 0

	initial_state = np.array([[4000], 
				  [3000], 
				  [280],
				  [120]]) # x, y, xdot, ydot given

	initial_P = np.array([20, 5],
			     [ 0, 1]) # 20m x error, 5m/s x vel error

	control_u = np.array([[2],
			      [0]]) # ax, ay	
	
	kalmanfilter = KalmanFilter(delta_T, initial_state, control_u)
	i = 0		
	for i in len(range(Pos_obs)):
		
		state = kalmanfilter.newStateCalc()
		statePosX.append(state[0])
		statePosY.append(state[1])
		stateVelX.append(state[2])
		stateVelY.append(state[3])
		time.append(i)
		
		i = i + 1	
	
	# PLOTTING	
	fig, axs = plt.subplots(3)
	
	axs[0].plot(time, statePosX, label="Pos X")
	axs[0].plot(time, statePosY, label="Pos Y")
	axs[0].set_title("State Position")
	axs[0].set_xlabel("Time (seconds)")
	axs[0].set_ylabel("Distance (meters)")
	axs[0].legend()
	
	axs[1].plot(time, stateVelX, label="Vel X")
	axs[1].plot(time, stateVelY, label="Vel Y")
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
