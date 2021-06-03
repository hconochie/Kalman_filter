#default imports
import time
import random

#math imports
import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter:
	"""
	This class kalman filter is for 3 DOF systems creating states with position and velocity in three axes.
	
	This class calculates the predicted state, predicted covariance matrix, kalman gain, new state
	new state covariance matrix and also processes new measurements
	
	---
	Glossary
	---
	delta_T = time between measurements (constant value)
	K = Kalman gain
	state = kalman filter state
	stateP = predicted state
	P = process covariance matrix
	PP = predicted process covariance matrix
	control_u control matrix u
	H = trasnformation matrix (varies for what it needs to be)
	"""

	def __init__(self, delta_T, K, state, P, control_u, R):
		# Time per measurement 
		self.delta_T = delta_T
		
		# Kalman Gain
		self.K = K	

		# Current state
		self.state = state
	
		# Process covariance matrix P
		self.P = P

		# control matrix u	
		self.control_u = control_u
		
		# Observation error matrix
		self.R = R
		
	def statePCalc(self):
		"""
		Calculate the predicted state
		"""
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
		
		self.stateP = np.matmul(A, self.state) + np.matmul(B, self.control_u)
		return self.stateP

	def statePCovarianceCalc(self):
		A = np.array([[1, 0, 0, self.delta_T,            0,            0],
			      [0, 1, 0,            0, self.delta_T,            0],
			      [0, 0, 1,            0,            0, self.delta_T],
			      [0, 0, 0,            1,            0,            0],
			      [0, 0, 0,            0,            1,            0],
			      [0, 0, 0,            0,            0,            1]])

		self.PP = np.matmul(np.matmul(A, self.P), A.transpose()) # + self.Q
		self.PP = np.array([[self.PP[0][0], 0, 0, 0, 0, 0],
				    [0, self.PP[1][1], 0, 0, 0, 0],
				    [0, 0, self.PP[2][2], 0, 0, 0],
				    [0, 0, 0, self.PP[3][3], 0, 0],
				    [0, 0, 0, 0, self.PP[4][4], 0],
				    [0, 0, 0, 0, 0, self.PP[5][5]]]) # simply matrix
		return self.PP
		
	def kalmanGainCalc(self):
		"""
		Calculate kalman gain
		"""
		# parameters
		H = np.identity(6)
		
		self.K = np.matmul(self.PP, H.transpose()) / (np.matmul(np.matmul(H, self.PP), H.transpose()) + self.R)
		self.K = np.array([[self.K[0][0], 0, 0, 0, 0, 0],
				   [0, self.K[1][1], 0, 0, 0, 0],
				   [0, 0, self.K[2][2], 0, 0, 0],
				   [0, 0, 0, self.K[3][3], 0, 0],
				   [0, 0, 0, 0, self.K[4][4], 0],
				   [0, 0, 0, 0, 0, self.K[5][5]]])
		return self.K

	def measurementCalc(self, measurement):
		C = np.array([[1, 0, 0, 0, 0, 0],
		              [0, 1, 0, 0, 0, 0],
			      [0, 0, 1, 0, 0, 0],
			      [0, 0, 0, 1, 0, 0],
			      [0, 0, 0, 0, 1, 0],
			      [0, 0, 0, 0, 0, 1]]) # position and velocity measurements in XYZ
		
		Y = np.matmul(C, measurement)
		return Y
	
	def newStateCalc(self, Y):
		"""
		Calculate new state
		"""
		# update parameters
		H = np.identity(6)
		self.PP = self.statePCovarianceCalc()
		self.K = self.kalmanGainCalc()
		
		self.state = self.stateP + np.matmul(self.K, (Y - np.matmul(H, self.stateP)))
		self.P = self.newCovarianceCalc()
		return self.state

	def newCovarianceCalc(self):
		"""
		Calculate new covariance matrix
		"""
		H = np.identity(6)
		self.P = np.matmul((np.identity(6) - np.matmul(self.K, H)), self.PP)
		return self.P


def main():
	
	# Starting the kalman filter
	# Initial estimate = 29, initial est error= 3, initial mea error= 2, initial kalman gain = 0.5
	
	# Initial parameters for falling object t = 0, fall from 50m with initial velocity of 5m/s
	
	# Observations
	Pos_obs_x = [4000, 4260, 4550, 4860, 5110]
	Pos_obs_y = [3000, 2910, 2820, 2920, 3000]
	Pos_obs_z = [200, 190, 180, 195, 210]
	Vel_obs_x = [280, 282, 285, 286, 290]	
	Vel_obs_y = [120, 110, 50, 60, 90]	
	Vel_obs_z = [5, 5, 7, 9, 5]
	
	# state tracking
	statePosX = []
	statePosY = []
	statePosZ = []
	stateVelX = []
	stateVelY = []
	stateVelZ = []

	# predicted state tracking
	statePosPX = []
	statePosPY = []
	statePosPZ = []
	stateVelPX = []
	stateVelPY = []
	stateVelPZ = []
	

	time = [0, 1, 2, 3, 4]
	
	delta_T = 1

	initial_state = np.array([[4000], 
				  [3000], 
				  [280],
				  [280],
				  [120],
				  [5]]) # x, y, z, xdot, ydot, zdot

	initial_P = np.array([[20**2, 0, 0, 0, 0, 0],
			      [0,  5**2, 0, 0, 0, 0],
			      [0, 0,  3**2, 0, 0, 0],
			      [0, 0, 0, 10**2, 0, 0],
			      [0, 0, 0, 0,  2**2, 0],
			      [0, 0, 0, 0, 0,  1**2]])
	
	control_u = np.array([[  1],
			      [ -2],
			      [0.2]]) # ax, ay, az	

	K = np.array([[0.5, 0, 0, 0, 0, 0],
		      [0, 0.5, 0, 0, 0, 0],
		      [0, 0, 0.5, 0, 0, 0],
		      [0, 0, 0, 0.5, 0, 0],
		      [0, 0, 0, 0, 0.5, 0],
		      [0, 0, 0, 0, 0, 0.5]])

	deltaX_ObsError = 20
	deltaY_ObsError = 10
	deltaZ_ObsError = 1

	deltaVX_ObsError = 5
	deltaVY_ObsError = 2
	deltaVZ_ObsError = 1

	R = np.array([[ deltaX_ObsError, 0, 0, 0, 0, 0],
		      [0,  deltaY_ObsError, 0, 0, 0, 0],
		      [0, 0,  deltaZ_ObsError, 0, 0, 0],
		      [0, 0, 0, deltaVX_ObsError, 0, 0],
		      [0, 0, 0, 0, deltaVY_ObsError, 0],
		      [0, 0, 0, 0, 0, deltaVZ_ObsError]])
	
	kalmanfilter = KalmanFilter(delta_T, K, initial_state, initial_P, control_u, R)
	i = 1		
	for i in range(len(Pos_obs_x)):
	
		measurement = kalmanfilter.measurementCalc(np.array([[Pos_obs_x[i]],
								     [Pos_obs_y[i]],
								     [Pos_obs_z[i]],
								     [Vel_obs_x[i]],
								     [Vel_obs_y[i]],
								     [Vel_obs_z[i]]]))

		stateP = kalmanfilter.statePCalc()
		statePosPX.append(stateP[0])	
		statePosPY.append(stateP[1])	
		statePosPZ.append(stateP[2])	
		stateVelPX.append(stateP[3])	
		stateVelPY.append(stateP[4])	
		stateVelPZ.append(stateP[5])
		print("predicted state: ", stateP)
		
		state = kalmanfilter.newStateCalc(measurement)
		statePosX.append(state[0])
		statePosY.append(state[1])
		statePosZ.append(state[2])
		stateVelX.append(state[3])
		stateVelY.append(state[4])
		stateVelZ.append(state[5])
		print("state: ", state)	

		i = i + 1
		print
		print	
	
	# PLOTTING	
	fig, axs = plt.subplots(4,2)
	axs[0,0].plot(time, statePosX, label="kalman filter")
	axs[0,0].plot(time, statePosPX, label="predicted")
	axs[0,0].plot(time, Pos_obs_x, label="Observation")
	axs[0,0].set_title("State Position X")
	axs[0,0].set_xlabel("Time")
	axs[0,0].set_ylabel("Distance")
	axs[0,0].legend(loc= 'upper left', prop={'size': 6})
	
	axs[0,1].plot(time, statePosY, label="kalman filter")
	axs[0,1].plot(time, statePosPY, label="predicted")
	axs[0,1].plot(time, Pos_obs_y, label="Observation")
	axs[0,1].set_title("State Position Y")
	axs[0,1].set_xlabel("Time")
	axs[0,1].set_ylabel("Distance")
	axs[0,1].legend(loc= 'upper left', prop={'size': 6})
	
	axs[1,0].plot(time, statePosZ, label="kalman filter")
	axs[1,0].plot(time, statePosPZ, label="predicted")
	axs[1,0].plot(time, Pos_obs_z, label="Observation")
	axs[1,0].set_title("State Position Z")
	axs[1,0].set_xlabel("Time")
	axs[1,0].set_ylabel("Distance")
	axs[1,0].legend(loc= 'upper left', prop={'size': 6})
	
	axs[1,1].plot(time, stateVelX, label="kalman filter")
	axs[1,1].plot(time, stateVelPX, label="predicted")
	axs[1,1].plot(time, Vel_obs_x, label="Observation")
	axs[1,1].set_title("State Velocity X")
	axs[1,1].set_xlabel("Time")
	axs[1,1].set_ylabel("Velocity")
	axs[1,1].legend(loc= 'upper left', prop={'size': 6})
	
	axs[2,0].plot(time, stateVelY, label="kalman filter")
	axs[2,0].plot(time, stateVelPY, label="predicted")
	axs[2,0].plot(time, Vel_obs_y, label="Observation")
	axs[2,0].set_title("State Velocity Y")
	axs[2,0].set_xlabel("Time")
	axs[2,0].set_ylabel("Velocity")
	axs[2,0].legend(loc= 'upper left', prop={'size': 6})
	
	
	axs[2,1].plot(time, stateVelZ, label="kalman filter")
	axs[2,1].plot(time, stateVelPZ, label="predicted")
	axs[2,1].plot(time, Vel_obs_z, label="Observation")
	axs[2,1].set_title("State Velocity Z")
	axs[2,1].set_xlabel("Time")
	axs[2,1].set_ylabel("Velocity")
	axs[2,1].legend(loc= 'upper left', prop={'size': 6})
	
	axs[3,0].plot(statePosX, statePosY)
	axs[3,0].set_title("Trajectory")
	axs[3,0].set_xlabel("X pos")
	axs[3,0].set_ylabel("Y pos")
	
	axs[3,1].plot(statePosX, statePosZ)
	axs[3,1].set_title("Trajectory")
	axs[3,1].set_xlabel("X pos")
	axs[3,1].set_ylabel("Z pos")
	
	fig.tight_layout()
	plt.legend()
	plt.show()




	
if __name__=="__main__":
	main()
