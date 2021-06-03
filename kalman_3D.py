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
				    [0, 0, 0, 0, 0,  self.PP[5][5]]) # simply matrix
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
	
	# state tracking
	statePosX = []
	statePosY = []
	statePosZ = []
	stateVelX = []
	stateVelY = []
	stateVelZ = []

	# predicted state tracking
	statePPosX = []
	statePPosY = []
	statePPosZ = []
	statePVelX = []
	statePVelY = []
	statePVelZ = []
	

	time = [0, 1, 2, 3, 4]
	
	delta_T = 1

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
