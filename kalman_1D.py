#default imports
import time
import random

#math imports
import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter:
	"""
	This class Kalman filter is for 1 DOF systems creating states with postion and velocity.

	This class calulates the predicted state, predicted covariance matrix, kalman gain,
	new state and new state covariance matrix, and also processes the new measurements.
	
	---
	Glossary
	---
	delta_T = time between measurements
	K = kalman gain
	state = kalman filter state
	stateP = predicted state
	P = process covariance matrix
	PP = predicted covariance matrix
	control_u = control matrix u
	H = transformation matrix (whatever it is required to be)
	"""

	def __init__(self, delta_T, K, state, P, control_u, H):
		# Time per measurement (constant value) 
		self.delta_T = delta_T
		
		# Kalman Gain
		self.K = K

		# Current state
		self.state = state
		
		# Process covariance matrix
		self.P = P
		
		# Control matrix u
		self.control_u = control_u
		
		# H matrix
		self.H = H
		
	def statePCalc(self):
		"""Calculating the predicted state"""
		A = np.array([[1, self.delta_T],
			      [0,            1]])
		
		B = np.array([[0.5 * self.delta_T**2],
			      [      self.delta_T]])
		
		# Predicted states
		self.stateP = np.matmul(A, self.state) + np.matmul(B, self.control_u)
		
		print("previous state",self.state)
		print("predicted state: ", self.stateP)
		return self.stateP # 2x1 matrix

	def statePCovarianceCalc(self):
		"""Calculating the predicted process covariance matrix"""
		print("prev process covariance matrix P: ", self.P)
		A = np.array([[1, self.delta_T],
			      [0,            1]])
		# State Covariance Matrix
		self.PP = np.matmul(np.matmul(A, self.P), A.transpose()) # + self.Q
		self.PP = np.array([[self.PP[0][0],             0],
				    [            0, self.PP[1][1]]])
		
		print("predicted covariance matrix: ", self.PP)	
		return self.PP # 2x2 matrix
	
	def kalmanGainCalc(self):
		"""Kalman Gain"""
		H = np.identity(2)
		
		R = np.array([[25**2,    0],
			      [    0, 6**2]]) # observation errors delta_x = 25, delta_Vx = 6

		self.K = np.matmul(self.PP, self.H.transpose()) / (np.matmul(np.matmul(self.H, self.PP), self.H.transpose()) + R)
		self.K = np.array([[self.K[0][0],            0],
				   [           0, self.K[1][1]]])
		print("kalman gain: ", self.K)
		return self.K # 2x2 matrix 
	
	def measurementCalc(self, measurement): 
		"""measurement should be a 2x1 matrix (column)"""
		# C = np.array([1, 0]) # position measurment only
		C = np.array([[1, 0],
	                     [0, 1]]) # position and velocity measurement

		Y = np.matmul(C, measurement) # + Z
		return Y # 2x1 matrix

	def newStateCalc(self, Y):
		"""Calculating the current state"""
		
		# Parameters
		self.PP = self.statePCovarianceCalc()
		self.K = self.kalmanGainCalc()	

		self.state = self.stateP + np.matmul(self.K, (Y- np.matmul(self.H,self.stateP)))
		self.P = self.newCovarianceCalc()
		print("New State! : ",self.state)
		print("new covariance matrix: ", self.P)	
		return self.state

	def newCovarianceCalc(self):
		"Calculate new covariance matrix"
		self.P = np.matmul((np.identity(2) - self.K*self.H), self.PP)
		return self.P
			
def main():
	"""
	Initial estimate: Pos=4000, Vel=280
	
	Change the observations your measurements
	"""
	# Observations
	Pos_obs = [4000, 4260, 4550, 4860, 5110]
	Vel_obs = [ 280,  282,  285,  286,  290]	
	
	# predicted kalman filter states
	statePosP = [Pos_obs[0]]
	stateVelP = [Vel_obs[0]]
	
	# kalman filter states	
	statePos = [Pos_obs[0]]
	stateVel = [Vel_obs[0]]
	time = [0]
	
	delta_T = 1

	# Initial Kalman gain (changes in the first instance)
	K = np.array([[0.5,   0],
		      [0  , 0.5]])

	initial_state = np.array([[Pos_obs[0]],
				  [Vel_obs[0]]])
	
	# Initial process errors matrix 
	initial_P = np.array([[20**2, 0],
			      [ 0, 5**2]]) # delta_x = 20 delta_Vx = 5

	# 1DOF system has 1 acceleration for the control
	control_u = np.array([[2]]) # ax	

	# Change this if required (just for transformations)
	H = np.identity(2)
	
	# Start the Kalman Filter
	kalmanfilter = KalmanFilter(delta_T, K, initial_state, initial_P, control_u, H)
	i = 1		
	while i in range(len(Pos_obs)):
		measurement = kalmanfilter.measurementCalc(np.array([[Pos_obs[i]],
								     [Vel_obs[i]]]))
		print("new measurement: ", measurement)
		stateP = kalmanfilter.statePCalc()
		statePosP.append(stateP[0])
		stateVelP.append(stateP[1])
		state = kalmanfilter.newStateCalc(measurement)
		statePos.append(state[0])
		stateVel.append(state[1])
		time.append(i)
		i = i + 1
		print
		print	
	
	# PLOTTING	
	fig, axs = plt.subplots(2)
	
	axs[0].plot(time, statePos, label="kalman filter")
	axs[0].plot(time, Pos_obs, label="observations")
	axs[0].plot(time, statePosP, label="predicted")
	axs[0].set_title("State Position")
	axs[0].set_xlabel("Time (seconds)")
	axs[0].set_ylabel("Distance (meters)")
	axs[0].legend(loc = 'upper left')
	axs[1].plot(time, stateVel, label="kalman filter")
	axs[1].plot(time, Vel_obs, label="observations")
	axs[1].plot(time, stateVelP, label="predicted")
	axs[1].set_title("State Velocity")
	axs[1].set_xlabel("Time (seconds)")
	axs[1].set_ylabel("Velocity (meters per second)")
	axs[1].legend(loc = 'upper left')
	
	fig.tight_layout()
	plt.show()




	
if __name__=="__main__":
	main()
