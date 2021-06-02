#default imports
import time
import random

#math imports
import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter:
	"""
	This class Kalman filter is for 2 DOF systems creating states with position and velocity
	in x and y directions (or two other parameters)
	This class calculates the predicted state, predicted covariance matrix, kalman gain, 
	new state, new state covariance matrix, and also processes new measurements.

	---
	Glossary
	---
	delta_T = time between measurements
	K = kalman gain
	state = kalman filter state
	stateP = predicted state
	P = process covariance matrix
	PP = predicted process covariance matrix
	control_u = control matrix u
	H = transformation matrix (whatever it needs to be)
	"""

	def __init__(self, delta_T, K, state, P, control_u, R):
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

		# Observation error matrix
		self.R = R
	
		
	def statePCalc(self):
		"""
		Calculate the predicted state
		"""
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

	def statePCovarianceCalc(self):
		"""
		Calculate the predicted process covariance matrix
		"""
		A = np.array([[1, 0, self.delta_T,            0],
			      [0, 1,            0, self.delta_T],
			      [0, 0,            1,            0],
			      [0, 0,            0,            1]])
		
		self.PP = np.matmul(np.matmul(A, self.P), A.transpose()) # + self.Q
		self.PP = np.array([[self.PP[0][0], 0, 0, 0],
				    [0, self.PP[1][1], 0, 0],
				    [0, 0, self.PP[2][2], 0],
				    [0, 0, 0, self.PP[3][3]]]) # simply matrix

		return self.PP

	def kalmanGainCalc(self):
		"""
		Calculate kalman gain
		"""
		
		#update parameters
		H = np.identity(4)
		
		self.K = np.matmul(self.PP, H.transpose()) / (np.matmul(np.matmul(H, self.PP), H.transpose()) + self.R)
		self.K = np.array([[self.K[0][0], 0, 0, 0],
				   [0, self.K[1][1], 0, 0],
				   [0, 0, self.K[2][2], 0],
				   [0, 0, 0, self.K[3][3]]])
		return self.K

	def measurementCalc(self, measurement):
		"""
		Calculate new measurement
		Must be a 4x1 matrix
		"""
		#C = np.array([1, 0, 0, 0]) # position measurment only
		
		# Use Pos x, Pos y, Vel x, Vel y
		C = np.array([[1, 0, 0, 0],
			      [0, 1, 0, 0], 
			      [0, 0, 1, 0], 
			      [0, 0, 0, 1]])
		
		Y = np.matmul(C, measurement)
		return Y

	def newStateCalc(self, Y):
		"""
		Calculate new state
		"""
		
		# update parameters
		H = np.identity(4)
		self.PP = self.statePCovarianceCalc()
		self.K = self.kalmanGainCalc()
	
		self.state = self.stateP + np.matmul(self.K, (Y - np.matmul(H, self.stateP)))
		self.P = self.newCovarianceCalc()
		return self.state

	def newCovarianceCalc(self):
		"""
		Calculate new covariance matrix
		"""
		H = np.identity(4)
		self.P = np.matmul((np.identity(4) - np.matmul(self.K, H)), self.PP)
		return self.P
	
def main():
	"""
	Initital estimate: posx=4000, Posy=3000, velx=280, vely=100
	
	Change the observations for your measurements
	"""	
	# Observations	
	Pos_obs_x = [4000, 4260, 4550, 4860, 5110]
	Pos_obs_y = [3000, 2910, 2820, 2920, 3000]
	Vel_obs_x = [ 280,  282,  285,  286,  290]
	Vel_obs_y = [ 120,  110,   50,   60,   90]

	# kalman state tracking
	statePosX = []
	statePosY = []
	stateVelX = []
	stateVelY = []

	# predicted state tracking
	statePosPX = []
	statePosPY = []
	stateVelPX = []
	stateVelPY = []
	time = [0, 1, 2, 3, 4]
	
	delta_T = 1

	initial_state = np.array([[4000], 
				  [3000], 
				  [280],
				  [120]]) # x, y, xdot, ydot given

	initial_P = np.array([[20**2, 0,  0, 0],
			      [ 0, 5**2,  0, 0],
			      [ 0, 0, 10**2, 0],
			      [ 0, 0, 0,  2**2]]) # 20m x error, 5m/s x vel error, 10m y error, 2m/s y vel error

	control_u = np.array([[2],
			      [1]]) # ax, ay	
	
	K = np.array([[0.5, 0, 0, 0],
		      [0, 0.5, 0, 0],
		      [0, 0, 0.5, 0],
		      [0, 0, 0, 0.5]])
	
	deltaX_ObsError = 20	
	deltaY_ObsError = 10	
	deltaVX_ObsError = 5	
	deltaVY_ObsError = 2	
	R = np.array([[deltaX_ObsError**2, 0,  0, 0],
       		      [0, deltaY_ObsError**2,  0, 0],
		      [0, 0, deltaVX_ObsError**2, 0],
		      [0, 0, 0, deltaVY_ObsError**2]])
	
	kalmanfilter = KalmanFilter(delta_T, K, initial_state, initial_P, control_u, R)
	i = 1		
	for i in range(len(Pos_obs_x)):
		measurement = kalmanfilter.measurementCalc(np.array([[Pos_obs_x[i]],
								     [Pos_obs_y[i]],
								     [Vel_obs_x[i]],
								     [Vel_obs_y[i]]]))
	
		stateP = kalmanfilter.statePCalc()
		statePosPX.append(stateP[0])
		statePosPY.append(stateP[1])
		stateVelPX.append(stateP[2])
		stateVelPY.append(stateP[3])
		print("predicted state: ", stateP)

		state = kalmanfilter.newStateCalc(measurement)
		statePosX.append(state[0])
		statePosY.append(state[1])
		stateVelX.append(state[2])
		stateVelY.append(state[3])
		print("state: ", state)
		
		i = i + 1	
		print
		print
	
	# PLOTTING	
	fig, axs = plt.subplots(3,2)

	print(len(time))
	print(len(statePosX))
	print(len(statePosPX))
	print(len(Pos_obs_x))	
	axs[0,0].plot(time, statePosX, label="kalman filter")
	axs[0,0].plot(time, statePosPX, label="predicted")
	axs[0,0].plot(time, Pos_obs_x, label="observation")
	axs[0,0].set_title("State Position X")
	axs[0,0].set_xlabel("Time (seconds)")
	axs[0,0].set_ylabel("Distance (meters)")
	axs[0,0].legend(loc='upper left')

	axs[0,1].plot(time, statePosY, label="kalman filter")
	axs[0,1].plot(time, statePosPY, label="predicted")
	axs[0,1].plot(time, Pos_obs_y, label="observation")
	axs[0,1].set_title("State Position Y")
	axs[0,1].set_xlabel("Time (seconds)")
	axs[0,1].set_ylabel("Distance (meters)")
	axs[0,1].legend(loc='upper left')
	
	axs[1,0].plot(time, stateVelX, label="kalman filter")
	axs[1,0].plot(time, stateVelPX, label="predicted")
	axs[1,0].plot(time, Vel_obs_x, label="observation")
	axs[1,0].set_title("State Velocity X")
	axs[1,0].set_xlabel("Time (seconds)")
	axs[1,0].set_ylabel("Velocity (meters per second)")
	axs[1,0].legend(loc='upper left')
	
	axs[1,1].plot(time, stateVelY, label="kalman filter")
	axs[1,1].plot(time, stateVelPY, label="predicted")
	axs[1,1].plot(time, Vel_obs_y, label="observation")
	axs[1,1].set_title("State Velocity Y")
	axs[1,1].set_xlabel("Time (seconds)")
	axs[1,1].set_ylabel("Velocity (meters per second)")
	axs[1,1].legend(loc='upper left')
	
	axs[2,0].plot(statePosX, statePosY)
	axs[2,0].set_title("Trajectory")
	axs[2,0].set_xlabel("X pos")
	axs[2,0].set_ylabel("Y pos")
	fig.tight_layout()
	plt.legend()
	plt.show()




	
if __name__=="__main__":
	main()
