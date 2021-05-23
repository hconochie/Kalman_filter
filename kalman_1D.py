#default imports
import time
import random

#math imports
import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter:
	"""
	The Kalman filter for a basic single variable plot. The target variable is 25.

	This class calulates the gain, estimate and estimate error, while updating the estimate, estimate error and measurment error.
	"""

	def __init__(self, EST, E_EST, E_MEA, KG):
		# Kalman Gain
		self.KG = KG

		# Estimates
		self.EST = EST
		self.E_EST = E_EST

		# Measurments
		self.E_MEA = E_MEA


	def gainCalc(self):
		# Calculate Kalman Gain (KG)
		#print("KG calc: E_EST: ", E_EST, " E_MEA: ", E_MEA)
		self.KG = float(self.E_EST)/(self.E_EST + self.E_MEA)
		#print("KG calc: ", KG)
		return self.KG

	def estimateCalc(self, MEA):
		self.EST = self.EST + self.KG*(MEA - self.EST)
			
		return self.EST

	def errorCalc(self):
		self.E_EST = (1-self.KG)*(self.EST)
		
		return self.E_EST
	
def main():
	# Set up plotting
	data = [27,28,21,34,30,22,23,25,28,21,29,32,24,24,26,27,20,31,27,21]
	
	estimates = []
	time = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
	print("start kalman")
	
	# Starting the kalman filter
	# Initial estimate = 29, initial est error= 3, initial mea error= 2, initial kalman gain = 0.5	
	kalmanfilter = KalmanFilter(29,3,2, 0.5)
	
	for measurement in data:
		
		KG = kalmanfilter.gainCalc()
		print("kalman gain: ", KG)
		
		EST = kalmanfilter.estimateCalc(measurement)
		estimates.append(EST)
		print("estimate: ", EST)

		E_EST = kalmanfilter.errorCalc()
		print("error estimate: ", E_EST)
		
	plt.plot(time, estimates, label= "estimates")
	plt.plot(time, data, label= "data")
	plt.xlim([0,23])
	plt.ylim([0,40])
	
	plt.legend()
	plt.show()



if __name__=="__main__":
	main()
