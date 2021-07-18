# Kalman Filter Tracker #
Learning the kalman filter one complexity at a time

The numeric kalman filter does not have matrices and is a simple example of calculating errors and updating a numeric state.

The process is as follows:

1) Setup observations from a sensor as a list
2) Instantiate kalman filter object with parameters
3) Calculate kalman gain
4) Calculate a kalman state estimate
5) Calculate the error in the estimate to the observation
6) Get the next measurement from sensor list and repeat from step 3 for remaining measurements
7) Plot the results.


1D, 2D, 3D Filters
---
Moving into more complexity, the 1D, 2D and 3D kalman filters are all the same process but vary in the sizes of the matrices due to the variable they are tracking.

The process is as follows:

1) Create observations list
2) Set up initial parameters: state, covariance matrix, kalman gain, control matrix
3) Instantiate kalman filter object with initial parameters
4) Process the first measurement into the correct matrix format
5) Calculate the predicted state
6) Calculate the accompaning predicted process covariance matrix
7) Calculate the kalman gain
8) Calculate the new state
9) Calculate the new process covariance matrix
10 Gather a new measurement and repeat from step 4

Plot the results


### Explanation

The kalman filter process requires a number of steps to be calculed in sequence. This sequence enables the prediction step and measurement step to be executed before the kalman gain can produce a weighting for each of the steps. Once the kalman gain has the weighting to how much of each step will produce the new state, it calculates the new state and new process covariance matrix. Each calculation will now be broken down:


### Prediction step

The prediction step uses the previous state, previous process covariance matrix and control matrix.

Firstly, the predicted state equation:

<img src="https://latex.codecogs.com/svg/latex?X_p=AX&plus;Bu&plus;w" title="X_p=AX+Bu+w" />

Where Xp is the predicted state, A and B are transitional matrices, u is the control matrix and w is the noise.

Secondly, the predicted process covariance matrix Pp:

<img src="https://latex.codecogs.com/svg.latex?P_p=APA^T&plus;Q" title="P_p=APA^T+Q" />

Where A is a transitional matrix, P is the previous process covariance matrix and Q is the process noise.


### Measurement Step

The measurement step processes the new measurement into the correct matrix format for use. In this step updates to the measurement error can also be updated.

<img src="https://latex.codecogs.com/svg.latex?Y=CY_m&plus;Z" title="Y=CY_m+Z" />

Where Y is the new measurement matrix, C is a transitional matrix, Ym is the raw measurement and Z is the measurement noise.


### Kalman Gain

The Kalman gain depicts how much of the predicted state and how much of the measurement to use in forming the new kalman filtered state. This values changes based on the process covariance matrix as follows:

<img src="https://latex.codecogs.com/svg.latex?K=\frac{P_pH^T}{HP_pH^T&plus;R}" title="K=\frac{P_pH^T}{HP_pH^T+R}" />

Where K is the kalman gain, Pp is the predicted process covariance matrix, H is a transitional matrix enabling matrix devision and R is the measurement error matrix.


### New State

The new state and accompaning process covariance matrix can now be calculated.

<img src="https://latex.codecogs.com/svg.latex?X=X_p&plus;K[Y-HX_p]" title="X=X_p+K[Y-HX_p]" />

<img src="https://latex.codecogs.com/svg.latex?P=(I-KH)P_p" title="P=(I-KH)P_p" />

Where Xp is the predicted state, K is the kalman gain, Y is the measurement, H is a transitional matrix, I is the identity matrix of correct size, and Pp is the predicted process covariance matrix.

With the new state and process covariance matrix, the process can be repeated for a new measurement coming into the system.





