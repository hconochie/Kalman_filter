# Kalman_filter
Learning the kalman filter one complexity at a time

The numeric kalman filter does not have matrices and is a simple example calculating errors and updating a numeric state.

The process is as follows:

1) setup observations from sensor as a list
2) Instantiate kalman filter object with parameters
3) Calculate kalman gain
4) Calculate a kalman state estimate
5) Calculate the error in the estimate to the observation
6) Get the next measurement from sensor list and repeat from step 3

Plot the results.

---
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

---
Explanation
---
The kalman filter process requires a number of steps to be calculed in sequence. This sequence enables the prediction step and measurement step to be executed before the kalman gain can produce a weighting for each of the steps. Once the kalman gain has the weighting to how much of each step will produce the new state, it calculates the new state and new process covariance matrix. Each calculation will now be broken down:

---
Prediction step
---
The prediction step uses the previous state, previous process covariance matrix and control matrix.

Firstly, the predicted state equation:

X_p = A*X + B*u + w

Where X_p is the predicted state, A and B are transitional matrices, u is the control matrix and w is the noise.

Secondly, the predicted process covariance matrix P_p:

P_p = A*P*A^T + Q

Where A is a transitional matrix, P is the previous process covariance matrix and Q is the process noise.

---
Measurement Step
---
The measurement step processes the new measurement into the correct matrix format for use. In this step updates to the measurement error can also be updated.

Y = C*Y_m + Z

Where Y is the new measurement matrix, C is a transitional matrix, Y_m is the raw measurement and Z is the measurement noise.

---
Kalman Gain
---
The Kalman gain depicts how much of the predicted state and how much of the measurement to use in forming the new kalman filtered state. This values changes based on the process covariance matrix as follows:

K = P_P*H^T / (H*P_P*H^T + R)

Where K is the kalman gain, P_p is the predicted process covariance matrix, H is a transitional matrix enabling matrix devision and R is the measurement error matrix.

---
New State
---
The new state and accompaning process covariance matrix can now be calculated.

X = X_p + K[Y-H*X_p]

P = (I-K*H)*P_P

Where X_p is the predicted state, K is the kalman gain, Y is the measurement, H is a transitional matrix, I is the identity matrix of correct size, and P_P is the predicted process covariance matrix.

With the new state and process covariance matrix, the process can be repeated for a new measurement coming into the system.





