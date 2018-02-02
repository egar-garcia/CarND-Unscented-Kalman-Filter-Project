**Extended Kalman Filter Project**


Self-Driving Car Engineer Nanodegree Program

In this project is utilized an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

[//]: # (Image References)
[image1]: ./Docs/run_dataset1.png
[image2]: ./Docs/run_dataset2.png

### Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./ExtendedKF
```

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x ["estimate_y"] <= kalman filter estimated position y ["rmse_x"] ["rmse_y"] ["rmse_vx"] ["rmse_vy"]


## [Rubric](https://review.udacity.com/#!/rubrics/783/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Compiling

#### 1. Code must compile without errors with cmake and make.

As the file ```CMakeLists.txt``` was made as general as possible (including necessary steps for compiling in Mac, which is the platform I used to work in this project) I left it unchanged. Thus, after setting up [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), the compilation and building can be done using the standard procedure (from the project's top directory):

```
mkdir build
cd build
cmake ..
make
./ExtendedKF
```


### Accuracy

#### 1. px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt", which is the same data file the simulator uses for Dataset 1.

Running the algorithm agains Dataset 1 (see following screenshot), in the simulator produced the RMSE [0.0729, 0.0636, 0.3464, 0.2331], which is less than the required pass mark.
![image1]

When running agains Dataset 2 (see following screenshot) the RMSE was [0.0721, 0.0742, 0.2369, 0.2233], which is also less than the required pass mark.
![image2]


### Follows the Correct Algorithm

#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The implementation follows the steps learned during the lessons, following the following steps to perform the prediction/update cycle:

* Prediction:
  * Calculating augmented sigma points
  * Performing prediction for sigma points
  * Predicting mean and covariance matrix
* Update
  * Predicting measurement (adjusted according to the type of measure from radar or lidar)
  * Updating state  (adjusted according to the type of measure from radar or lidar)


#### 2. Your Kalman Filter algorithm handles the first measurements appropriately.

The first measurement is used to initialize the state vector. In the case where the first measurement is from the lidar, only the X and Y components are extracted, everything else is initialized as zero. In the case where the first measure is from the radar, a polar coordinate is converted to cartesian to set the positions in X and Y, the velocity given by the radar is the one used as the initial velocity, and the other values are set to zero.

State covariance matrix P, and measurement noise covariance matrix R (two distinct for radar an lidar respectively), can actually be initialized in the constructor.


#### 3. Your Kalman Filter algorithm first predicts then updates.

Upon receiving a measurement after the first measurement, the algorithm applies the previously mentioned steps in each cycle to first predict the object's position to the current timestep, and then update the prediction using the new measurement.


#### 4. Your Kalman Filter can handle radar and lidar measurements.

The implemented algorithm sets up the appropriate steps given the measurement model (i.e. cartesian coordinates from lidar and polar coordinates from radar) to be transformed into the CTRV model or vice versa. Specifically the normalization of angles is undertaken to make sure angles are expressed in radians in the range [-PI, PI].


### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.

Modifications to the formulas were made in order to do some common calculations just once. Also the code was tried to be simplified to avoid unnecessary loops, data structures, complex control flow or code redundancy.

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.

One of the problems I faced was to tune the parameters to increase the accuracy of the process and reducing RMSE bellow the pass mark.

Initially with the default values given for process noise standard deviation longitudinal acceleration and yaw acceleration of 30 for both, the process was sticking in a loop with super high delay due to the predicted angles were so huge (around 10^19), thus the normalization process was unable to finish in reasonable time. The solution was to adjust the values process noise, I was trying different combinations, then finally the one that produced better results was 1) process noise standard deviation longitudinal acceleration of 1.5 m/s^2 and 2) process noise standard deviation yaw acceleration of 0.3 rad/s^2.

Another values I was experimenting with, were the ones for the state covariance matrix, I started with the identity matrix (with ones in the diagonal), and tried reducing the values till trying with the ones produced after a full testing cycle. I could observed that the smaller the values the less accuracy in the final result, due to smaller values represent a high confidence in the measurement and that conflicts with the lack of precision of the first measurement. At the end the identity matrix with just ones in the diagonal produced the best results.

The second set was specially complicated for getting an RMES under the passmark, particularly for the velocity. The solution to this problem was to set the velocity given by the first measurement (which in this case is from the radar) in the state. With this small change the RMES was reduced enormously even to beat the results of the first run. The general observation is that the lidar is better to estimate the position and the radar to estimate the velocity.
