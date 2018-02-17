# **Extended Kalman Filter**

---

**Extended Kalman Filter Project**

The goals / steps of this project are the following:

* Implementation of the Extended Kalman Filter in C++ to track the position and velocity of an object.
* Achievement of a RMSE under the specified in the project rubric.

[//]: # (Image References)

[image1]: ./output_images/RMSE.png "Undistorted"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.  

---


### Algorithm structure and processing flow

The algorithm is divided in the following source code files located in `/src`:

`main.cpp` -> Main routine to read the sensor data and call the processing measurement function.

`FusionEKF.cpp` -> Processing of each measurement, initialization of Kalman Filter matrices and call to predict and update functions.

`kalman_filter.cpp` -> Implementation of the predict and update steps.

`tools.cpp` -> Calculation of RMSE and jacobian matrix. Conversion from cartesian to polar coordinates. Adjustment of diff. angle.

The main function calls the processing measurement function of `FusionEKF.cpp`. From here the basic steps of the filter, predict and update, are performed via calls to the functions defined in `kalman_filter.cpp`.

#### 1. Handling of first measurement

The function `ProcessMeasurement` initializes the state vector in the first cycle with the first received measurement. The flag `is_initialized_` determines whether the initialization has already been made.

#### 2. Prediction and update border

The algorithm predicts first and then updates. `FusionEKF.cpp` firstly calls `Predict` and then `Update` or `UpdateEKF`, in function of the corresponding sensor.

#### 3. Handling of radar and lidar measurements

The algorithm detects whether the received measurement comes from the lidar or radar sensor in the `main` function and performs the handling of the update step via `Update` (laser) or `UpdateEKF` (radar).

The measurement covariance and measurement matrices are resized in function of the sensor type. In particular, R is 3x3 for radar and 2x2 for lidar, and H is 3x4 for radar and 2x4 for laser.

For radar, H is computed by calculating the jacobian of the matrix. Additionally, the predicted location is mapped to polar coordinates (`CartToPolar`) and the angle obtained after updating with the measurement is adjusted (`AdjustAngle`)

### Accuracy

The algorithm reaches a final RMSE of [.0974 .0856 .4542 .4408]:

![alt text][image1]

This amount satisfies the limit proposed in the project rubric ([.11 .11 .52 .52]).

### Compilation

As specified in the rubric, the code compiles using `cmake` and `make`.

It is important to mention that `CMakeLists.txt` was moved to the `/src` folder, since the Eclipse IDE was used during the project.

### Code efficiency

With regard to this point, particular attention has been paid to the realization of repeated calculations only once and its storage in variables. This makes the code specially faster in the function `CalculateJacobian`, where the squared root of the position is computed several times.

Additionally, unnecessary loops or loops that run many times have been avoided.

### Final notes

It was observed that the RMSE is influenced by the use of functions that modify the precision in the calculations. In particular, I initially used the function `pow` of the C++ `math` library to square the positions in the jacobian calculations, but the RMSE went above the limit for the X component of the velocity. Without its use (computing the square using simple multiplication) I achieved a RMSE under this limit.
