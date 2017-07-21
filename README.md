# CarND-ExtendedKalmanFilter Project


This is a program that operates a non-linear Extended Kalman Filter (EKF) fusing both RADAR and LASER (Lidar) data together to get an optimal estimate using the CTRV model of to obtain estimate of a vehicle (or pedestrian) location. This program works in conjunction with Udacity's Simulator which it communicates over WebSocket. The simulator also contains ground truth data so an RMSE can be calculated and displayed to check accuracy. I obtains between 5-10cm accuracy for px,py on over both datasets.

[//]: # (Image References)

[image1]: ./UKFSuccessDataset1.png "Result"
[image2]: ./UKFSuccessDataset2.png "Result"

[image3]: ./UKFBug1.png "Result"
[image4]: ./UKFBug2.png "Result"


After the UKF was working, I obtained the following results for both datasets:

| Figure 8 Data (+x/+y start)            | Figure 8 Data (-x/+y start)     | 
| :---:                                  |:---:                            |
| ![alt text][image1]                    |  ![alt text][image2]            |



It is worth noting that all parts of the Kalman Filter must be working robustly in order to get accurate results. Earlier in the development, I had a couple of bugs including not generating the sigma points properly, inserting the incoming sensor measurements (z) in the wrong place, and not re-initializing the augmented state vectors. In these cases, the state estimates can do some wacky things like:



| Bug1 in UKF                            | Bug2 in UKF                     | 
| :---:                                  |:---:                            |
| ![alt text][image3]                    |  ![alt text][image4]            |



It was very helpful that we had ground truth data to compare answers to in order to root out all the bugs and get a strong result.

---

## Environment to Compile Project
This was developed on a Macbook Pro with the following configuration:
* macOS Sierra 10.11
* Xcode 8.2.1
* Using uWebsockets for com w/ Udacity Simulator
* which in turn needs Openssl 1.0.2

*There is also a major dependency with the open source C++ library EIGEN which has the vector and Matrix data structues as well as all the overloaded operators for Matrix algebra for the Kalman Filter equations.


---

## Udacity Notes on this Project

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.
