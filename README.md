
# CarND-PID Controller Project (Term 2 - Project #4)
---

This project was to create a P-I-D (Proportional-Integral-Differential) steering control for the Udacity Simulator vehicle traveling down the loop track using the center track error (CTE) as the primary method to correct the steering angle. I programmed a PID controller to proportionally adjust the steering angle relative to the CTE to control the vehicle. The process, methods, and results are described below. I was able to successfully steer the vehicle up to 45 mph over multiple loops of the track without hitting any of the guard rails.   

[//]: # (Image References)

[image1]: ./ScreenCapture1.png "Result"
[image2]: ./ScreenCapture2.png "Result"

[image3]: ./ScreenCapture3.png "Result"
[image4]: ./ScreenCapture4.png "Result"


After the PID controller was working, I obtained the following results for the first part of the track, where with a throttle setting of .4, the car reached at top speed of 45 mph:



| Figure 1 - Vehicle on straight away    | Figure 2 - Vehicle on first curve | 
| :---:                                  |:---:                              |
| ![alt text][image1]                    |  ![alt text][image2]              |



Later in the track, a couple of sharp curves are introduced. The first curve has an open rail (leading to a dirt path) and angle corrections must respond quickly. Finally, the last right turn is dramatic and the vehicle comes pretty close to the gaurd rail but does not hit it:




| Figure 3 - Sharp Left Curve            | Figure 4 - Sharp Right Curve    | 
| :---:                                  |:---:                            |
| ![alt text][image3]                    |  ![alt text][image4]            |



It was very helpful that we had ground truth data to compare answers to in order to root out all the bugs and get a strong result.

---
//----------
// Main module for PID controller Project. This module talks to the Udacity simulator and receives data
// back from the Simulator's vehicle through WebSocket messages. A bulk of this code was starter code from
// Udacity which I modified to handle the results of the "telemetry" message.
//
// Note1: See the main method below for important constansts and current PID tuning parameters.
// Note2: Throttle value currently fixed. Follow-on suggestion to make this in a P loop as well
//----------

---

## Environment to Develop Project

This project was written in C++ on a Macbook Pro with the following configuration:

* macOS Sierra 10.11
* Xcode 8.2.1
* Using uWebsockets for communications w/ Udacity Simulator (v1.4.5)
* uWebsockets needs Openssl 1.0.2
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

---

## Methods

PID class

Formula

The key to implementing the PID class is to correctly update the errors at each step of the vehicle based on the current center track error (CTE) that is the distance between the vehicle to the center track line. This information is supplied by the simulator and updated by:


```sh
void PID::UpdateErrors(double cte) {
d_error = cte - p_error;  // Diff error d/dt(CTE). NOTE: dt assumed =1 so (current CTE - previous CTE) 
p_error = cte;            // Proportional error is just the new CTE
i_error += cte;           // Integral error is sum of ALL CTE
sumsq_error += cte*cte;   // Sum of squared error for RMSE (Error is reference to centerline of 0.0)
}
```


These errors are running totals and are statically kept in the object after initialization. Then, the total adjustment to the steering angle (or the total current error) can be found directly by:


```sh
double PID::UpdateControl() {
double adjust = -Kp*p_error - Ki*i_error - Kd*d_error;  // Equation from class
return adjust;
}
```


Now that this is setup, it is time to find the 3 PID parameter coefficients that give the best performance of keeping the vehicle on the track without hitting the gaurd rail.


---

## Manual Process to find PID parameters

Following the example given in the notes, I started with setting all coefficients to 0.0 except for Kp. I also used a a very slow throttle of around 10 mph. The vehicle zig-zags a lot (this oscillation is expected with only a P-controller) but as you increase and decrease Kp, you can see the wobble increase and decrease. At values of between 0.15 - 0.20, the car could stay on the track most of the time (albiet with a lot of zig-zag!). 

Next, I started to adjust the differential coefficient Kd. This had the effect of making the response to error faster which in turn made the steering corrections faster and less prone to zig-zag. Going up & down, I found that values of between 3.5 - 4.0 produced pretty good results for most of the track. I was also able to increase the throttle to between 0.3 - 0.4, which equates to about 30-40 mph where I obtained similar results. I did notice that the car still hit the guard rail on the last sharp right turn. I also noticed that I needed the higher Kd gain in order to respond to the sharp curves fast enough, at these speeds, in order to not go off the track.

Finally, by adjusting the integral parameter, Ki, I was able to get the car to stay closer to centerline, after a startup time, which ultimately kept the vehicle on the track during the sharpest turns and the largest steering corrections. After some final tweeking, I settled on the following values:

* Kp = .175
* Kd = 4.0
* Ki = .0005

with a throttle of up to .4=40 mph. I did test at some higher speeds and there was more wobble but it didnt go off the full track until over 80 mph!

---

## Automatic Process to find PID parameters (Thrun's Twiddle)

I then implemented Thrun's twiddle algorithm in the PID controller class trying to get the computer to run many simulations to see if it could find a more optimized set of parameters. I discovered quickly that this is an INTERRUPT driven process and that data comes in one point at a time and them my software responds. So, I needed a way to restart a run after changing the parameters and I also needed to figure out how to control the process from a master loop.

I found on the discussion board a snippet of code that can reset the simulator so that seemed to be what I needed. I implemented the algorithm and started to do runs to see if the algorithm could work. I ran into a fundamental problem is that the algorithm given in class assumes you can re-run the track FROM WITHIN THE OPTIMIZER. As this is interrupt driven, where the algorithm just gets woken up each time there is a new data point, I do not see how to implement this approach. I asked my mentor, and he did not implement Twiddle either but stuck with the manual approach.

I might be missing something, but I didnt see how to implement twiddle in this current architecture so I am submitting by report with my working set of manually tuned parameters.

---

## Other Notes on this Project

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid

Note! The files that are needed to compile this project are:

* src/pid.h
* src/pid.ccp
* src/main.ccp
* src/json.hpp 

---
## Future Developments

First, it seems like it would be possible to optimize the PID parameters Kp,Ki,Kd better. I would like to figure out how to implement a version of the hill climb or twiddle algorithm that worked and further optimize the performance.

Second, I note that all of this work has been done at fixed throttle. It was suggested that one could implement a P-controller for the throttle to vary the speed appropriately around the turns. This is a nice idea and would like to do that when I have the chance. Further, with a P-controller on throttle, I believe that a different set of PID parameters would probably give a smoother and superior result.





```python

```
