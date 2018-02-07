# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## PID

A PID controller is a control loop feed-back mechanism that allows for the control of an output signal given a transfer function.

It is composed of 3 main controllers:

* P - Proportional controller:
  The proportional controller takes in the current error and multiplies it by a constant Kp. The idea is to penalize the system for having large errors. 
  This will allow the controller to cause the output signal to reach a steady state value which is only 0 if the reference value is 0 (steady state error)

* I - Integator controller:
  The integrator controller integrates the error and multplies it with some constant Ki. The idea is to penalize the system for continuously having error.
  This will make the steady state value shrink to 0 since the controller will increase (linearly) if the error is constant.

* D - Derivative controller:
  The derivative control differentiates the error signal and multiplies it with some constant Kd. The idea is to penalize the system for having rapid changes in error.
  This will smooth the output signal and reduce overshoot.

## Tuning

Kp = 0.15
Ki = 0.00
Kd = 0.1
Throttle speed = 0.15

The constants for the PID controller were tuned manually. 

1. The Proportional controller parameter was tuned until the car was steadily oscilating.
2. The Derivative controller parameter was tuned until the overshoot decreased.
3. The Integrator controller parameter was tried but increasing the parameter caused the systems to become unstable. 
Only very small values ~0.0001 allowed the controller to work properly. Even setting the value to 0 allowed the controller to accomplish the task.

Since the reference signal is 0 (Car staying in the middle of the road), the proportional controller will not produce a steady state error

## Software info

The software has 4 main files:

1. PID.h - Header file with prototypes
2. PID.c - Defines the PID controller class
3. main.cpp - Main driver code. 
      * Instantiates the PID controller with tuning parameters.
      * Receives the state information from the simulator
4. json.hpp

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

