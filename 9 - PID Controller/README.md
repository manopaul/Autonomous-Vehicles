# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

The Proportional (P), Integral (I) and Differential (D) coefficients each has an impact in the way the vehicle drove around the track. 
In order to understand how they impacted the drive, I tried setting just one of the coefficients first (P Only, D Only, I Only) and then combination of two of the coefficients (PD only, DI only, PI only), before setting all three coefficients (PID) and observed the drive in the simulator. 

Given below are the observations (also commented out in the main.cpp file)
When the P coefficient was the only one set as pid.Init(0.15,0.0,0.0), the car oscillated a lot and reached a maximum speed of ~35 mph before getting off road.
When the D coefficient was the only one set as pid.Init(0.0,0.0004,0.0), the car veered off to the left once it started and reached a maximum speed of ~35 mph before getting off road.
When the I coefficient was the only one set as pid.Init(0.0,0.0,4.0), the car veered off to the right once it started and reached a maximum speed of ~40 mph before getting off road.

When the P and D coeficient were set as pid.Init(0.15,0.0004,0.0), the car oscillated from the started and veers off the road reaching a max speed of ~40 mph 
When the D and I coeficient were set as pid.Init(0.0,0.0004,4.0), the car stayed on course till a little while after the bridge. It did not osciallate as much but veers off the road at sharp turns, reaching a max speed of ~65 mph
When the P and I coeficient were set as pid.Init(0.15,0.0,4.0), interestingly the car stayed on course reaching a max speed of ~71 mph. It seemed like the differential value did not impact the drive. 

When the P and D and I controller were set as pid.Init(0.15,0.0004,4.0), the car stays on course throughout and reaches the maximum speed of ~75 mph. This seemed to be the most optimal.

When these parameters were determined, I attempted to try different combinations of PID coefficients to see their impact.
When the PID coefficients were initialized as pid.Init(0.15,0.5,4.0) (note: large D value), interestingly from the start, the car started going in reverse and veered of the road. 

Reverting back to the original value for the D coefficient (0.0004), I reduce the I coefficient from 4.0 to 3.0 and noticed that the car stayed on course throughout reaching a maximum speed of ~71 mph, but was relatively oscillating more on the bridge. 
Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

### Describe how the final hyperparameters were chosen. Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

Hyperparameters were identified by several manual trials and errors (tuning). 

Attempted to implement twiddle to determine coefficients but since the class lesson was in python and the project was in C++, porting over the syntax was a challenge. Would have been nice if the class lesson was also in C++ or the project was in python. Either way, understood the concept of Twiddle which was great. 

