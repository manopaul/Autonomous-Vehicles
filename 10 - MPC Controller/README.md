# CarND-Controls-MPC 
Self-Driving Car Engineer Nanodegree Program

---
# Compilation Details (Basic Build Instructions)

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

When you run the program in your terminal (or command prompt), you will see a message "Listening to port 4567"

5. Start the Simulator and navigate to Project 5 in the simulator and click on Select. 

The terminal should output "Connected" and then the simluator should display how the MPC controller works. The terminal will output the MPC calculations as well. 

6. You can stop by quitting the simulator

# Implementation 

## The Model
Student describes their model in detail. This includes the state, actuators and update equations.

Vehicle models that were covered were the Kinematic and dynamic models. In this MPC Controller project, the kinematic model was used to predict the trajectory (center of the road) that the vehicle would need to traverse (aka reference trajectory or desired trajectory), given the waypoints (lake_track_waypoints.csv) as inputs, and computing the cross track error (cte) and the orientation error (epsi). The cte and epsi gives will give us the computed trajectory. The cte tells us how far the vehicle is away from the center of the road or predicted trajectory and the epsi is the error in the orientation angle of the car relatively to the predicted waypoints. The cte and epsi are collectively used as the cost function. 
In order for the vehicle to safely navigate and drive autonomously, the cost function should minimize the error - cte and eψ (epsi).

The kinematic model takes into account the following: State, Actuators, and Update Equations

State:
px, py, psi and v are the four variable used to describe the state of the vehicle. 
px - gives the current position of the vehicle in the X-axis
py - gives the current position of the vehicle in the Y-axis
psi - gives the orientation (heading angle) of the vehicle
and 
v - gives us the current velocity that the vehicle is traveling at. 

Actuators: 
The such as steering angle (delta) and acceleration (positive of moving forward and negative for braking) are control inputs that need to also be taken into account. 
The cost function is not limited to the state of the vehicle along. We can also include the control inputs. This way, we can adjust for sharp turns when changing lanes by computing the cost function with a compensation (penalizing) factor for making smoother and non-abrupt turns. 

Update Equation for the model now takes into account 
the 4 State variables (px, py, psi and v) and 
the 2 Control inputs (delta and a) 
The equations to solve the cost function is now


## Timestep Length and Elapsed Duration (N & dt)
Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Here we had to assign values to N and dt. It's likely you set these variables to slightly different values. That's fine as long as the cross track error decreased to 0. It's a good idea to play with different values here.

For example, if we were to set N to 100, the simulation would run much slower. This is because the solver would have to optimize 4 times as many control inputs. Ipopt, the solver, permutes the control input values until it finds the lowest cost. If you were to open up Ipopt and plot the x and y values as the solver mutates them, the plot would look like a worm moving around trying to fit the shape of the reference trajectory.


## Polynomial Fitting and MPC Preprocessing
A polynomial is fitted to waypoints.
If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The reference or desired trajectory is usually passed to the control block as a polynomial. In the lesson we worked with 1st order polynomial but in this MPC project, a 3rd order polynomial was used since 3rd order polynomials can fit trajectories for most roads. 
Using the polyfit function, the 3rd order polynomial that was computed was made to fit to the given x and y coordinates that represented the waypoints. Then the polyeval function was used to evaluate y values for each given x coordinate. 

## Model Predictive Control with Latency
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

# Simulation

The vehicle must successfully drive a lap around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.




## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
