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

psi - gives the orientation (heading angle) of the vehicle and 

v - gives us the current velocity that the vehicle is traveling at. 

![MPC State](https://github.com/manopaul/Autonomous-Vehicles/blob/master/10%20-%20Model%20Predictive%20Control/images/State.png)

The state vector is [x,y,ψ,v]

Actuators: 
The such as steering angle (delta) and acceleration (positive of moving forward and negative for braking) are control inputs that need to also be taken into account. 
The cost function is not limited to the state of the vehicle along. We can also include the control inputs. This way, we can adjust for sharp turns when changing lanes by computing the cost function with a compensation (penalizing) factor for making smoother and non-abrupt turns. 

Upon computing the cte and epsi, the new state vector is [x,y,ψ,v,cte,eψ].
![MPC New State Vector](https://github.com/manopaul/Autonomous-Vehicles/blob/master/10%20-%20Model%20Predictive%20Control/images/State_Vector.png)

Update Equation for the model now takes into account the 4 State variables (px, py, psi and v) and the 2 Control inputs (delta and a).

![MPC Model Equations](https://github.com/manopaul/Autonomous-Vehicles/blob/master/10%20-%20Model%20Predictive%20Control/images/Model_Equations.png)

## Timestep Length and Elapsed Duration (N & dt)
Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Started out by using the values that were given in lesson at 25 and 0.05 and found the simulation to be slow and choppy. With repeated trials it seemed that the value of 7 and 0.1 for N and dt was optimal and the simulation was not slow. 
Increasing the value of N made the simulation slow and decreasing the value of dt to 0.05 made the vehicle oscillate relatively more as the vehicle was trying to fit around the reference trajectory. 

## Polynomial Fitting and MPC Preprocessing
A polynomial is fitted to waypoints.
If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The reference or desired trajectory is usually passed to the control block as a polynomial. In the lesson we worked with 1st order polynomial but in this MPC project, a 3rd order polynomial was used since 3rd order polynomials can fit trajectories for most roads. 
Using the polyfit function, the 3rd order polynomial that was computed was made to fit to the given x and y coordinates that represented the waypoints. Then the polyeval function was used to evaluate y values for each given x coordinate. 

## Model Predictive Control with Latency
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Additionally, taking into account the car's vantage point, the px and py values at the point where the car is was set as 0 and since the angle in which the car is heading was considered as the x axis the psi at that point was also set a 0, simplifying the equation for MPC.

In a real world situation since the actuation command to change the steering angle or speed won't execute instantly - there will be a delay as the command propagates through the system. This is refered to as latency and speculated to be about 100 milliseconds. Latency was handled in this project by using the previously (N-2) steering angle (delta) and acceleration (a) values instead of the computed (N-1) values.

# Simulation
The vehicle must successfully drive a lap around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok.

Video of the simulation can be found at:
[![MPC Controller Video](https://github.com/manopaul/Autonomous-Vehicles/blob/master/10%20-%20Model%20Predictive%20Control/images/MPC-YouTube.png)](https://youtu.be/Y37twslZZ4Y)

# Credits
Images - screen captured from Udacity Course Lessons
