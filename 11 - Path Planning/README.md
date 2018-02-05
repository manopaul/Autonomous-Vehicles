# Path Planning Project

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

# Project Rubric

##Compilation

The code compiles correctly without errors with cmake and make.

##Valid Trajectories

The car is able to drive at least 4.32 miles without incident.

The car drives within the maximum allowed speed limit of 50 mph. Also the car regulates its speed when it has traffic ahead or around it and drives at 't driving much slower than speed limit unless obstructed by traffic.

Max Acceleration and Jerk are not Exceeded.

The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

Car does not have collisions.

The car stays in its lane, except for the time between changing lanes. The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

##Reflection

In this project, the car in the simulator was able to successfully navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
The car's localization and sensor fusion data was provided from a highway map (data/highway_map.txt) file along with a sparse map list of waypoints around the highway.
The car did not exceed the max speed limit of 50 mph and successfully change lanes and pass other vehicles on the road without collisions or exceeding max jerk (10 m/s^3) and total accelerations (10 m/s^2). The car was able to make one complete loop around the 6946m highway.

### Lane Width and Current Lane (Code lines 204-207)
The Lane width is set to 4 meters.
The current lane is set to 1, which is the middle lane (0 is left, 1 is middle, 3 is right).

### Speed Limit, Speed Change and Safe Distance Constants (Code lines 231-234)

Code lines 231-234 sets the following Constants
Maximum allowed speed limit in mph at 49.5 mph as K_MAX_SPEED_LIMIT = 49.5
Maximum allowed speed change at 0.224 which is ~ 5mph change as K_SPEED_CHANGE = 0.224
Buffer zone around the car and when no other cars or objects are within this range the car can change lanes as K_SAFE_DISTANCE = 30.0

### Waypoint, Previous path, and Sensor Fusion data provided (Code lines 236-252)

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

Previous path data (as listed below) is given to the planner for smooth driving.
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

Additionally, sensor fusion data (as given below) that is provided to the planner lists out all the other car's attributes on the driving side of the road.
["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

### Avoid Collisions (Code lines 254-257)
Previous path size is determined and used to avoid collisions by setting the car_s position to the end_path_s frenet coordinates.

### Other Vehicles (Code lines 262-264)
Boolean values to check and see if there are other vehicle in front of, to the left and to the right of our car is set initially as false. This means that we start out by assuming that we are the only car on the road.

### Current Lane (Code lines 268-280)
The current lane of the car is determined using the d frenet coordinate and the lane width.
Since the lane width is 4 meters wide.
If the d frenet coordinate is greater than or equal to 0 and lesser than or equal to 4, the car is in the left lane.
If the d frenet coordinate is greater than 4 and lesser than or equal to 8, the car is in the middle lane
If the d frenet coordinate is greater than 8 and lesser than or equal to 12, the car is in the right lane

### Vehicles around (Code lines 282-302)
Once the lane in which our car is determined, then using the sensor fusion data and the configured safe distance, other vehicles that are around our car is determined and the boolean flags that set if a vehicle is in front, to the left or to the right is set accordingly.

### Car Behavior - Change Lanes Speed Adjustment (Code lines 305-331)
When our car is obstructed because of a vehicle in front, it checks to see if it is safe to change lanes by checking whether we have any other vehicle to the left or to the right of the car. If it is safe, then the car would change lanes to the left or to the right accordingly. If it is unsafe, then our car will reduce it speed by 0.224 factor (~ 5 mph) to maintain the safe distance.

### Using Spline (Code lines 435 - 439)
A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

### Spline points (Code lines 452-470)
By determining what our target x and target y position on the spline needs to be and using the safe distance for the car,
we calculate the spline points so that we can travel at our desired reference velocity and we fill up the rest of the path planner after filling it with previous points (always outputting 50 points)

### Improvements
Though this code compiles and meets the requirements of our car to navigate a course autonomously, there are certain improvements that can be made to improve this.
An improvement to this code would be to dynamically determine the lane width using camera vision and ground truth data for the road the car is in.
Also instead of setting the current lane of the car to be the middle lane as the starting point, which is physically incongruent, the code can be improved to find out the number of lanes on the road that the car is in and start it at the right most lane, if you are driving in a country that drives on the right side of the road or start the car by setting its current lane to the left most lane if you are driving in a country that drives on the left side of the road, to be more realistic.
Speed limit should be dynamically provided using the GPS coordinates and not hard coded.
Speed change should be dynamically determined using the speed of the car, localization information and other sensor fusion data.
Buffer zone (Safe Distance) should also not be statically set as it can lead the car to get boxed in heavy traffic situations.
When our car is obstructed with an vehicle in front, the code checks to see if the lane to the left is safe and then it checks the right lane. This is fine when the country law needs you to overtake on the left, but this is not the most efficient and the right lane may be completely free and safe. Simulating a human driver in this situation would require the code to be unbiased in terms of preference to the left or the right.
