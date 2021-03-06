This project's GitHub repository contains all of the files that you will need for the UKF Project. 

The github repository includes:
- all of the code that implements the Unscented Kalman Filter (UKF) functionality is in the src folder
- a data folder with the data file named obj_pose-laser-radar-synthetic-input.txt for testing your UKF which the simulator interface provides.
- this README file with instructions on compiling the code

To Run the Code

Clone this repo locally and perform
Navigate to the directory for this repo in a Terminal (Command Prompt) window
In a terminal, Make a directory called "build" using the mkdir command e.g., mkdir build
Change directory and navigate into the build directory using the cd command e.g., cd build
Run the cmake command as follows cmake .. (make sure there is a space between the two dots that follow)
Run the make command as follows: make
You should see a few files get generated and you will see the UnscentedKF executable created.
Run the UnscentedKF executable by supplying it with an input file (from the data directory). 
You can see the output of the program on screen.

To see the computations on screen, run the following command in your terminal window from the build directory
./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt 

The terminal should output "listening on port 4567" 

Then run the simulator for your flavor of Operating System that is provided in the course. 
When the simulator runs, select the first Project EFK/UKF. 
The terminal window should now show "Connected!!!" 
Click the Start button in the simulator. 

You will see the measurement and update values plotted on the simulator and the RMSE values will be computed and shown in the simulator as well.


NOTE:
To get the RMSE to be within the acceptable range of .09, .10, .40, .30 for px, py, vx, and vy, the process noise standard deviation longitudinal acceleration in m/s^2 (std_a_) was throttled from reduced from the initial 30 to 3 using several trials. 
and the process noise standard deviation yaw acceleration in rad/s^2 (std_yawdd_) was throttled from 30 to 0.4 using several trials.  
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 3; //30
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.4; //30

RMSE calculation in tools.cpp file is the same as was in the EKF project (no changes to the code was made). 
