
The project's GitHub repository contains all of the files that you will need. 
The github repository includes:

1. All of the code that implements the Extended Kalman Filter (EKF) functionality is in the src folder
2. this README file with instructions on compiling the code
3. a Docs folder, which contains details about the structure of the code templates (no changes were made)
4. CMakeLists.txt file that will be used when compiling your code (no changes were made)
5. a data folder with the data file named obj_pose-laser-radar-synthetic-input.txt for testing your extended Kalman filter which the simulator interface provides.
6. The cmakepatch.txt file that was used for making uWebSockets work in a Mac OS system.

To run the Code you have two options
1. Navigate to the build directory in a terminal window and run the ExtendedKF executable using the following syntax
./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt 
The terminal should output "listening on port 4567"
2. Then Run the simulator 
If you are on the Mac, then the simulator can be found in the simulator directory.
When the simulator runs, select the first Project EFK/UKF
The terminal window should now show "Connected!!!"
3. Click the Start button in the simulator
The Terminal will output the results of the EKF filter computations and you will see the measurement and update values plotted on the simulator. 


