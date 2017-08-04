
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
2. Then Run the simulator for your flavor of Operating System that is provided in the course
When the simulator runs, select the first Project EFK/UKF
The terminal window should now show "Connected!!!"

To Run the Code

Clone this repo locally and perform
1. Navigate to the directory for this repo in a Terminal (Command Prompt) window 
2. In a terminal, Make a directory called "build" using the mkdir command e.g., mkdir build
3. Change directory and navigate into the build directory using the cd command e.g., cd build
4. Run the cmake command as follows cmake .. (make sure there is a space between the two dots that follow)
5. Run the make command as follows: make
6. You should see a few files get generated and you will see the ExtendedKF executable created.
7. Run the ExtendedKF executable by supplying it with an input file (from the data directory) 
You can see the output of the program on screen or you can pipe it to a log file (EKF_Output.txt) and open it

7.a. To see the computations on screen, run without specifying any output and piping it
./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt 
The terminal should output "listening on port 4567"
Then run the simulator for your flavor of Operating System that is provided in the course
When the simulator runs, select the first Project EFK/UKF
The terminal window should now show "Connected!!!"
Click the Start button in the simulator
The Terminal will output the results of the EKF filter computations and you will see the measurement and update values plotted on the simulator. 

7.b. To log the output and open it, run the program by specifying an output file name and piping it into a log file as follows.
./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt > ../data/EKF_Output.txt
Note you will not see anything printed on the terminal window.
Just run the simulator for your flavor of Operating System that is provided in the course
When the simulator runs, select the first Project EFK/UKF
Click the Start button in the simulator
Once it completes, quit the simulator and 
navigate to the "data" directory where you will see the log file name EKF.log
Open the EKF_Output.txt in any text editor

