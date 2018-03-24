This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. 
This project combines the various concepts learned throughout the course, from camera vision (OpenCV), Neural networks (could have done traffic light classification but instead used camera vision techniques to determine light state), Control (PID control), behavior planning) integrating it with ROS.

Individual Name: 
Mano Paul (dash4rk(at)gmail(dot)com)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/manopaul/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator. Select Highway and then check Camera and uncheck Manual.

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
