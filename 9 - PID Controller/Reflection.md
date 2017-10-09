## Reflection

#### Describe the effect each of the P, I, D components had in your implementation.

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
[![PID Controller - Large D Coefficient](https://github.com/manopaul/Autonomous-Vehicles/blob/master/9%20-%20PID%20Controller/images/PID_Run_Thumbnail.png)](https://youtu.be/ydCSwQjmIQM)

When these parameters were determined, I attempted to try different combinations of PID coefficients to see their impact.
When the PID coefficients were initialized as pid.Init(0.15,0.5,4.0) (note: large D value), interestingly from the start, the car started going in reverse and veered of the road.
[![PID Controller - Large D Coefficient](https://github.com/manopaul/Autonomous-Vehicles/blob/master/9%20-%20PID%20Controller/images/PID_Reverse_Thumbnail.png)](https://youtu.be/4eW6z7TcikI)


Reverting back to the original value for the D coefficient (0.0004), I reduce the I coefficient from 4.0 to 3.0 and noticed that the car stayed on course throughout reaching a maximum speed of ~71 mph, but was relatively oscillating more on the bridge. 
Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

#### Describe how the final hyperparameters were chosen. 

Hyperparameters were identified by several manual trials and errors (tuning). 

Attempted to implement twiddle to determine coefficients but since the class lesson was in python and the project was in C++, porting over the syntax was a challenge. Would have been nice if the class lesson was also in C++ or the project was in python. Either way, understood the concept of Twiddle which was great. 
