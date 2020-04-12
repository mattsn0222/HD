# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Data structure
### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
## Rubic points

1. The code compiles correctly.
* Code must compile without errors with cmake and make.

No changes in the cmake configuration from the initial repo. Spline file was added in src/spline.h as suggested.

2. Valid Trajectories
* The car is able to drive at least 4.32 miles without incident..

I could confirm the simulator drove upto 7.77 miles without incidents as shown in the below screenshot.

![complete_screen_1](images/complete.jpg)

* The car drives according to the speed limit.

No speed limit message was observed during drive.

* Max Acceleration and Jerk are not Exceeded.

No max jerk message was observed during drive.

* Car does not have collisions.

No collisions was occured during drive.

* The car stays in its lane, except for the time between changing lanes.

The car stays in its lane unless when it changes lane as shown in the below screenshot.

* The car is able to change lanes

The car change lane when the car observed a slow car in front of it (example as below screenshot).
![lane_change](images/lane_change.jpg)

## Code briefing

My main code is from line 100 to line 276. The code consists of three parts below.

1. Prediction (Line 111~)

This part gathers all the environmental information via sensor fusion data.
I defined three bool variables to understand envrironment around the car.

* car_ahead: is there a car in front of our lane.
* car_left: is there a car to the right of our lane.
* car_right: is there a car to the lift of our lane.

I also calulated the target car speed to estimate the target car position by considering latency of the sensor fusion and the current time.
I configured 30 meters as a parameter to judge the target car in range of our car.
So, car_ahead will be true if the target car is within 30 meters in front of us. car_left/car_right will be true if the target car is within 30 meters in front and in back of our car in the left/right lane.

2. Behavior planning (Line 151~)

Based on the prediction, I define our car's reaction depending on cars around us, and our car speed.
The main target situation is to drive the car in the center lane and MAX_SPEED.
So, I check two points, and define the reaction accordingly.

* is there a car in front of us?
- Yes, check car presence on our left/right lane, and if there is no car, the car will change the lane. If cars are in both lanes, decrease speed.
- No, go to next point: speed check.
* is the car speed at MAX_SPEED?
  
3. Trajectory definition (Line 175~)

This code derives the actual trajectory of the car based on the speed and positioning information from the behavior planning and past path points.

Regarding past path points, I used the last two points of the previous trajectory to initialize the spline calculation.
Then, add three points of the future trajectory: 30, 60, 90 meters ahead of the pathway (s direction).
The coordinate is transformed from s, d, yaw to x, y coordinates, and then transformed to car local x, y coordinates to ease spline calculation, then derive spline curve.

The previous path points are copied to the new trajectory output.
The future points are derived by evaluating the spline and transforming the output coordinates to X, Y coordinates. Worth noticing the change in the velocity of the car from line 393 to 398. The speed change is decided on the behavior part of the code, but it is used in that part to increase/decrease speed on every trajectory points instead of doing it for the complete trajectory.

## Reflection

The program successfully let the car drive over 4.32 miles, but I think there is room to improve two points especially.

1. stable speed while a car is in front of us.
When a car is in front of us and lane change is not possible (heavy traffic situation), my car is repeating decelation and acceleration around 30 meters behind of the car in front of us.
This is because my program is targeting MAX_SPEED as baseline.
It may be able to resolve to have another state such as heavy traffic state to control target speed as average speed of cars around us.

2. lane change is not so frequent.
I looked at the drive by my program, the car is not so frequently making lane change even if I felt the car should be able to make lane change.
I thought the judgement of the presence of other cars in left/right lane (30 meters in front/back of us) is conservative.
However, I keep it as it is to prioritize safety.