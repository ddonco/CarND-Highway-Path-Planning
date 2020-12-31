# Highway Planning Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[screenshot1]: ./images/Screenshot_1.png "Simulator Screenshor 1"
[screenshot2]: ./images/Screenshot_2.png "Simulator Screenshor 2"

### Goals

![alt text][screenshot1]

The goal of this project is to safely navigate a car around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulated highway environment provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car will try to go as close as possible to the 50 MPH speed limit, which means it will pass slower traffic when possible. The car will avoid hitting other cars and will drive inside the marked road lanes at all times, unless going from one lane to another. The car will be able to make one complete loop around the 6946m highway. The car has been programmed to not experience a total acceleration over 10 m/s^2 or a jerk that's greater than 10 m/s^3.

### Reflection

This project employs open source code for the spline generation ([found here](https://kluge.in-chemnitz.de/opensource/spline/)) and a finite state machine template ([found here](https://github.com/eglimi/cppfsm)). Additionally, several path planning concepts including planning a vehicle trajectory, lane changing, and implementing a spline have been taken from the project Q&A video. The path planning logic of this project can be broken into three sections: Vehicle Location Prediction, Driving Behavior Planning, and Calculate Trajectory.

![alt text][screenshot2]

#### 1. Vehicle Location Prediction

Found in `main.cpp` starting at line 159. This section iterates over the sensor fusion data and determines which lane each detected vehicle occupies. Next, the code checks whether or not each vehicle is directly ahead, to the left, or to the right and if the vehicle is close enough to be an obsticle for driving at the desired highway speed or to prevent a lane change in a particular direction.

#### 2. Driving Behavior Planning

Found in `main.cpp` starting at line 206. This section triggers the finite state machine to change state based on whether or not the road ahead is free of other vehicles. If a vehicle is ahead, the state machine will either initiate a lane change if possible, or follow the vehile ahead until a lane change is possible. Following the vehicle ahead involved decelerating until there is a 30 m gap and then accelerating again to maintain highway speed.

#### 3. Calculate Trajectory

Found in `main.cpp` starting at line 212. This section builds a spline from the point at the front of the car to several point ahead of the car at 30, 60, and 90 m distances. The spline provides a smooth trajectory for the car to follow and prevent uncomfortable accelerations and jerks. Once the car begins moving, the spline is built using two points from the previous trajectory plus the three points in the distance. The target waypoints for the car are calculated in the car's (x, y) coordinate system then transformed into the maps coordinate system to simplify the trajectory calculations.

## Project Data

#### The map of the highway is in data/highway_map.txt

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

### Dependencies

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