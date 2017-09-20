# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program

### Introdution

The goal of this project is to make the ego car safely navigate around a virtual highway with other vehicles by following a smooth and safe trajectory. To achieve this goal, I've built a path planner which first takes into account sensor fusion and localization data to predict ego car and other vehicles' states, then implements Finite State Machine to makes decisions of ego car's behaviours, and finally uses the Spline to generate smooth, safe and feasible trajectories which do not provoke any kind of penalties.

### Prediction

The prediction was done for the S value in Frenet coordinates for both ego car and other vehicles. It's simple in term of calculation. 

- **for ego car**: just take the end S value of the previous path as we suppose the ego car will continue previous path till the end if no new ways points were generated.
- **for other vehicles**: 
```future S of the vehicle = actual S of the vehicle + 
vehicle's speed * 
time lapse between waypoints in previous path  * 
nb waypoints left in previous path```

I then compare the ego car's future S value with other vehicles' future S values to decide the behaviour that the ego car should take at one moment.

I also built a class vehicle to store other vehicles' states. Once initialized, I appended vehicle objects to their corresponding lanes (left lane /center lane /right lane) for further checking in Finite State Machine. 

### Behaviour Decisions

I used a pretty simple design of Finite State Machine to make decisions. There are four states (START, KEEP_LANE, CHANGE_LEFT, and CHANGE_RIGHT). 

- The START state is the initial one and the machine cannot come back to it at any time. Once the car is started, the standard and desired behavior is to KEEP_LANE, so stay in the same lane at maximum allowed speed.

- In case there is another car on its trajectory which drives slower, it will first slow down and then try to change lanes. Before taking lane change actions, it will check if the action is feasible. 

- On the target lane, if there are vehicles presented within 15 meters before or behind it, it will consider not possible to change lane.

- If there are vehicles drive within 30 meters in front of it on the target lane but with a lower speed, it will also consider not possbile to change. 

- If the vehicle is 30 meters behind it on the target lane but with a higher speed, it's still not possible for ego car to change lane.

- To decide which lane is the target lane, I didn't explicitly implemented cost functions, but the idea was implicitly incorporated. The ego car will always priorize CHANGE_LEFT than CHANGE_RIGHT (the cost of CHANGE_LEFT is smaller than CHANGE_RIGHT). If CHANGE_LEFT is not feasible (already on the lane most left / other vehicles presented), then it will consider to CHANGE_RIGHT if the right lane is safe and free to move in.

- If at any time it's not possible to change lanes, it will continue to reduce the speed in order to keep a minimum distance with the vehicle in front of it.

### Path Generation

To make the path smooth, I first fed the path with the lefting points of previous path. Then the library Spline is used to generate further points. We would generate 50 points in total with 0.02 seconds of time lapse between each point to guide the car in next 1 second.

In Frenet coordiates, I took 5 waypoints to set up the Spline. As we should insure the continuity from the previous path, I took the end point of the previous path and the last second end point as the first two waypoints. Then ahead from the end point, I took 3 points equally spaced with 30 meters on S direction as rest waypoints. Thus a spline was built by interpolating these 5 waypoints.

Once the Spline was built, we could generate the rest path points by feeding into the Spline a set of X values in car's local coordinates. The X values were generated based on reference speed so that the car could drive at the desired velocity. The output were a set of Y values in local coordinates. I then retransformed these X,Y values into global coordinates to append to the path.

### Video

Here the link to the demonstration video on Youtube.

[![Path Planner Demo](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](https://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID_HERE)


## Udacity Instructions

### 1. Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### 2. Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

### 3. Data

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Here is the data provided from the Simulator to the C++ Program

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

### 4. Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### 5. Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### 6. Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

### 7. Dependencies

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