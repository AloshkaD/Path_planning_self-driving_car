# Path Planning in C++
![Alt text](images/1-D3VPNvW9Nd52u9IrvQqyzA.png?raw=true "Main")
In this project I'm taking the trajectories of cars on the road and generate a safe and a smooth path for a simulated car. I'm demonstrating the program by driving the car within the acceptable 50MPH speed limit and with jerk minimization techniques in place. 

### Details
The sensor fusion data is provided by the simulator, which are used to avoid accidents and plan for overtaking the other cars on the same road.  
A sparse map list of waypoints around the highway is also provided. The car receives (x,y) coordinates that are each 0.02 seconds apart. Hence, I controlled the car velocity by incrementing 0.224 mph in 0.02 sec time frame to eliminate longitudinal jerks. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position. The path is smoothed using the spline library to create an approximation of the path. 

I used Frenet coordinates because it is easier to determine whether there are other cars on the same lane than cartesian coordinates. 
![Alt text](images/1-p3Qb5sCKYUGvkRnPOW-Gpg.png?raw=true "Frenet")   
The s value in fernet denote the distance and the d value denote the lane. I used the d value to determine the occupancy of my car lane and nearby lanes. In some rare situation, the occupancy was falsely determined due to inaccuracies in converting between Fernet and Cartesian coordinates.

After the lane occupancy is determined I've planned for two scenarios.
1- if all lanes are busy the car deaccelerate at a rate of -0.224 mph. The car waits for the first free lane and changes its lane

2- if the car's lane is busy and there is a free lane the car changes lanes.

I have not used FSM or cost functions in solving this part of path planning because of lack of time but I'm planning to do it after I finish the final project. 


### Results
[![IMAGE ALT TEXT](http://img.youtube.com/vi/7AMou8C_T80/0.jpg)](http://www.youtube.com/watch?v=7AMou8C_T80 "Simulation")

 
The shown simulator consist of a 6946m highway loop.
The simulator can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

We can see that we've achieved our goal by safely navigating around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car goes to as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, while other cars will try to change lanes too. Since the car is trying to go 50 MPH, it takes a little over 5 minutes to complete 1 loop. The recorded simulation provides only shows part of the track. 
The car remained at total acceleration below 10 m/s^2 and jerk that is below 50 m/s^3 at all times.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

 

## Credit

The most useful resource was the course materials and lectueres for path planning on Udacity and this really helpful  [project walkthrough](https://www.youtube.com/watch?v=3QP3hJHm4WM)  
Also, for smoothing the path the [Spline library](http://kluge.in-chemnitz.de/opensource/spline/) was really helpful.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo. in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

