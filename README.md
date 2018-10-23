# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
This file is updated based on the assignment submission. The section - *Reflection* contains my comments.

[//]: # (Image References)
[image1]: ./planpipe.png
[image2]: ./wpgen.png
[image3]: ./14milerun.png

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Reflection
I approached the exercise based on lessons and project Q & A session to begin with. I also wanted to make the exercise less math intensive more intuitive, with pirority to safety, especially when lane changing. 
Path Planning consists of various functional modules including *prediction, behaviour planning, and trajectory generation.* as depected in the following picture taken from Lesson section, 4-01.
![alt text][image1]

### Prediction
The *Prediction* component takes other vehicles position, velocity and other information and estimates future position in the next frame.
The following code located in *main.cpp:[line #350]* accomplishes this computation.
`                a_car_s += prev_path_size * 0.02 * a_car_speed;`

In the implementation the information about every other car with respect to ego car.  The relational information includes the longitudinal position (**s**), lateral position (**d**), and vehicle speed components (**vx**, **vy**). These inputs are distilled into immediate surroundings / neighborhood information of ego vehicle.  The neighborhood information is stored in *lane_info* data structure, in terms of the following in each lane.
1. The distance estimate between the nearest vehicle in the back and ego vehicle.
2. The speed difference between the nearest vehicle in the back and ego vehicle.
3. The distance estimate between the nearest vehicle in the front and ego vehicle.
4. The speed difference between the nearest vehicle in the front and ego vehicle.
The relevant code is located in the vicinity of *main.cpp:[line #s 332 - 382].*
This neighborhood information is the crucial decison enabler for lane switching as well as speed control.

Two helper functions are used for prediction and behavior planning.
#### get_lane()
The function *get_lane()* return lane number by taking lateral distance (**d**) as input. This function is located in the vicinity of *main.cpp:[line #s 171 - 181]*.

#### get_cost()
This function returns cost associated with a lane change based on the relative speeds, and position estimates of the immediate vehicle in the front and in the back. This function is located in the vicinity of *main.cpp:[line #s 186 - 206]*.  
The cost is determined by how much cushion is available for ego car to switch into an adjacent lane. If there is relatively long space in the front as well as in the back of ego vehicles and the front vehicle faster than ego vehicle and the back vehicle in the lane is moving slower than ego vehicle, then the cost of switching is lesser. 

### Behavior Planning
The intuitive logic of *get_cost()* is useful in choosing left or right lane, when the ego vehicle is in the middle lane or continue in the middle lane. When ego vehicle is in one of edge lanes, the cost can be used to decide whether switch to middle lane or stay put in the current lane.  
The cost information is used in conjunction with the relative distance between front car and the ego vehicle in the same lane. The relevant logic is located in the vicinity of *main.cpp:[line #s 411 - 488]*. The logic also handles the smooth lane switch transition by tracking lane switch initiation and lane switch complete.

Another important aspect of behavior planning is speed control and to maintain safe distance in the front. I chose the safe distance between immediate front vehicle and the ego vehicle is 50 meters. If it is less than 50 meters initiate slow down.  If there is no vehicle within next 55 meters then improve the speed if possible within the speed limit.  The relevant logic is coded in the vicinity of *main.cpp:[line #s 490 - 541]*. The logic uses step-wise speed changes within the constraints of acceleration, jerk and speed limits.

### Trajectory Generation
We got help from project Q & A session on this section.  I followed the guidance and spline function with proposed lane and proposed speed as inputs. The guidance was very helpful in coordinate conversion from global to local and vice versa and next way point generation. The relevant logic is coded in the vicinity of *main.cpp:[line #s 544 - 644]*. 

The trajectory generation logic starts with using any left-over points in the previous trajectory. If previous points are less than 2, which would be the case in the beginning the previous points are generated from car position and yaw (ref: *main.cpp [line #s 548 – 576]*).

Three high level way points are generated using helper function getXY() and using map information as inputs. (ref: *main.cpp [line #s 548 – 576]*).

By using proposed speed, next fine grained x-points are computed by using spline function over next x-distance as 30. Next y-points on spline are computed by feeding x-points to spline function. The math was nicely explained in Q&A session, using the following picture. These computed x-points and corresponding y-points on the spline are converted to global coordinates and fed to simulator as next-x and next-y values.

(ref: *main.cpp [line #s 617 – 644]*).

![alt text][image2]


### Results
The resulting car behavior was robust in terms of changing lanes, maintaining safe distance and behaving within the constraints of jerk and acceleration. The following snapshot is a sample run. 
![alt text][image3]

Here is the link for the corresponding video. https://youtu.be/Yq9Z0RSz1P0


### Further Thoughts
Further improvements are possible in the cost function are tracking acceleration and jerk in absolute values. 
I noticed difference in acceleration calculation between my calculation and simulator calculations, when I calculated acceleration as an experiment.
I also noticed there is considerable difference in speed proposed (e.g. 49.9 mph) and speed computed by simulator (49.7 mph)even after steady state condition is reached. 

Sometimes, simulator behavior is adversarial in the sense that the fellow cars randomly jump in front of ego car, within too short distance to not to violate jerk and acceleration constraints.  Here is the video link regarding such an instance. https://youtu.be/0D_7OBvxGJc




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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

