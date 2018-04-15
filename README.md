# CarND-Controls-MPC

This is project 5 of Udacity Self-driving car Nanodegree term 2. The goal is to write a Model Predictive Controller (MPC) to drive a car in the simulator. A sample video of the implemented MPC driving the car can be found [here](./MPC_driving_example.mov).

## Model description

### The states
At each timestamp, there are 6 states used:
* `x`: x position in car coordinate
* `y`: y position in car coordinate
* `psi`: car orientation
* `v`: car speed
* `cte`: error between the actual y position and desired y position
* `epsi`: error between the actual car orientation and desired car orientation

Because all values are in the car coordinate, the initial values of `x, y, psi` should always be 0.

### The actuators
At each timestamp, there are 2 actuators:
* `delta`: the steering angle in [-25, 25 degrees], and
* `a`: the throttle (acceleration) value in [-1, 1]

### The update equations
The states at time `t + 1` depends on the states at time `t` and the actuators at time `t`. The MPC uses the simple kinematic model.
Here are the update equations of the states:
```
x1 = (x0 + v0 * CppAD::cos(psi0) * dt);
y1 = (y0 + v0 * CppAD::sin(psi0) * dt);
psi1 = (psi0 + v0 * delta / Lf * dt);
v1 = (v0 + a * dt);
cte1 = ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
epsi1 = ((psi0 - psides0) + v0 * delta / Lf * dt);
```

### The cost function
The MPC controller finds the optimal actuator values by minimizing a pre-determined cost function. The cost function minimizes the error between the actual and desired positions, orientation, and speed. It also minimizes the use of actuators and the gap between sequential actuator values to make the car drive smoothly. The weighting of different terms are manually tuned.

## Implementation Details

### Determination of N & dt
Since there is a 0.1s delay before sending the actuator values to the car, there is no need to use small dt values. So I only tried to 0.1 and 0.05 for dt. The N value is determined so that the distance `N * dt` is long enough for the car to see and react to a corner (~ 0.6s), and not too long for the optimizer since only the first actuator values will be used. I tried a few different values, both `N = 12, dt = 0.05` and `N = 8, dt = 0.1` would work at reference speed `ref_v = 70`. The final implementation used `N = 12, dt = 0.05`
NOTE: the `idx_delay` value needs to be updated according to `dt` value.

### MPC preprocessing and polynomial fitting
The waypoints need to be preprocessed before passing to the controller. First, the waypoints are given in the map coordinate system, but the car thinks in the car coordinate system, so all waypoints are first translated to the car coordinate system.
Second, the waypoints are a few discrete points, and the controller needs to know the desired y position and psi at any x position. This is done by a 3rd order polynomial fitting of the waypoints, and passing the polynomial coefficients to the controller. The desired y position can then be calculated from the polynomial function and the desired psi is arctan of the polynomial derivative.

### Latency
In this project, the simulator is set to have a 0.1s delay before sending the actuator values to the car. That is to say, at the first 2 timestamps (assuming `dt = 0.05`), the actuator would stay at the current values, and can only change from the 3rd timestamp. To handle this constraint, I passed the current actuator values to the optimizer as the initial state (a additional pair of actuator state values at index 0), and added additional `2 * idx_delay` constraints to make the first `idx_delay` actuator values the same as the initial state. Now the actuator can only be updated after the delay, and the first updated actuator values will be sent to the car.

### Parameter tuning
Similar to the PID controller, the MPC controller also has several parameters that need tuning. But unlike the PID controller, the MPC controller relies less on those parameters and has a much larger tolerance on those parameters. Here I initialized all the values to 1, increased the coefficients for delta because of the the smaller value range of delta, and added more weight to cte error and psi error to confine the car on the desired route. With the current values, the car runs perfectly when speed is 70 mph and can even loop around at 100 mph.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
