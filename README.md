# Unscented Kalman Filter Project Starter Code

This project utilizes an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurementsand obtaining RMSE values that are lower that the tolerance outlined in the project [rubric](https://review.udacity.com/#!/rubrics/783/view).

[//]: # (Image References)

[image1]: ./images/ukf.png "Both Radar and Lidar sensors"
[image2]: ./images/ukf_radar.png "Radar sensor only"
[image3]: ./images/ukf_lidar.png "Lidar sensor only"
[image4]: ./images/sample_run.png "Sample program run"

## Results

### Comparisson of the different sensors as well as the fused result

In order to validate the Unscented Kalman Filter performance, the filter was applied in three scenarios. While using: both the radar and lidar sensors, only the lidar sensor and only the radar sensor. After comparing the three respective oucomes we validate that the fusion of both the radar and lidar sensors provided the best result.

![alt text][image1]

Simulation with both Radar and Lidar - On: [youtube video](https://youtu.be/KTuKzkEWcRk)

![alt text][image3]

Simulation with only the Lidar - On: [youtube video](https://youtu.be/G2oJpH0RKlA)

![alt text][image2]

Simulation with only the Radar - On: [youtube video](https://youtu.be/zRHq3L4nf1A)

The table below summarizes the results for the three runs. It is clear that sensor fusion provided the best results for all the RMSE values (X, Y, VX, VY):

| Sensor  | X         | Y           | VX        | VY         |
|----------|---------|----------|----------|----------|
| Both     | **0.0890**  | **0.0947**  | **0.3949** |  **0.2078** |
| Lidar     | 0.1090  |  0.1109 | 0.4554 |  0.2679 |
| Radar   | 0.2285  | 0.2227  | 0.4423 |  0.3913 |

### NIS - Normalized Innovation Squared

The program calculates the NIS as can be seen in the following sample run:
![alt text][image4]

The NIS distribution follows the table below with `df = 3` for the radar and `df = 2` for the lidar.

| df  | x^2.950 | x^2.900 | x^2.100 | **x^2.050** |
|------|--------|---------|--------|-----------|
| 2     | 0.103  | 0.211  | 4.605 | **5.991** |
| 3     | 0.352  |  0.584 | 6.251 | **7.815** |

From the sample run we can see that we are slightly underestimating the lidar performance since only 3.212% percent of the measurements exceed the 5% line. However, the radar NIS is 4.819% which is very close to 5%.

### Comparisson with the Extended Kalman Filter results

The results that we obtained using the unscented kalman filter are sligthly better than the results that we got using the [extended kalman filter](https://github.com/itornaza/sdc-extended-kalman-filter) on the same set of data. 

## Installation

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

## Important Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
* On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF`

```
$ mkdir build && cd build
$ cmake .. && make
$ ./UnscentedKF
```

Note: If you want to run the program with different sensor configuration see the `./src/constants.h` file.

## Data flow between the program and the Simulator

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator:

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
