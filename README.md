# Extended Kalman Filter Project

In this project a Kalman Kilter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements. RMSE values are lower than a specified tolerance.


1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


# Installing and Using Eigen in Your Project

## System-Wide Installation on Ubuntu 22.04

1. Update package list:
   ```
   sudo apt update
   ```

2. Install Eigen3:
   ```
   sudo apt install libeigen3-dev
   ```

3. Verify installation:
   ```
   pkg-config --modversion eigen3
   ```

Eigen headers are installed in `/usr/include/eigen3/`.

## Using Eigen in Your Project

1. Include Eigen headers in your C++ files:
   ```cpp
   #include <Eigen/Dense>
   ```

2. When compiling, add this flag:
   ```
   -I/usr/include/eigen3
   ```

3. If using CMake, add to your CMakeLists.txt:
   ```cmake
   find_package(Eigen3 3.3 REQUIRED NO_MODULE)
   target_link_libraries(your_target_name Eigen3::Eigen)
   ```

After following these steps, Eigen will be available system-wide, eliminating the need to include it in your source directory.

## Updating Your Current CMakeLists.txt

To integrate Eigen into your existing project, update your CMakeLists.txt:
