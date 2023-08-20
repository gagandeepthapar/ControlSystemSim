# StarLander 
Independent research in optimal control under stochastic conditions, specifically for trajectory-driven vehicles.

## Context 
This repository contains work for implementing a closed-loop control system to simulate a spacecraft vehicle landing similar to the SpaceX Falcon 9 or the JPL Mars Lander. It was primarily a project to practice C++ and begin research into optimal control under stochastic conditions.

An example of a landing spacecraft (with no thruster control) is plotted below. It uses ideal sensors and the trajectory was generated via spline interpolation such that the initial position and velocity and final position and velocity conditions are met.

![Fail_landing](lander.gif)

## Includes
Requires the `Eigen` C++ library which can be found [here](https://eigen.tuxfamily.org/index.php?title=Main_Page).
Also uses the `Matplot++` C++ library to plot data directly in C++ which can be found [here](https://github.com/alandefreitas/matplotplusplus/tree/master#line-plots). The required custom libraries are submodueld here. 
`CMake` and `make` are used also used to build the program.

## Running 
To run, clone the github in the directory of your choosing via 
```
git clone https://github.com/gagandeepthapar/ControlSystemSim
```


Ensure that the required libraries are installed. My system adds the `Eigen` and `Matplot++` headers directly in my include path so I don't have to explicity call it. You can either add the libraries in the same project directory or where you'd like (e.g., `usr/local/include`) and add them to your include path:
```
$ export CPLUS_INCLUDE_PATH="usr/local/include:$CPLUS_INCLUDE_PATH"
```


Once installed, you should be able to run the following commands in the project directory to setup the build directory and output, respectively:
```
$ cmake -S . -B build
$ make -C build
```


The output file should be called `example.out` which can be renamed directly in the `CMakeLists.txt` file.
