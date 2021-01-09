# Project: Kidnapped Vehicle - Particle Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

[//]: # (Image References)
[flowchart]: ./writeup_images/flow-chart.png "flowchart"
[psuedo]: ./writeup_images/psuedo.png "psuedo"
[init]: ./writeup_images/init.png "init"
[prediction_0]: ./writeup_images/prediction_0.png "prediction_0"
[prediction_1]: ./writeup_images/prediction_1.png "prediction_1"
[transform]: ./writeup_images/transform.png "transform"
[nearest]: ./writeup_images/nearest.png "nearest"
[multivariate]: ./writeup_images/multivariate.png "multivariate"
[pf_1]: ./writeup_images/pf_1.gif "pf_1"

## Project Introduction

Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Particle filter

### Flow Overview

![alt text][flowchart]

### Psuedo Code

![alt text][psuedo]


### Initialization

The number of particles used for this project is 1000, it's chosen after a limited experimentation from 125, 500, 1000 and 10000.
The 1000 seems to produce least errors with acceptable running time.

Each of the particle is then initialized with GPS coordinate using random gaussian distribution.

![alt text][init]

### Prediction

![alt text][prediction_0]
![alt text][prediction_1]

### Weight Update

The first step is to use the particle coordinates and heading to transform the car's frame of reference to the map's frame of reference.

![alt text][transform]

Then each of the transformed observation is associated with nearest landmark. For example using the following example
the transformed coordinates for each observation as such TOBS1 = (6,3), TOBS2 = (2,2) and TOBS3 = (0,5).

![alt text][nearest]

Using the euclidean distance, for instance OBS2 and OBS3 are associated L2 because has the lowest distance.

Finally to update the particle weight, the Multivariate Gaussian probability density is used.

![alt text][multivariate]

x and y are the observations in map coordinates, μx and μy are the coordinates of the nearest landmarks and
the σx and σy are the landmark measurement uncertainty.

### Resampling

For final step, the particles are resampled proportion to its weight. 
After the resample, The new set of particles represents the Bayes filter posterior probability. 
We now have a refined estimate of the vehicles position based on input evidence.

### Simulation

![alt text][pf_1]



## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


# Repository 
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```


## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.
