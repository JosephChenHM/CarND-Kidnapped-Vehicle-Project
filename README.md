# CarND-Kidnapped-Vehicle-Localization-Project

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project we will implement a 2 dimensional particle filter in C++. Our particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 

Our system successfully pass the accuracy and performance criteria.
1. **Accuracy**: ourr particle filter localize vehicle position and yaw to within the values `max_translation_error` = **0.115** and `max_yaw_error` = **0.104** .

2. **Performance**: our particle filter complete execution within the time of 97 seconds on Intel i5 4200U 2.4GHz.

## Particle Filter System Design
### Particle Filter Algorithm Flowchart
![](https://i.imgur.com/hienv0x.png)
### Implementing the Particle Filter
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

The main particle filter algorithm is implemented at `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods as above flowchart.

* **Initialize Particle Filter** is implemented in line `24 ~ 51` in `particle_filter.cpp`
* **Prediction** is implemented in line `53 ~ 92` in `particle_filter.cpp`
* **Data association** implemented ***nearest neighbor search*** for association between map and observation, locate in line `94 ~ 164` in `particle_filter.cpp`
* **Update Weights** is implemented in line `126 ~ 226` in `particle_filter.cpp`
* **Resample** implemented ***resample wheel*** that effectively improve effeciency in line `228 ~ 258` in `particle_filter.cpp`
* All the **Process Measurement** will be evalutae by **RMSE**(Root-Mean-Square-Error).
## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./particle_filter`

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. `./clean.sh`
2. `./build.sh`
3. `./run.sh`

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.



