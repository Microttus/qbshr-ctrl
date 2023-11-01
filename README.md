# qbshr control

**ROS2 interface for the qbRobotics Soft Hand Research**

## Introduction

This package aims to create a ROS2 interface for control of the qbRobotics Soft Hand Research.
The qbrobotics devices has native ROS1 support but no official ROS2 support.
There are a native API available with the necessary control commands for the qb devices,
and this package utilize this to export the commands for ROS2 support.

### Credits

This was made as a result of the need for ROS2 support enabling the use of the hand
for my robotics project at University of Agder Autumn 2023. In the process I 
received help from multiple persons especially [@kristianmk](https://github.com/kristianmk/).

### Maintenance

Due to the nature of this being a school project the continuous maintenance of this
package is not guarantied. That being said some work and improvements will be available
for the next year as I continue to work with this device for new projects. 

There are two main know issues as the 1.0.0 version are created. There are a continuous
problem with segmentation errors occurring especially while connection to the hand.
The API requires some time connecting properly. Restarting the program usually solves
the problem while a solution are investigated. 

The second problem is due to the Serial library provided by the qbrobotics device API.
This library is not compiled correctly for transport, leading to the api library first 
have to be built in the selected folder, then the specific path to the includes and 
the cmake file set. Guide to this is explained during installation chapter.

## Installation

### Step 1 - Clone the qbshr_ctr package 

First step clone the package into the src folder of your ROS2 workspace

Create workspace
```commandline
mkdir wsROS
cd wsROS
mkdir src
cd src
```

Clone repository
```commandline
https://github.com/Microttus/qbshr_ctr.git
```

### Step 2 - Get qbrobotics API and build

Step two clone the qbrobotics API into the lib folder of the repository.

```commandline
cd qbshr_ctr/libs
git clone --recurse-submodules https://bitbucket.org/qbrobotics/qbdevice-api-7.x.x.git
```

Build the library for "activation"

```commandline
cd qbdevice-api-7.x.x
mkdir build
cd build
cmake ..
make
```

### Step 3 (Optional) - Build package

The package can be built and tested separately

```commandline
cd ../../../..
colcon build --packages-select qbshr_ctr
. install/setup.bash
```

One test program is available

```commandline
ros2 run qbshr_ctr qbshr_test
```

### Step 4 - Include in other packages

For inclusion in yor own ros2 packages add these lines for your cmake

```cmake
include_directories(include ../qbshr_ctr/include/qbshr_ctr ../qbshr_ctr/libs/qbdevice-api-7.x.x/serial/include ../qbshr_ctr/libs/qbdevice-api-7.x.x/qbrobotics-driver/libs/research/include)

get_filename_component(API_PATH_uq ../qbshr_ctr/libs ABSOLUTE)
if(EXISTS "${API_PATH_uq}/qbdevice-api-7.x.x/serial/CMakeLists.txt")
    message("-- [ ${PROJECT_NAME}] Serial folder found")
endif()
SET(CMAKE_PREFIX_PATH ${API_PATH_uq}/qbdevice-api-7.x.x/serial)
```

The library can be included with

```c++
#include "qbshr_ctr/qbSoftHandHandler.hh"
#include "qbshr_ctr/qbSoftHandControl.hh"
```

## 🌳Files tree layout

```text
├── include                             : Include folder for headers
│   ├── qbshr_ctr                       : Folder for specific package header
│       ├── qbSoftHandControl.hh        : Control class header
│       └── qbSoftHandHandler.hh        : Serial handler header
├── libs                                : Folder for external lib files
│   ├── (qbdevice-api-7.x.x)            : Must be cloned from source
│   └── README.md                       : A short reminder
├── src                                 : Source file folder
│   ├── qbSoftHandControl.cc            : Source file for control class
│   ├── qbSoftHandHandler.cc            : Source file for handler class
│   ├── robotHandControl.cc             : Example script for control
│   └── simpTest.cc                     : Basic test script for debug
├── .gitignore                          : Git repository file
├── CMakeList.txt                       : Main CMake file for building package
├── package.xml                         : Package file for ROS2
└── README.md                           : Main instructions 
```


