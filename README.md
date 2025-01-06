# CaveX_Summer_2025
The CaveX project is being continued from end of November 2024 to approximately the end of January 2025. The scope of work to be done is currently being determined. Below is the most recent README content from the 2023 group who worked on the project.

# CaveX2023
All the code for the 2023 iteration of the Arachnida project, formerly known as the CaveX project. This repository is an amalgamation of the repositories used in each of the project iterations (2021, 2022 & 2023). As such, this repository contains all the code needed to operate the Arachnida robot, and simulate it in RViz and Gazebo!
![logo1](https://github.com/CaveX/CaveX2023/assets/110513531/ba4b5694-5d24-4b31-b722-a023e608a9c7)

## Software Development Process
Each of the 2023 team members (Luka Moran, Riley Groome, Tyler Groome) utilised windows PCs with Windows Subsystem for Linux (WSL) version 2 for software development. WSL 2 does however have considerable drawbacks. These include blocking device inputs such as playstation controllers and running linux GUIs (such as RViz and Gazebo) extremely slowly. To overcome these issues the CaveX designated computer, which runs Ubuntu 20.04 was utilised for Gazebo and RViz simulations. Once code was tested in simulation it was brought across to the Jetson processor for physical prototype testing. If an improvement was observed the code was merged into the main branch and permanently built on the Jetson. The following development process is summarised below:

![DT E](https://github.com/CaveX/CaveX2023/assets/110513531/8a3289da-5f09-4349-ab33-c5103d64f9eb)

## First steps
The Jetson runs Ubuntu 20.04, Noetic is the ROS1 distriution which is targeted at this version of Ubuntu. On your personal PC you should have Ubuntu 20.04 installed (If you're on a windows machine you'll need WSL2).
- Follow these [instructions](https://learn.microsoft.com/en-us/windows/wsl/install) for setting up WSL, and upgrading to WSL2.
- Follow the [ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) instructions to install ROS noetic and its dependencies.

The README on the Open Source Syropod Highlevel Controller (OpenSHC) [git page](https://github.com/csiro-robotics/syropod_highlevel_controller) is also worth looking into. This page features a link to OpenSHC tutorials which demonstrate the steps to build the repository and its dependencies for PC's or Raspberry Pi's (or Jetsons!). These steps have already been followed by the 2023 team so the steps to build the code are mentioned in the README in `openshc_ws > src`.

### Dependencies
The following are required dependencies to run the arachnida ROS package, dependencies for OpenSHC and Syropod Remote should be documented on the appropriate CSIRO git page. Note that these dependencies are already installed on the Jeston Orin NX processor but will need to be installed on your personal computer if you wish to run the Arachnida ROS package. 
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page#Download): C++ library which is used for various linear algebra operations, used in SLAM, obstacle detection, and path-planning.
- [Ceres Solver](http://ceres-solver.org/installation.html): Solver which configures transforms between frames for SLAM, also read through the dependencies documented on the attached link. Currently the code is run on Ceres version 2.1.0, please install that for the best compatibility.
- [PCL Library](https://pointclouds.org/downloads/#linux): Point Cloud Library (PCL) for processing and storing points in the cloud.

## Info
Each folder has its own README.md file with explanations on code and directory structure, and how to run the code.

**".vscode" folder**: VS Code has been used for code development throughout this project, this folder contains settings specific to VS Code for code development i.e. for the intellisense (c_cpp_properties.json).

**"arachida_ws" folder**: The Arachnida workspace which contains all the source code for the Arachida ROS package.

**"imu_ws" folder**: Contains the bno-055 driver which publishes data from the IMU chip to ROS topics.

**"launch_app" folder**: Contains service files (launch_services) and shell scripts (launch_scripts) which are located on the Jetson. The service files which are placed in the /lib/systemd/system directory on the Jetson processor and are enabled to run on system bootup. This contains all the startup services for the Jetson to run the services for ROS and OpenSHC. The launch shell scripts are alled by these services and run the commands to start the ROS processes.

**"Miscellaneous" folder**: Contains some of the NeoVim (NVIM) configurations used and the latex source code for our report, should you wish to replicate our formatting style. Also contains four recordings of LiDAR date from the velodyneSocketReader code.

**"Miscellaneous/Sample Velodyne Data" folder**: Contains a pcap file of LiDAR data used for testing the SLAM and obstacle detection alrorithms an extremely useful ROS tool for testing is [ROS bags](https://wiki.ros.org/rosbag/Commandline) which can record data published to topics and 'replay' this data using a simple CLI command. This is extremely useful for testing on personal devices, for example LiDAR data published to ROS topics on the Jetson can be recorded into a .bag file. This can be 'played' on your personal device to simulate the same feed of data on the Jeston from the LiDAR scanner on your personal device. This functionality can also be used for listening to IMU data and servo motor data. 

**"openshc_ws" folder**: All the repositories from the CSIRO to help control the robot, and visualise it using RViz or Gazebo, see the README.md file in `openshc_ws > src` for a description on the packages utilised by the 2023 team. Note that modifications have been made to the CSIRO's syropod_remote repository for integration with dronedeploy and robust dynamic control method switching.

**"websocket_ws"/"arachnida_web" folders**: Contains some of the code used to setup a website which displays live data from the LiDAR sensor, and obstacle detection algorithm. This was used by the 2023 team at Ingenuity, however the code will need to be deployed on a new web sever host, Luka Moran was responsible for this and should be contacted if you wish to display similar functionality at Ingenuity.

## Folder Structure
The folder structure for this repository excluding the openshc_ws and Miscellaneous folders can be seen below. openshc_ws was not included in the below tree as it contained a large number of folders and cluttered up the diagram. Miscellaneous was not included as it does not contain many folders or subdirectories.
```
.
├── arachnida_web
│   ├── arachnida-backend
│   │   └── dist
│   ├── arachnida-backend-nginx
│   │   ├── sites-available
│   │   └── sites-enabled
│   └── arachnida-frontend
│       ├── public
│       └── src
│           ├── app
│           ├── components
│           │   ├── render
│           │   └── theming
│           └── utils
│
├── arachnida_ws
│   └── src
│       └── arachnida
│           ├── include
│           │   ├── arachnida
│           │   │   ├── lidar
│           │   │   ├── path_planning
│           │   │   └── robot_state
│           │   ├── floam_cpu
│           │   └── object_detection_cpu
│           ├── launch
│           ├── msg
│           └── src
│               ├── floam_cpu
│               ├── gait_energetics
│               ├── lidar
│               ├── nodelets
│               ├── object_detection
│               ├── path_planning
│               └── robot_state
│
├── imu_ws
│   └── src
│       └── ros-imu-bno055
│           ├── include
│           │   ├── imu_bno055
│           │   └── watchdog
│           ├── launch
│           └── src
│
├── launch_app
│   ├── launch_scripts
│   └── launch_services
│
└── websocket_ws
    └── src
        └── websocket_interface
            └── src
                └── dist
```
## Useful Resources
|Description        |Link                          |
|----------------|-------------------------------|
|ROS Wiki Documentation|https://wiki.ros.org/Documentation|
|ROS Tutorials|https://wiki.ros.org/ROS/Tutorials|
|Using ROS in WSL2/Creating ROS packages and nodes in C++|[Emil Vidmark (9 videos)](https://www.youtube.com/watch?v=C6BlNbeU3fQ)|
|OpenSHC Git Page|https://github.com/csiro-robotics/syropod_highlevel_controller|
|OpenSHC Paper|https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9223640|
|Syropod Remote Git Page|https://github.com/csiro-robotics/syropod_remote|
|Dynamixel Interface|https://github.com/csiro-robotics/dynamixel_interface|
|DroneDeploy Docs|https://docs-automate.dronedeploy.com/|
