## Arachnida Workspace

The Arachnida workspace contains all the code for the Arachnida ROS package, this includes:
- Fast LiDAR Odometry and Mapping (F-LOAM) SLAM algorithm
- Obstacle detection
- Path-planning
- Gait Energetics

Navigate to the `src/arachnida/src/README.md` file for further explanations on these features.

## Arachnida Workspace folder structure
```
.
├── src
│   ├── arachnida
│   │   ├── include
│   │   │   ├── arachnida
│   │   │   │   ├── lidar
│   │   │   │   │   ├── velodynePCAPReader.h
│   │   │   │   │   ├── velodyneSocketReader.h
│   │   │   │   │   └── velodyneUtils.h
│   │   │   │   ├── path_planning
│   │   │   │   │   ├── artificialPotentialField.h
│   │   │   │   │   └── pathPlanningNode.h
│   │   │   │   └── robot_state
│   │   │   │       └── extendedKalmanFilter.h
│   │   │   ├── floam_cpu
│   │   │   │   ├── laserMappingClass.h
│   │   │   │   ├── laserProcessingClass.h
│   │   │   │   ├── lidarOptimisation.h
│   │   │   │   └── odomEstimationClass.h
│   │   │   ├── object_detection_cpu
│   │   │   │   ├── objBox.h
│   │   │   │   ├── objCluster.h
│   │   │   │   ├── objKdtree.h
│   │   │   │   ├── objPointCloudProcessor.h
│   │   │   │   ├── objRansac.h
│   │   │   │   └── objRender.h
│   │   │   └── utils.h
│   │   ├── launch
│   │   │   └── arachnida_main.launch
│   │   ├── msg
│   │   │   ├── ObstacleList.msg
│   │   │   └── Obstacle.msg
│   │   ├── src
│   │   │   ├── floam_cpu
│   │   │   │   ├── laserMappingClass.cpp
│   │   │   │   ├── laserProcessingClass.cpp
│   │   │   │   ├── laserProcessingNode.cpp
│   │   │   │   ├── lidarOptimisation.cpp
│   │   │   │   ├── odomEstimationClass.cpp
│   │   │   │   └── odomEstimationNode.cpp
│   │   │   ├── gait_energetics
│   │   │   │   ├── gait_energetics.cpp
│   │   │   │   ├── gait_energetics.h
│   │   │   │   └── main.cpp
│   │   │   ├── lidar
│   │   │   │   ├── velodynePCAPReader.cpp
│   │   │   │   ├── velodyneReaderNode.cpp
│   │   │   │   ├── velodyneSocketReader.cpp
│   │   │   │   └── velodyneUtils.cpp
│   │   │   ├── nodelets
│   │   │   │   ├── floamNodelet.cpp
│   │   │   │   ├── objectDetectionNodelet.cpp
│   │   │   │   └── robotStateNodelet.cpp
│   │   │   ├── object_detection
│   │   │   │   ├── objCluster.cpp
│   │   │   │   ├── objPointCloudProcessor.cpp
│   │   │   │   ├── objRansac.cpp
│   │   │   │   └── objRender.cpp
│   │   │   ├── path_planning
│   │   │   │   ├── artificialPotentialField.cpp
│   │   │   │   └── pathPlanningNode.cpp
│   │   │   ├── robot_state
│   │   │   │   └── extendedKalmanFilter.cpp
│   │   │   ├── hello_world.cpp
│   │   │   ├── publisher_sample_node.cpp
│   │   │   ├── README.md
│   │   │   └── subscriber_sample_node.cpp
│   │   ├── CMakeLists.txt
│   │   ├── nodelets_plugins.xml
│   │   └── package.xml
│   └── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
└── README.md
```
### Build Arachnida Workspace
If you have each of the dependencies installed mentioned on the Home [ReadME](https://github.com/CaveX/CaveX2023), you can build the Arachnida ROS package using:

`catkin_make`
