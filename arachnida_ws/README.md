## Arachnida Workspace

The Arachnida workspace contains all the code for the Arachnida ROS package, this includes:
- Fast LiDAR Odometry and Mapping (F-LOAM) SLAM algorithm
- Gait Energetics
- LiDAR
- Obstacle/Object detection
- Path Planning
- Robot State

Navigate to the `src/arachnida/src/README.md` file for further explanations on these features.

## Arachnida Workspace Files and Folders
The file structure for the arachnida workspace can be seen in the section below. In the main folder it only contains this README.md file and src folder. Within the src folder lies the CMakeLists.txt file used by the catkin workspace and an arachnida folder. I believe this is a general CMakeLists.txt file and is not customised to the project and code like the one located in the arachnida folder. Within the arachnida folder lies a customised CMakeLists.txt document used to build and associate c++ code with the relevant ros node. Also in this folder lies two xml files, one is for establishing information for the arachnida package as a whole, the other is setting plugins for the various nodelets used within this package. 

The arachnida folder contains 4 subfolders, these being "include", "launch", "msg", and "src". The include folder contains all the created ".h" files needed for this package to be used in the relevant ".cpp" files. This folder has been divided into components as listed in the previous section. The src file contains all the ".cpp" files used for this package, these have also been split up into folders like the include folder. It also includes an additional folder for the ros nodelets. The launch folder contains a launch file for starting and initialising the code. Lastly, the msg folder contains two ".msg" files determining the structure of the obstacle and obstacle list messages sent between certain ros nodes.

It is also worth mentioning that when the following two lines appear: 
```
#include "arachnida/Obstacle.h"
#include "arachnida/ObstacleList.h"
```
It is possible that vscode will give an error saying these files cannot be found. To fix this, try calling catkin_make and sourcing the devel/setup.bash file. These files are created when catkin_make is called and are within the devel directory. They are based upon the format of the ".msg" files in the msg folder.


### Arachnida Workspace File Structure
```
.
├── src
│   ├── arachnida
│   │   ├── include
│   │   │   ├── floam_cpu
│   │   │   │   ├── laserMappingClass.h
│   │   │   │   ├── laserProcessingClass.h
│   │   │   │   ├── lidarOptimisation.h
│   │   │   │   └── odomEstimationClass.h
│   │   │   ├── gait_energetics
│   │   │   │   └── gait_energetics.h
│   │   │   ├── lidar
│   │   │   │   ├── velodynePCAPReader.h
│   │   │   │   ├── velodyneSocketReader.h
│   │   │   │   └── velodyneUtils.h
│   │   │   ├── object_detection_cpu
│   │   │   │   ├── objBox.h
│   │   │   │   ├── objCluster.h
│   │   │   │   ├── objKdtree.h
│   │   │   │   ├── objPointCloudProcessor.h
│   │   │   │   ├── objRansac.h
│   │   │   │   └── objRender.h
│   │   │   ├── path_planning
│   │   │   │   ├── artificialPotentialField.h
│   │   │   │   └── pathPlanningNode.h
│   │   │   ├── robot_state
│   │   │   │   └── extendedKalmanFilter.h
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
## Build Arachnida Workspace
If you have each of the dependencies installed mentioned on the Home [ReadME](https://github.com/CaveX/CaveX2023), you can build the Arachnida ROS package using:

`catkin_make`
