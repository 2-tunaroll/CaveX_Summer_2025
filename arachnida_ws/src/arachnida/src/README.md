## Archnida ROS Package
The arangement of the Arachnida ROS package source code is quite intricate, the following diagram depicts a logical architecture arangement of the Arachnida robot system, which includes the integrated structure of the code.
![systemOperation](https://github.com/CaveX/CaveX2023/assets/110513531/91a0dc3a-8580-40e9-9b34-842472102379)

### lidar

### floam_cpu

### object_detection

### robot_state

### nodelets

### path_planning
The path planning achieves obstacle avoidance and optimal heading through the artificial potential field method proposed by [Khatib 1985](https://ieeexplore.ieee.org/document/1087247). The generic flow chart of th path planning implementation is shown in the image below.
![pathplan_imp](https://github.com/CaveX/CaveX2023/assets/110513531/48f53ad5-9524-42e6-b174-1dc44baffefa)

If the Arachnida workspace has been build, the following command will run the path planning node:

`rosrun arachnida pathPlanNode`

The Artifical Potential Field class uses obstacle data from the object detection and position information from the F-LOAM nodelets to compute the instantaneous direction vectors for optimal heading. If no data is publishes to these topics, the node will not behave as expected. The path planning node uses the direction vector calculated from the artificial potential field and publishes this data to the syropod_remote/desired_velocity ROS topic. To check the node is working as required you can listen to data on the desired velocity topic (if the control method is set to auto). When the control method is set to Joy, data is continously published to every syropod_remote ROS topic this is why separate control methods were setup for effective control. 

`rostopic echo /syropod_remote/desired_velocity`

### gait_energetics
