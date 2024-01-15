This folder contains all of the packages to run OpenSHC on the CaveX robot. You should have a working Ubuntu 20.04 environment on your PC with ROS noetic installed you can check this with the following bash shell command:

`printenv | grep ROS`

You should see that relevant ROS environment variables are set similar to below:

![image](https://github.com/CaveX/CaveX2023/assets/110513531/7b53529a-c058-436b-9fb8-12e9dfe2ae9b)

If the above environment variables are not set, type the following line into the terminal (this should also be added to your `~/.bashrc` configuration file.

`source /opt/ros/noetic/setup.bash`

The following command is also required prior to building the CaveX codebase:

`sudo apt install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers python3-catkin-tools`

Ensuring that you are in the `~/CaveX2023/openshc_ws` directory run the following command:

`catkin_make`

This may take awhile, if you get an internal compiler error this is likely because your WSL server (if you are on windows) is using too much memory and your operating system kills the build process. Try isolating the build to a single core using the following command:

`catki_make -j1`

Once the build is complete you will need to source the bash setup script in the development space before launching any ROS packages to reconfigure the changes:

`source devel/setup.bash`

You can also put this command into your `~/.bashrc` file for these changes to take effect each time you open a new shell:

`echo "source ~/CaveX2023/openshc_ws/devel/setup.bash" >> ~/.bashrc`

Ensuring that the correct filepath is specified. This was how it was setup on Tyler's PC.

You have now setup all the code necessary to run the robot prototype and to simulate the robot in Rviz and Gazebo. Noting that simulation is all that can be achieved on a PC.

# Folder Description
**frankenX_syropod**

The frankenX_syropod package is useful for simulating robots running OpenSHC in RViz, see this [link](https://github.com/csiro-robotics/frankenX_syropod/tree/master).
It is conventionally used by the CSIRO for simulation of the FrankenX syropod, however, the *hexapod21.yaml* `(location: frankenX_syropod > config > hexapod21.yaml)` configuration file can be used to simulate the caveX robot in RViz.
This package does not yet contain a gazebo launch file.

To run the Rviz simulation with the caveX robot run the following, note that frankenX_syropod has no gazebo launch file hence the group utilised the bullet_syropod package for gazebo simulations.

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod21`

To move the robot, a control input is required. If you are running the above command on a linux OS, such as the Jetson, a USB connected playstation controller works well for robot control. However if you are running this command on a Windows machine with an embedded linux OS via Windows Subsystem for Linux (WSL) then using a usb control is likely not to work. WSL 2 no longer supports USB inputs, an alternative is to use the in-built rqt control window. This can interface with openSHC via the *syropod_rqt_reconfigure_control* package with is included in this workspace. Hence to launch the RViz simulation of the CaveX robot on WSL the following command  is recommended:

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod21 control:=rqt`

Note that WSL also does not support linux Graphical User Interfaces (GUIs) including those integrated with ROS, you should look at X11 forwarding for ROS applications if this command does not produce an RViz window.

**DynamixelSDK**

This Package is a dependency of the Dynamixel_interface package which is responsible for configuring the Dynamixel motors with OpenSHC and the require ROS topics.

**Dynamixel_interface**

Refer to the README on the CSIRO [git page](https://github.com/csiro-robotics/dynamixel_interface).

**hexapod22**

The hexapod22 package launches the dynamixel interface package which sets up the dynamixel motors on the Arachnida robot. This package is not used for developers just simulating the robot in Gazebo or RViz. Note that Gazebo and RViz can also be used in real time whilst the robot prototype is running.

**syropod_highlevel_controller**

This code repository was created by the CSIRO and is responsible for generating gaits and poses for multilegged robots, see this [link](https://github.com/csiro-robotics/syropod_highlevel_controller).

**syropod_remote**

This repository acts as the interface between user input devices and the *syropod_highlevel_controller*, see this [link](https://github.com/csiro-robotics/syropod_remote). Note modifications have been made to this repository for integration with dronedeploy and dynamic control method switching.

**syropod_rqt_reconfigure_control**

As stated in the description of the *frankenX_syropod*, the *syropod_rqt_reconfigure_control* repository contains code which supplies an rqt GUI which can interface with OpenSHC for control, syropod_remote can therefore not be running whilst this is in used. See this [link](https://github.com/csiro-robotics/syropod_rqt_reconfigure_control) for more information.

The rqt control method is generally set in a *roslaunch* command by the following syntax, note that the default value for control is joy (joystick):

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod21 control:=rqt`

**bullet_syropod**

The bullet syropod package is the package the 2023 team used to run gazebo simulations. This is configured for the CSIRO's [bullet syropod robot](https://github.com/csiro-robotics/bullet_syropod) but was utilised for testing and debugging code as this code also calls the syropod_highlevel_controller package like the CaveX robot code does. The 2021 team tried to setup a gazebo simulation package with the CaveX robot CAD files but they could not get it to work refer to this [link](https://github.com/CaveX/cavex_hexapod). Custom Gazebo models were made to simulate rough terrain to ensure that gazebo has access to these you will need to run the following commands:

`source /usr/share/gazebo-11/setup.bash`

`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/CaveX2023/openshc_ws/src/bullet_syropod/models`

Ensuring the correct file path is specified for the CaveX2023 repository. To launch the Gazebo simulation of the bullet syropod run the following, noting that a PC running linux, rather than WSL is prefered:

`roslaunch bullet_syropod bullet_highlevel.launch rviz:=false gazebo:=true`

Navigate to the insert tab when gazebo opens and the models should appear clicking on the models and then into the display window will allow the models to be imported into the simulation environment. Note that this may take awhile to load.
