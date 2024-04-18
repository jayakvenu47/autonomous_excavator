## Introduction - Autonomous Excavator

This project is an attempt to automate the arm of a Mini-Excavator using Robot Operating System(ROS) with the help of MoveIt Library to do motion planning between two given points.

The URDF is developed for simulation purposes from CAD model using CAD to URDF coversion utility. 
The project uses the angle to length(cylinder extension) mapping to work around the closed loop kinematic chains associated hydraulic mechanisms present in the excavator. The planned waypoints given by MoveIt Library is executed using a closed loop control (PID control) interfacing it with LabView installed in another computer on a WIndows OS due to practicalities associated with legacy motor control systems.
The communication between the two computers(Linux and Windows) is done through Network Sockets based on the TCP/IP Network environment.

In addition to ROS automation, a few LabView applications are employed for joystick-based manual excavator control.




## Softwares/Platforms
SOLIDWORKS 2021, Creo Parametric 6.0.20

LabView 2019

PyCharm/VSC

Python 3.10.5

PyQt5

ROS Noetic Ninjemys/Gazebo/RViz
Linux Ubuntu 20.04

### For ROS Simulation
Linux Ubuntu 20.04

ROS Noetic Ninjemys/Gazebo/RViz



## Installation
Clone the repo
    ```sh
    git clone https://github.com/AarniAhvonen/autonomousexcavator.git
    ```

## ROS (simulation only)
### Install ROS
The version of ROS we used is Noetic, and the operation system we used is Linux Ubuntu 20.04.

To install ROS on your local workstation, please access this page on ROS wiki:

http://wiki.ros.org/noetic/Installation/Ubuntu
 
or follow expand the section below
<details>
  <summary>Instructions for installing ros</summary>
  
  #### Setting up your computer to accept packages from ros.org
   ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```
    
  #### Set up your keys
   ```
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```
   #### Update debian packages
   ```
    sudo apt update
   ```
  #### ROS desktop full install
   ```
    sudo apt install ros-noetic-desktop-full
   ```
  #### Environment Setup
   ```
    source /opt/ros/noetic/setup.bash
   ```
    
  #### Automatically source this script every time you launch a shell
   ```
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
   ```
</details>


### Install moveit
Install moveit and setup it according to the official tutorial:

https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-ros-and-catkin

or follow the instructions below

<details>
  <summary>Instructions for installing MoveIt and creating a workspace</summary>
  
  #### Update all the packages
   ```
    rosdep update
    sudo apt update
    sudo apt dist-upgrade
   ```
    
  #### Install catkin the ROS build system
   ```
    sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon-
   ```
   #### Install wstool
   ```
    sudo apt install python3-wstool
   ```
  #### create a catkin worksapce and download MoveIt package
   ```
    mkdir -p ~/ws_moveit/src
    cd ~/ws_moveit/src

    wstool init .
    wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
    wstool remove  moveit_tutorials
    wstool update -t
   ```
  #### Install dependencies
   ```
    rosdep install -y --from-paths . --ignore-src --rosdistro noetic
   ```
    
  #### Build the catkin workspace
   ```
    cd ~/ws_moveit
    catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin build
   ```
  #### Source the catkin workspace
   ```
    source ~/ws_moveit/devel/setup.bash
    
   ```
  ###### Optional : Add the previous command to your workspace
   ```
   echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
   ```
</details>

### Download related packages
1. Go the repo
    ```
    cd $Your_repository_path
    ```
2. Copy the packages:
    ```
    cp -rf jcb_description $Your_ros_workstation_src_directory
    cp -rf jcb_moveit_config $Your_ros_workstation_src_directory
    ```
3. Build the packages:
    ```
    roscd && cd ..
    catkin build / catkin_make
    source devel/setup.bash(depending on your shell setting, commonly could be sh, bash, or zsh)
    ```

### Run model in the world of simulation
Run the model:

    roslaunch jcb_moveit_config demo_gazebo.launch

### Run interface (planning path with specific positions)
Open up the interface:

    roscd jcb_moveit_config
    cd scripts
    python3 excavatorprogram2.py

A interface like this should be seen:

![Interface for planning the path][interface-screenshot]

Input the values:

    Example values: -x -1.6 -y 0 -z 1.2 -pitch -2 -roll 0 -yaw 0

Click "Send", and check the result

### Planning path with Rviz interface
A interface like this in the view of Rviz should be seen:

![Interface for planning the path][rviz-screenshot]

The is a module called "MotionPlanning". After setting up the goal state, one can plan and execute 
the planned path via pressing the button "plan" and "execute".

The simulated actions could also be seen in the view of Gazebo after excuting the planned path.


## ROS (with excavator)

### Install ROS
The version of ROS we used is Noetic, and the operation system we used is Linux Ubuntu 20.04.

To install ROS on your local workstation, please access this page on ROS wiki:

http://wiki.ros.org/noetic/Installation/Ubuntu

### Install moveit
Install moveit and setup it according to the official tutorial:

https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-ros-and-catkin

### Download related packages
1. Go the repo
    ```
    cd $Your_repository_path
    ```
2. Copy the packages:
    ```
    cp -rf jcb_description $Your_ros_workstation_src_directory
    cp -rf jcb_moveit_config $Your_ros_workstation_src_directory
    ```
3. Build the packages:
    ```
    roscd && cd ..
    catkin build / catkin_make
    source devel/setup.bash(depending on your shell setting, commonly could be sh, bash, or zsh)
    ```

### Run model in the world of simulation
Run the model:

    roslaunch jcb_moveit_config demo_gazebp.launch

### Run interface
Open up the interface:

    roscd jcb_moveit_config
    cd scripts
    python3 excavatorprogram2.py

A interface like this should be seen:

![Interface for planning the path][interface-screenshot]

Input the values:

    Example values: -x -1.6 -y 0 -z 1.2 -pitch -2 -roll 0 -yaw 0

### Let the robot in simulation world goes to the same position 

3. Run python code
    ```
    roscd jcb_moveit_config
    cd scripts
    python3 excavatorprogram2.py

    ```
4. Open a new terminal
    ```
    roscd jcb_moveit_config
    cd scripts
    python3 refractor.py
    ```
5. Plan path with the interface

    Wait for the robot in the simulation world moves to the positoin in real world, and the "Client Connected" appears in the terminal that run refractor.py

    Click the button of "send" in interface window.

[interface-screenshot]: images/interface.png
[rviz-screenshot]: images/excavator_rviz.png

## CAD/URDF
Previous CAD models created in Creo were used in the assembly of the excavator. URDF files were generated with simplyfying the assembly and utilizing SolidWorks to URDF exporter which can be found here:
http://wiki.ros.org/sw_urdf_exporter

## LabView

The LabView version used in this project is LabView 2019. The Python nodes used in this project are only available in LabView 2018 and never versions. LabView can be downloaded here: 

https://www.ni.com/fi-fi/support/downloads/software-products/download.labview.html#443865

In order to control the Bosch Rexroth motors, an EAL4LabView Toolkit by Bosch Rexroth is needed. EAL4LabView can be downloaded here:

https://www.vipm.io/package/eal4labview/

In order to download the toolkit, the Visual Instruments Package Manager (VIPM) should be downloaded:

https://www.vipm.io/download/

### Programs

For a single target position, run:
```
digitally_ros_without_joystick_single_target.vi
```

For consecutive target positions, run:
```
digitally_ros_without_Joystick_multiple_targets.vi
```

For manual control using joysticks, run
```
two_joysticks_without_sensors.vi
```
