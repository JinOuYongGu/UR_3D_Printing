# UR Additive Manufacturing
Developed for **Universal Robot** base on **ROS**. Enable an UR3 robot to 3d printing parts with generic 3-axis or multi-axis way.

# Recommend Environment
OS: Ubuntu 18.04
ROS: Melodic
IDE: VScode

# Dependency & Building
## Dependency
### robot driver

    # switch to workspace src folder
    cd ~
    cd catkin_ws/src

    # clone the robot driver
    git clone https://github.com/JinOuYongGu/Universal_Robots_ROS_Driver.git

The repo is a fork from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
You can also clone the official one.
For more installation guide about the driver, see this: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#building

### serial library
    sudo apt-get install ros-melodic-serial

### visual tools library
    sudo apt-get install ros-melodic-moveit-visual-tools

You may need to reboot your PC after install these libraries.

## Building
After installing the ur_robot_driver, download the code of this repo and build it.

    cd ~
    cd catkin_ws/src
    git clone https://github.com/JinOuYongGu/UR_3D_Printing.git

Then your src folder should look like this:
- universal_robot
- Universal_Robots_ROS_Driver
- UR_Additive_Manufacturing

Now you can switch to catkin_ws folder and build the project.

    cd ..
    catkin_make
