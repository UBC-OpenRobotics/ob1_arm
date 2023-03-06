# Setup Instructions for ob1_arm repo
Step by step guide to setup and troubleshoot this repo in order to get all the ob1 arm control and simulation code running properly. Follow each section and make sure to checkout the troubleshooting section if you run into unexpected issues.

**Author: Yousif El-Wishahy (yel.wishahy@gmail.com)**

## 0. Before you start
Make sure you're running an **Ubuntu 20.04 (codename: focal)** installation. ROS Noetic only works on this version. Additionally, python 3.8 is reccomended for this setup, but you can probably get by with older or newer python 3 versions (keep this in mind though, when you run into python issues).

## 1. Basics
Install git, curl, and python3.8
```
sudo apt-get install git curl python3.8
```

## 2. Install ROS Noetic
Follow ALL the instructions [found here](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ros noetic.

*Note:* 
Installinng the full desktop version will take more space and time, you also don't need the majority of the packages it installs.

## 3. Install ROS Noetic Dependencies
The ob1_arm uses rviz, gazebo, moveit, moveit planners, ros control, and several other ros packages. Run the following command to install them.

```
sudo apt-get install ros-noetic-rviz ros-noetic-gazebo-ros-pkgs \ 
ros-noetic-moveit ros-noetic-gazebo-ros-control ros-noetic-ros-controllers \
ros-noetic-rqt-joint-trajectory-controller ros-noetic-moveit-planners \
ros-noetic-moveit-fake-controller-manager libgflags-dev ros-noetic-rosparam-shortcuts \
ros-noetic-moveit-commander
```

## 4. Clone ob1_arm repo

### 4a. Create directory
Create a work directory in the location of your choosing, usually in home `~/`. Name suggestions: `ob1_arm` or `arm_ws` or `ob1_arm_ws`. The remainder of the setup guide will use the workspace name `ob1_arm`.

### 4b. Clone the repo
Next, clone the repo using the following command:
```
git clone --recurse-submodules -j8 https://github.com/UBC-OpenRobotics/ob1_arm.git src
```

After cloning the repo, there should be a src folder with the contents of the repo inside and your directory structure should look like this. **Note: The directory structure in the src folder may differ due to code changes**:

```
ob1_arm
└── src
    ├── ob1_arm_control
    ├── ob1_arm_description
    ├── ob1_arm_gazebo
    ├── ob1_arm_hw_interface
    ├── ob1_arm_mobile_moveit_config
    ├── ob1_arm_moveit_config
    ├── README.md
    ├── requirements.txt
    ├── ros_control_boilerplate
    ├── setup.md
    └── test_moveit_config
```

## 5. Install python dependencies
There is a `requirements.txt` file in the cloned src folder that contains the required python packages to install. You can pass this file to pip directly to install them.
```
pip install --upgrade -r <path to src>/requirements.txt
```

## 6. Build the ob1_arm workspace
Next, we will use catkin to build all the required packages. Navigate to top level directory `ob1_arm` and run 

```
catkin build
```

## 7. Source workspace
Next you will want to source the workspace. Navigate to top level directory `ob1_arm` and run 
```
source <path to ob1_arm>/devel/setup.bash
```

## 8. Setup complete! Test the workspace
Now that you've successfully built and sourced the workspace, you can try running demos from moveit and our ob1_arm_control packages.

try:
```
roslaunch ob1_arm_control test.launch
```

or

```
roslaunch ob1_arm_moveit_config demo.launch
```

# Troubleshooting

WIP

