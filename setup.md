# Setup Instructions for ob1_arm repo
Step by step guide to setup and troubleshoot this repo in order to get all the ob1 arm control and simulation code running properly. Follow each section and make sure to checkout the troubleshooting section if you run into unexpected issues.

**Author: Yousif El-Wishahy (yel.wishahy@gmail.com)**

## 0. Before you start
Make sure you're running an **Ubuntu 20.04 (codename: focal)** installation. ROS Noetic only works on this version. Additionally, python 3.8 is reccomended for this setup, but you can probably get by with older or newer python 3 versions (keep this in mind though, when you run into python issues).

## 1. Basics
Install git, curl
```
sudo apt-get install git curl
```
### 1.1 Install Python3.8 with pyenv (Optional)
Install python3.8 (if it is not installed). There are multiple ways to do this, but I reccomend to use pyenv, a convenient python version management tool https://github.com/pyenv/pyenv#installation.

Install pyenv
```
$ curl https://pyenv.run | bash
```
Add the following to your ~/.bashrc file. You can use nano to edit files. I.e. `nano ~/.bashrc`.
```
export PYENV_ROOT="$HOME/.pyenv"
command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"
```
Restart your shell.
```exec "$SHELL"```
Install python 3.8 with pyenv.
```pyenv install 3.8```

## 2. Install ROS Noetic
If you have not done so, run this command to install the full desktop version of ros noetic (reccomended).
```
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

If this fails, follow ALL the instructions [found here](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ros noetic.

## 3. Install ROS dependencies
The ob1_arm uses rviz, gazebo, moveit, moveit planners, ros control, and several other ros packages. Run the following command to install them.

```
sudo apt-get install ros-noetic-rviz ros-noetic-gazebo-ros-pkgs ros-noetic-moveit ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ros-noetic-rqt-joint-trajectory-controller ros-noetic-moveit-planners ros-noetic-moveit-fake-controller-manager libgflags-dev ros-noetic-rosparam-shortcuts ros-noetic-moveit-commander
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
<ws name>
└── src
    ├── Pipfile
    ├── Pipfile.lock
    ├── README.md
    ├── ik_solvers
    ├── moveit_configs
    ├── ob1_arm_control
    ├── ob1_arm_description
    ├── ob1_arm_gazebo
    ├── ob1_arm_hw_interface
    ├── ros_control_boilerplate
    └── setup.md
```

## 5. Install python dependencies
There is a `Pipfile` file in the cloned src folder that contains the required python packages to install. We will use pipenv for python package management.

https://pipenv.pypa.io/en/latest/

Install pipenv with 
```
pip install pipenv
```
 or 
 ```
 sudo apt-get install pipenv
 ```

Navigate to src/ and run the following command: 
```
pipenv install
```

Note, when running python code, always ensure you are in the pipenv shell by running 
```
pipenv shell
``` 
in your terminal.

## 6. Build the ob1_arm workspace
Next, we will use catkin to build all the required packages. Navigate to top level directory `ob1_arm` and run 

```
catkin build
```

After building the workspace, the file structure should look something like this:
```
<arm_ws name>
├── build
│   ├── ...
├── devel
│   ├── ...
├── logs
│   ├── ...
└── src
    ├── Pipfile
    ├── Pipfile.lock
    ├── README.md
    ├── ik_solvers
    ├── moveit_configs
    ├── ob1_arm_control
    ├── ob1_arm_description
    ├── ob1_arm_gazebo
    ├── ob1_arm_hw_interface
    ├── ros_control_boilerplate
    └── setup.md
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
Some issues I encountered before:
* Error: missing glfags when building workspace
    * solution: run ```sudo apt-get install libgflags-dev```
    * based on https://github.com/gflags/gflags/blob/master/INSTALL.md

