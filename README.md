# ROS Environment

**Go [here](https://github.com/repa1030/simulator_windows) for the Unity project of the simulator.**  
The computing part the simulator is placed in the ubuntu environment.  
It is possible to execute this in a virtual machine or on a native ubuntu PC.  
**1. Ubuntu Version: 18.04 with ROS Melodic**  
**2. Ubuntu Version: 20.04 with ROS Noetic**  
_Please follow the corresponding instructions to build and run the repository correctly._

## 1. Ubuntu 18.04 and ROS Melodic (recommended)

Go [here](http://wiki.ros.org/melodic/Installation/Ubuntu) for installation of ROS Melodic and install ros-melodic-desktop.

If this is the first distribution of ROS that you installed on your PC, please execute (if not already done in the ROS Melodic installation):  
`echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`  
If you already installed a ROS distribution on your PC, go to your home directory and check the file ".bashrc" for the line  
`"source /opt/ros/.../setup.bash"`  
and replace the distribution with "melodic". If you can't see the file ".bashrc" in your home directory press "Strg + H".

### 1.1. Install requirements

There are some requirements that are maybe not automatically installed with ROS.

1. For the usage of git:   
`sudo apt-get install git`
2. For the compressed image transport:  
`sudo apt-get install ros-melodic-compressed-image-transport`

### 1.2. Set up the catkin workspace (for the very first time)

1. Create new catkin workspace  
`mkdir -p ~/catkin_ws/src`
2. Go to the source folder of your workspace  
`cd ~/catkin_ws/src`
3. Clone the repository from bitbucket  
`git clone https://github.com/repa1030/simulator_ubuntu.git`
4. Go to the simulator_ubuntu folder  
`cd simulator_ubuntu`
5. Initialize the git submodules  
`git submodule init`
6. Update the git submodules  
`git submodule update`

### 1.3. Build the catkin workspace (only when source code changed)

1. Go to your catkin workspace  
`cd ~/catkin_ws/`
2. Update ROS dependencies  
`rosdep update`
3. Install ROS dependencies  
`rosdep install --from-paths src --ignore-src --rosdistro melodic -y`
4. Build your catkin_ws  
`catkin_make`

### 1.4. Run the catkin workspace (everytime when running)

1. Source the workspace  
`source ~/catkin_ws/devel/setup.bash`
2. Run the desired package/system:
    1. Rosbridge Server  
    `roslaunch hska_adas rosbridge.launch`
    2. Ego Vehicle Visualization  
    `roslaunch hska_adas car.launch`
    3. Cruise Controller  
    `roslaunch hska_adas adas_cc.launch`
    4. Lane Keeping Assist System  
    `roslaunch hska_adas adas_lkas.launch`
    5. Full ADAS Functionality  
    `roslaunch hska_adas adas_full.launch`

### 1.5. Troubleshooting

* Probably the visualization of the images from unity is not working due to errors in rqt_image_view.  
If this happens make sure the compressed_image_transport package is installed.  
`sudo apt-get install ros-melodic-compressed-image-transport`
* If there are some errors facing the messages and message types (e.g. twist stamped, ...) just restart the launch file.
* If the simulator in Windows was stopped there might be some warning messages in the console.  
Best practice is to restart the launch file if this happens otherwise there might be some complications  

## 2. Ubuntu 20.04 and ROS Noetic (experimental)

If you are using Ubuntu 20.04 and ROS Noetic, make sure to switch the serializer to Newtonsoft JSON  
in the Main Menu of the Unity Project due to complications in deserialization of BSON in the current version of the ROSBRIDGE.

Go [here](http://wiki.ros.org/noetic/Installation/Ubuntu) for installation of ROS Noetic and install ros-noetic-desktop.

If this is the first distribution of ROS that you installed on your PC, please execute (if not already done in the ROS Noetic installation):  
`echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`  
If you already installed a ROS distribution on your PC, go to your home directory and check the file ".bashrc" for the line  
`source /opt/ros/.../setup.bash"`  
and replace the distribution with "noetic". If you can't see the file ".bashrc" in your home directory press "Strg + H".

### 2.1. Install requirements

There are some requirements that are not automatically installed with ROS.

1. For the usage of git:  
`sudo apt-get install git`
2. For linking the env to python3:  
`sudo apt-get install python-is-python3`
3. For using rosdep:  
`sudo apt-get install python3-rosdep`

### 2.2. Set up the catkin workspace (for the very first time)

1. Create new catkin workspace  
`mkdir -p ~/catkin_ws/src`
2. Go to the source folder of your workspace  
`cd ~/catkin_ws/src`
3. Clone the repository from bitbucket  
`git clone https://github.com/repa1030/simulator_ubuntu.git`
4. Go to the simulator_ubuntu folder  
`cd simulator_ubuntu`
5. Initialize the git submodules  
`git submodule init`
6. Update the git submodules  
`git submodule update`

### 2.3. Build the catkin workspace (only when source code changed)

1. Go to your catkin workspace  
`cd ~/catkin_ws/`
2. Update ROS dependencies  
`rosdep update`
3. Install ROS dependencies  
`rosdep install --from-paths src --ignore-src --rosdistro noetic -y`
4. Build your catkin_ws  
`catkin_make`

### 2.4. Run the catkin workspace (everytime when running)

1. Source the workspace  
`source ~/catkin_ws/devel/setup.bash`
2. Run the desired package/system:
    1. Rosbridge Server  
    `roslaunch hska_adas rosbridge.launch bson_only:=false`
    2. Ego Vehicle Visualization  
    `roslaunch hska_adas car.launch`
    3. Cruise Controller  
    `roslaunch hska_adas adas_cc.launch bson_only:=false`
    4. Lane Keeping Assist System  
    `roslaunch hska_adas adas_lkas.launch bson_only:=false`
    5. Full ADAS Functionality  
    `roslaunch hska_adas adas_full.launch bson_only:=false`

### 2.5. Troubleshooting

* Probably the visualization of the images from unity is not working due to errors in rqt_image_view.  
If this happens make sure the compressed_image_transport package is installed.  
`sudo apt-get install ros-noetic-compressed-image-transport`
* If there are some errors facing the messages and message types (e.g. twist stamped, ...) just restart the launch file.
* If the simulator in Windows was stopped there might be some warning messages in the console.  
Best practice is to restart the launch file if this happens otherwise there might be some complications  
* RQT (or other ros core nodes) throwing an python error. This might happen due to missing python3 linking to env.  
`sudo apt-get install python-is-python3`
