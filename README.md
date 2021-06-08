# SLAM with RTAB-Map and Kobuki robot
This repository consists of necessary packages for running RTAB-Map on Khadas Edge-V. Versions required for Khadas: Ubuntu 18.04 Server, ROS: Melodic.

## 1. Installation requirements
### 1.1. Install ROS Melodic
Refer to http://wiki.ros.org/melodic/Installation/Ubuntu for details. It is recommended for a desktop-full installation for complete tools and libraries.
   
### 1.2. Install dependencies for RTAB-Map (rtabmap) and RTAB-Map ROS (rtabmap_ros)
The easiest way to install all the dependencies is to install then uninstall the rtabmap binaries:
```
sudo apt install ros-melodic-rtabmap ros-melodic-rtabmap-ros

sudo apt remove ros-melodic-rtabmap ros-melodic-rtabmap-ros
```
Some libraries that might be in need are:

```
sudo apt install ros-*-rgbd-launch ros-*-libuvc ros-*-libuvc-camera ros-*-libuvc-ros

sudo apt install --reinstall --yes python-minimal python2.7-minimal libpython2.7-minimal mesa-common-dev

sudo apt install libsdl-image1.2-dev

sudo apt install libsdl-dev

sudo apt install ros-*-tf2-sensor-msgs
```
As we may want to modify some parts in `rtabmap_ros` or even `rtabmap` and to make `rtabmap_ros` support for multiple-camera input, it is required to build both from source.

### 1.3. Build RTAP-Map binary from source
Note that this step SHOULD NOT be performed in the catkin workspace.
```
cd ~

git clone -b melodic-devel https://github.com/introlab/rtabmap.git rtabmap

cd rtabmap/build/

cmake ..

make -j2

sudo make install
```
Step 1.3 and 1.4 is recommended to de done on both Khadas board and your PC. For Khadas board, due to the memory limit, `make` command should be executed using less than 3 threads (`make -j2`). Higher number of threads may result in failure of `make` process for shortage in RAM. 


### 1.4. Build RTAP-Map ros-pkg in catkin workspace
```
cd ~/catkin_ws/src

git clone https://github.com/minhduccse/superh-lab-kobuki-slam

cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin_make -j2 -DRTABMAP_SYNC_MULTI_RGBD=ON
```
Note that option `-DRTABMAP_SYNC_MULTI_RGBD=ON` is for multiple-camera usage and `catkin_make` should also be run with 2 threads.

### 1.5. Config Orbbec Astra camera
This step is required if you are using Orbbec Astra camera, for Kinect, please refer to other relevant documents.
```
roscd astra_camera/

./scripts/create_udev_rules
```

## 2. How to run
This section includes a few steps to get RTAB-Map to perform SLAM on the Khadas board (ROS master's side) and PC (ROS client's side).
### 2.1. ROS network configuration
**Remark: This step is performed on both Khadas and PC**

First, let's get both Khadas board and your PCs in the same local network. In this case, Khadas board will be the ROS master node with its only one `roscore` running and other PCs will be ROS client node with NO `roscore` running.

For the Khadas terminal, do the followings:
```
vi ~/.bashrc

# add these lines to the end of .bashrc file
export ROS_MASTER_URI=http://192.168.0.100:11311 # address of Khadas
export ROS_HOSTNAME=192.168.0.100
```

For your PC terminal:
```
vi ~/.bashrc

# add these lines to the end of .bashrc file
export ROS_MASTER_URI=http://192.168.0.100:11311 # address of Khadas
export ROS_HOSTNAME=192.168.0.110 # address of PC
```
Finalize this step by executing this on both:
```
source ~/.bashrc
```
For your PC, note that ROS will only run with `roscore` running on the master node detected. Therefore, modify the added lines to these to get your ROS locally and independently available:
```
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```
### 2.2. SLAM with RTAB-Map: Mapping
**Remark: Steps in this Section 2.2 are mostly to be executed on Khadas. You are recommended to use SSH.**

First step of SLAM algorithm is to generate the surrounding's map by manually maneuvering the robot around. We already created a few launch files to invoke our ROS components of mainly `nodes` and `topics`. The next step is to navigate to and run one by one.

Let's open your favorite console; if not chosen yet, why not trying [Byobu](https://www.byobu.org/downloads)? In one window (1), run `roscore`:
```
roscore
```
In a new window (2), run `rplidar_ros` package:
```
roscd rplidar_ros/launch

roslaunch rplidar.launch
```
You may need to change the USB serial port for your LIDAR laser scanner by editting this line in the `rplidar.launch` file:
```
<param name="serial_port"   type="string"   value="/dev/ttyUSB1"/>
```
In another new window (3), run `astra_camera` package:
```
roscd astra_camera/launch

roslaunch astra.launch # for 1 camera input

roslaunch multi_astra.launch # for 2 camera input 
```
Note that for 2-camera launch file, you should specify `device_x_id` in these lines:
```
<arg name="device_1_id"       default="2bc5/0401@1/4"/>
<arg name="device_2_id"       default="2bc5/0401@6/3"/>
```
Device IDs can be obtained from the log output of either above 1-cam or 2-cam launch files.

In another new window (4), run `kobuki_node` package to bring up the nodelet manager for our Kobuki robot. Note that kobuki robot needs to be connected to Khadas board using the mini-USB port in the front.
```
roscd kobuki_node/launch

roslaunch kobuki.launch
```
In another new window (5), run `kobuki_keyop` package to manually control the robot to build the map:
```
roscd kobuki_keyop/launch 

roslaunch safe_keyop.launch
```
Finally, run `rtabmap_ros` package to begin mapping. During this, use the keyboard onto the `kobuki_keyop` terminal to navigate the robot.
```
roscd rtabmap_ros/launch

roslaunch rtabmap_2_cam.launch
```
As for rtabmap, the mapping process is considered complete if a loop closure is found (= the robot trajectory is roundly enclosed).

Want to see the mapping process? You can use `rviz` and add a few necessary topics by running this **on your PC**:
```
rosrun rviz rviz
```
**Note:** To open the rviz files in this repository, they need to be set `-rw-r--r--` permission with `chown` command.

(Experimental) To make the robot automatedly discover the surrounding environment and do mapping, we can use the `explore_lite` **on Khadas**:
```
roscd explore_lite/launch

roslaunch explore.launch
```
The map database file is store at `~/.ros/rtabmap.db`. You may want to delete this file before a new mapping session. To view the recorded stuffs, use this tool **on your PC**:
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
### 2.3. SLAM with RTAB-Map: Localization\
**Remark: Steps in this Section 2.3 are to be executed on Khadas. You are recommended to use SSH.**

After mapping is done, we can utilize it to autonomously navigate our robot. In the `rtabmap_2_camp.launch`, change this to `true`:
```
<arg name="localization"    default="true"/>
```
Next, launch all the ROS nodes as in section 2.2 above, but without `kobuki_keyop`. Instead, we have to run the `move_base` to enable autonomous navigation:
```
roscd rtabmap_ros/launch

roslaunch move_base.launch
```
Now, on the PC client node, we can use `rviz` GUI to see the map and to navigate our robot to the destination through obstacles using `rviz` and pre-configured `rtab_nav.rviz` file.

## 3. What's next?

### 3.1. Performance evaluation
As we are unsure of the performance of 2-cam vs 1-cam RTAB, we think that empirical experiments would shed some light. So, we may want to do a mapping with 2-cam sensor input and record all the required topics for `rtabmap_ros` package to run.

The needed sensor topics are (may not be remembered correctly and subject to changes):
- \scan
- \odom
- \rtabmap\rgbd_image0
- \rtabmap\rgbd_image1
- \tf2
- \tf_static

We successfully record them in a `rosbag` while doing a 2-cam mapping process. `rtabmap_ros` also works well when those topics are being played back offline. However, when we try to replay the `rosbag` for 1-cam rtabmap, as the `tf_tree` is not properly mapped, `rtabmap_ros` is not ready to work yet in this case. We think some small remapping effort would do the job! 

After that, we think it is straightforward to generate the 2D maps from both cases using `rtabmap-databaseViewer` tool and make some comparisons between them. "accuracy" is a good choice to start with, refer to a paper on this at here. // adding

### 3.2. Other useful ideas
Some thoughts we had but not yet put into reality.
1. An idea we previously discussed was about how the cameras' position may affect the performance of the RTAB-Map. We already know that feauture-based method is currently being used to extract features and thus to find loop closure amongst images. For this reason, I think whatever position they are in, as long as they are immobile throughout the mapping process, it is just like we are using a wider angle camera and there is not much to improve. However, this may come into play in cases of rich-feature environment so why not discussing this more?

2. We also talked about the mapping time between 1-cam vs 2-cam. However, this heavily depends on the autonomous mapping algorithm and I think if we want to use the metric, we may need to develop one ourselves.

### 3.3. Some interesting papers
Listed here are a few papers we find somewhat helpful, covering a wide range of topics:
1. 1
2. 2
3. 3


## Appendix

### 1. Power supply

![alt text](https://github.com/minhduccse/superh-lab-kobuki-slam/blob/readme/power-supply-board/docs/power_supply.png?raw=true "Power supply for Khadas, Orbbec Astra cameras and RPLIDAR A1")

| | Component  | Purpose |
| ------------- | ------------- | ------------- |
| A  | Switch  | Switch to turn on/off the whole system |
| 1  | DC Connector | XT60 port that accepts 12.6V power input from adapter (for testing purpose) |
| 2  | XT60  | XT60 port that accepts 12.6V power input (battery power delivery) |
| 3  | USB-A  | USB-A port that supplies 6V-5A power to Khadas |
| 4,5  | USB-A | USB-A port that supplies 5V-3A power to USB Hub |
| 6  | DC Connector | DC charging port that accepts 12.6V power input |
| 7  | XT60 | XT60 port that supplies 12V-10A power to power supply board |

