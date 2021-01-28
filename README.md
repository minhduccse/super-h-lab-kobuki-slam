# SLAM with RTAP-Map and Kobuki robot
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

### 1.4. Build RTAP-Map ros-pkg in catkin workspace
```
cd ~/catkin_ws/src

git clone https://github.com/minhduccse/superh-lab-kobuki-slam

rosdep install --from-paths src --ignore-src -r -y

cd ..

catkin_make -j2 -DRTABMAP_SYNC_MULTI_RGBD=ON
```
Note that option `-DRTABMAP_SYNC_MULTI_RGBD=ON` is for multiple-camera usage.

### 1.5. Config Orbbec Astra camera
This step is required if you are using Orbbec Astra camera, for Kinect, please refer to other relevant documents.
```
roscd astra_camera/

./scripts/create_udev_rules
```

## 2. How to run
This section includes a few steps to get RTAB-Map to perform SLAM on the Khadas board (ROS master's side) and PC (ROS client's side).
### 2.1. ROS network configuration
First, let's get both Khadas board and your PCs in the same local network. In this case, Khadas board will be the ROS master node with its only one `roscore` running and other PCs will be ROS client node with NO `roscore` running.

For the Khadas terminal, do the followings:
```
vi ~/.bashrc

# add these lines to the end of .bashrc file
export ROS_MASTER_URI:http://192.168.0.100:11311 # address of Khadas
export ROS_HOSTNAME:192.168.0.100
```

For your PC terminal:
```
vi ~/.bashrc

# add these lines to the end of .bashrc file
export ROS_MASTER_URI:http://192.168.0.100:11311 # address of Khadas
export ROS_HOSTNAME:192.168.0.110 # address of PC
```
Finalize this step by executing this on both:
```
source ~/.bashrc
```
Note that ROS will only run with `roscore` running on the master node detected. Therefore, modify the added lines to these to get your ROS locally and independently available:
```
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```
### 2.2. SLAM with RTAB-Map: Mapping



## 3. What's next?