# SuperH-Lab Kobuki Slam

## How to install
 ### Install ROS Melodic
 Refer to http://wiki.ros.org/melodic/Installation/Ubuntu

 ### Install step by step with commands below
   
#### Install dependencies

    sudo apt install ros-*-rgbd-launch ros-*-libuvc ros-*-libuvc-camera ros-*-libuvc-ros
    sudo apt install --reinstall --yes python-minimal python2.7-minimal libpython2.7-minimal mesa-common-dev
    sudo apt install libsdl-image1.2-dev
    sudo apt install libsdl-dev
    sudo apt install ros-*-tf2-sensor-msgs

#### Build Rtabmap from source

    git clone -b melodic-devel https://github.com/introlab/rtabmap.git rtabmap
    cd rtabmap/build/
    cmake ..
    make -j2
    sudo make install

#### Build ROS package

    cd catkin_ws/src
    # clone this repository
    rosdep install --from-paths src --ignore-src -r -y
    cd ..
    catkin_make -j2 -DRTABMAP_SYNC_MULTI_RGBD=ON

#### Config Orbbec Astra camera

    roscd astra_camera/
    ./scripts/create_udev_rules


## How to run
