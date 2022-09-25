# Object detection and localization in ROS melodic
<p align="center"><img src="https://user-images.githubusercontent.com/99736012/192159479-c78523a6-8d31-42b7-965c-67cccc4a2bd2.png"></p>
<p align="center"><img src="https://user-images.githubusercontent.com/99736012/192159690-968f0414-d5d5-4087-9f47-a581bdcaba1c.png" height="300px" width="400px"></p>


Tested in   
Intel Realsense Depth Camera D455 for Object detection and localization in local coordinate system   
RPlidar A3 for 2D L-SLAM   
Velodyne VLP-16 3D L-SLAM
# Installation
Dependencies
```
$ wget https://raw.githubusercontent.com/greasyrapha/capstone_install/main/dependencies_install.sh
$ sudo bash ./dependencies_install.sh
```
ROS Package (catkin_ws is your workspace)
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/greasyrapha/object_detection_localization
$ pip3 install -r requirements.txt
$ cd .. && catkin_make
```

# Usage
Boot up your SLAM Algorithms   
   
With 2D SLAM (Hector, Cartographer)
```
$ roslaunch detection 2D_run.launch
```
With 3D SLAM (LeGO-LOAM)
```
$ roslaunch detection 3D_run.launch
```


# License
https://github.com/ultralytics/yolov5
