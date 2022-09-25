# Object detection and localization in ROS melodic

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
