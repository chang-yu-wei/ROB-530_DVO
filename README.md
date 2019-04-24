# Direct Visual Odometry with a Loop Closure Detection and Offline Pose-Graph Optimization 
Final project for ROB 530 (Mobile Robotics), Team 5

## Summary
In this project we modified a [simple DVO](https://github.com/muskie82/simple_dvo) created by muskie82 and integrated with keyframes and loop closure candidates extracted from [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) to do pose-graph optimization. 
Evaluation tools provided by [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools) was also modified to evaluate our results.

## Executable Files
The code in this repository can be devided into 3 parts:
* Modified Simple DVO
* DVO with Pose-Graph Optimization
* Evaluation Tools

# Modified Simple DVO

## Summary
This code was mostly done by muskie82 in [simple DVO](https://github.com/muskie82/simple_dvo). We only modified the code to visualize trajectory in rviz and log out information we need. (A launch file was also added to make it easier to run.)

## Dependency
Ubuntu 14.04 LTS or 16.04
ROS indigo/kinetic
OpenCV
PointCloudLibrary

## Examples
The source codes are in catkin_ws/src/SDVO. To run this code, you can follow the instruction below:
* Download rosbag file from the [TUM RGB-D benchmark](https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.bag) and put it in ```data/``` 

* Then run the following commands:
```
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch dvo run_dvo.launch
```

# DVO with Pose-Graph Optimization

## Summary
Our main contributions are in here. We extracted keyframes and loop closure candidates from ORB-SLAM2 and operated pose-graph optimization.

## Dependency
Ubuntu 14.04 LTS or 16.04
Eigen
OpenCV

## Examples
* Download dataset from the [TUM RGB-D benchmark](https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz) and unzip it in ```data/``
* Then run the following commands:
```
mkdir build
cd build
cmake ..
make
./test_node
```
* A log file with format "log_time.txt" will be generated ```build/```. 
* Move the log file to ```data/traj_log/```, rename the file and follow the instruction in the Evaluation section to evaluate the result.




