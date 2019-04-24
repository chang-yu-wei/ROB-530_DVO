# Direct Visual Odometry with Pose-Graph Optimization
Final project for ROB 530 (Mobile Robotics), Team 5

## Summary
In this project we modified a [simple DVO](https://github.com/muskie82/simple_dvo) created by muskie82 to do pose-graph optimization. Keyframes and loop closure candidates were extracted from [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). 
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
* Ubuntu 14.04 LTS or 16.04
* ROS indigo/kinetic
* OpenCV
* PointCloudLibrary

## Examples
The source codes are in ```catkin_ws/src/SDVO/```. To run this code, you can follow the instruction below:
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
* Ubuntu 14.04 LTS or 16.04
* Eigen
* OpenCV

## Examples
* Download dataset from the [TUM RGB-D benchmark](https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz) and unzip it in ```data/```
* Put the keyframe files in ```data/keyframes/```
* Data path can be modified in ```line 7-8``` of ```main.cpp```
* In ```line 10-12``` of ```main.cpp``` you can choose the mode by simply commenting the other modes.
* Then run the following commands:
```
mkdir build
cd build
cmake ..
make
./test_node
```
* A log file with format ```log_time.txt``` will be generated in ```build/```. 
* Move the log file to ```data/traj_log/```, rename the file and follow the instruction in the Evaluation section to evaluate the result.

# Evaluation Tools

## Summary
We modified the evluation codes from [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools). The code was orinally in Python2, but we modified it to be compatible with Python3. We added some visualization in 3D and log out some information we needed. A data conversion code ```convTraj.py``` was also added to convert our result into proper format. All the evaluation codes are in ```evaluate/```

## Dependency
* Python3
* numpy
* matplotlib
## Examples
* Put the association files into ```associations/```
* In ```line 5``` of ```convTraj.py```, you can modify the name of the trajectory output from the previous section.
* Then run the conversion code by:
```
cd evaluation/
python3 convTraj.py
```
* The evaluation code can be called by doing ```python3 evaluation_ate.py``` and ```python3 evaluation_rpe.py```. 
* The arguments for input can be found using ```python3 evaluation_ate.py --h``` and ```python3 evaluation_rpe.py --h```
* Or if you just want a quick test, we've prepared shell files for you. Simply run:
```
./evaluate_ate.sh
```
and
```
./evaluate_rpe.sh
```

:)






