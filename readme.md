# dynamic_object_detection
Ros implementation of "Dynamic Object Detection in Range data using Spatiotemporal Normals". The method is formally described in "Dynamic Object Detection in Range data using Spatiotemporal Normals" submitted to ACRA 2023.

Predictions:
![](images/DOALS_estimation.png)

Ground truth:
![](images/DOALS_GT.png)

### Datasets:

The method has been tested on a dataset collected with a UR5 robot arm.

We have also tested our approach on the [undistorded scans](https://github.com/ethz-asl/lidar_undistortion/) from the Urban Dynamic Objects LiDAR Dataset (DOALS) ([project page](https://projects.asl.ethz.ch/datasets/doku.php?id=doals), [direct link to downloads](http://robotics.ethz.ch/~asl-datasets/2021_ICRA_dynamic_object_lidar_dataset/scenes/)). See images above for samples from the dataset.


### Requirements

Install ROS following [these instructions](http://wiki.ros.org/noetic/Installation/Ubuntu) (ROS2 is not implemented).

```bash
sudo apt install build-essential cmake libeigen3-dev libomp-dev git
```

### To build:

```bash
cd normal_filter
git clone --recurse-submodules https://github.com/nmwsharp/polyscope.git
ln -s ../normal_filter ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

### To run:

```bash
roscore
cd ~/dev/data/hauptgebaeude/sequence_1 && rosbag play -l 2023-09-15-08-56-49.bag # undistorded scans
rosparam load ~/dev/dynamic_lidar/normal_filter/src/parameters.yaml
rosrun normal_filter apply_tf
rosrun normal_filter process_clouds_centered
```