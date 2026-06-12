# Near-Field Terrain Mapping (NFTM)

## Overview

This project implements a contact-aware LiDAR-inertial odometry framework for quadruped locomotion.

It is built upon [FAST_LIO](https://github.com/hku-mars/FAST_LIO) and inspired by [MARG](https://arxiv.org/abs/2509.20036), aiming to improve robustness for legged robots moving over challenging terrain.

## Environment requirements

1. Ubuntu 22.04
2. ROS2 Humble

## Usage

```bash
ros2 launch go2_description go2_modified_rviz.launch.py

ros2 launch fastlio2 lio_launch.py

ros2 launch nftm_tester ground_height_grid.launch.py

ros2 bag play your_bag_file
```

## Build dependencies

```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

### Detailed Instructions

#### 1. Build LIVOX-SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

#### 2. Build livox_ros_driver2

```bash
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

#### 3. Build Sophus

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

#### 注意事项

新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错

## 实例数据集

```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计

```bash
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环

#### 启动回环节点

```bash
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```

#### 保存地图

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位

#### 启动重定位节点

```bash
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```

#### 设置重定位初始值

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```

#### 检查重定位结果

```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化

#### 启动一致性地图优化节点

```bash
ros2 launch hba hba_launch.py
```

#### 调用优化服务

```bash
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```

如果需要调用优化服务，保存地图时需要设置save_patches为true

## Special thanks

[FAST_LIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)

## Performance considerations

该代码主要使用timerCB作为频率触发主函数，由于ROS2中的timer、subscriber以及service的回调实际上运行在同一个线程上，在电脑性能不是好的时候，会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来(如timerCB)来提升性能
