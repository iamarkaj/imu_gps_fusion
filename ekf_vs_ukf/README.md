# ekf_vs_ukf ROS package


## INSTALL

```bash
mkdir -p $HOME/git && cd $HOME/git
git clone https://github.com/iamarkaj/imu_gps_fusion.git
ln -s $HOME/git/imu_gps_fusion/ekf_vs_ukf $HOME/catkin_ws/src
roscd && rosdep install --from-paths src --ignore-src -r -y
catkin_make
source ~/.bashrc
```


## USAGE

**ekf** : `roslaunch ekf_vs_ukf ekf_navsat.launch` <br>
**ukf** : `roslaunch ekf_vs_ukf ukf_navsat.launch`


## TRAJECTORY MARKERS

[1] <font color="yellow">Yellow</font> : Ground truth (/kitti/oxts/odom) <br>
[2] <font color="green">Green</font> : Filtered pose (/odometry/filtered) <br>
[3] Positions are scaled (default: 0.01)


## DIRECTORY STRUCTURE

```
├── config
│   ├── display.rviz
│   ├── ekf_navsat.yaml
│   └── ukf_navsat.yaml
├── dataset
│   ├── 0042_odom.bag
│   └── info
│       ├── frames.pdf
│       └── rosbag_info.pdf
├── launch
│   ├── ekf_navsat.launch
│   └── ukf_navsat.launch
├── scripts
│    └── trajectory_marker.py
├── package.xml
└── CMakeLists.txt
```