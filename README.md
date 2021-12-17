# imu_gps_fusion


## OVERVIEW

Comparison between the performance of ekf_localization and ukf_localization based pose estimation using [robot_localization](https://github.com/cra-ros-pkg/robot_localization) for kitti dataset. 


## TOPICS

**input** : <br>
[1] `/kitti/oxts/imu` <br>
[2] `/kitti/oxts/gps/fix`

**ground truth** : <br> 
[1] `/kitti/oxts/odom`