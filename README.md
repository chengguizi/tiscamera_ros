# ROS Wrapper for TIS Camera

## Dependencies
- https://github.com/TheImagingSource/tiscamera
- https://github.com/chengguizi/imu_vn_100
- Remember to enable serial access, by adding current user to `dialout` group

## Get Started

After `catkin build tiscamera_ros`, the `snapimage` executable should reside in the `build\tiscamera_ros\tests` folder. It could be used to take snapshot images without running ROS.