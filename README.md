# diff_2dnav
Robot initialization & navigation stack meta package for GNSS waypoint based autonomous mobile delivery robot.
![작품_사진](https://github.com/user-attachments/assets/a9e52e55-2bda-4918-b7d5-76a4e4e07f41)
![구성도](https://github.com/user-attachments/assets/f4dbab38-a2a9-44c5-a561-14a1b428b38f)

## Installation
1. Clone the repository or download the package into your ROS workspace's ```src``` folder.
```bash
cd /path/to/your/ros_ws/src
git clone https://github.com/jonghee-son/diff_2dnav.git
```
2. Build the package using ```catkin_make```
```bash
cd /path/to/your/ros_ws
catkin_make
```

## Operation
To run the package, use the provided launch file:
```bash
roslaunch diff_2dnav device.launch
roslaunch diff_2dnav diff_2dnav_ukf.launch or roslaunch diff_2dnav diff_2dnav_ukf_odom.launch
roslaunch diff_2dnav imnav_goal.launch
```
or
```bash
roslaunch diff_2dnav device_camera.launch
roslaunch diff_2dnav diff_2dnav_ukf_camera.launch or roslaunch diff_2dnav diff_2dnav_ukf_odom_camera.launch
roslaunch diff_2dnav imnav_goal.launch
```
This will launch the following nodes:
 * ```md_robot_node & ekf_odom_pub```: Motor driver node that subscribes to ```/cmd_vel``` and publishes ```/odom_data_quat``` (wheel odometry).
 * ```realsense2_camera```: Intel RealSense camera node that publishes depth image.
 * ```rob_st_pub```: Robot State Publisher node responsible for whole TF initialization.
 * ```iahrs_driver```: Driver node for intellithings iAHRS
 * ```ntrip_client_node```: NTRIP client node that connects to National Geographic Information Institute's NRTK server.
 * ```ublox_gps```: Driver node for sparkfun GPS-RTK2 board (ublox ZED-F9P).
 * ```rplidarNode```: Driver node for RPLIDAR S2.
 * ```robot_localization_ekf_node_odom```: UKF localization node for odom to base_footprint translation.
 * ```robot_localization_ekf_node_map```: UKF localization node for map to odom translation.
 * ```laser_filter```: laser filter for cutting out portion of ```/scan``` data.
 * ```slam_toolbox```: slam_toolbox for mapping and obstacle avoidance.
 * ```navsat_transform_node```: Node responsible for GPS to ROS frame coordinate transform.
 * ```move_base```: move base node.
   
## ROS Topics
The package uses the following ROS topics:
 * ```/odom_data_quat``` (nav_msgs/Odometry): Robot's wheel odometry.
 * ```/ublox_gps/fix``` (sensor_msgs/NavSatFix): Current GNSS coordinate (Lat/Long) of robot.
 * ```/imu/data``` (sensor_msgs/Imu): Raw imu data from iAHRS.
 * ```/ublox_gps/nmea``` (nmea_msgs/Sentence): NMEA data from ublox ZED-F9P.
 * ```/ntrip_client/rtcm``` (rtcm_msgs/Message): RTCM correction data from NGII's NRTK server.
 * ```/odometry/filtered``` (nav_msgs/Odometry): UKF applied fused odometry (IMU + wheel odometry) (odom to base_footprint).
 * ```/odometry/filtered/global``` (nav_msgs/Odometry): UKF applied fused odometry (IMU + wheel odometry + GPS) (map to odom).
 * ```/scan``` (sensor_msgs/LaserScan): Raw laser scan data from RPLIDAR S2.
 * ```/scan_filtered``` (sensor_msgs/LaserScan): filtered laser scan data from ```laser_filter``` node.

## Related Packages
[imnav_goal](https://github.com/jongwonbaek2000/imnav_goal): Package to send GPS coordinate as move_base goal.</br>
[image_nav](https://github.com/jongwonbaek2000/image_nav): Image based path planning and visualization package for waypoint based navigation.</br>
[ublox](https://github.com/jonghee-son/ublox): Modified version of ublox ROS driver for RTK use.</br>
[iahrs_driver](https://github.com/jonghee-son/iahrs_driver): Modified version of iAHRS ROS driver.</br>
[md](https://github.com/jonghee-son/md): Modified version of MDROBOT motor driver ROS driver.

## Paper
손종희, 백종원, 신윤호, 황채민, 남윤석 (2024). [위성측위시스템 경유점 기반 자율주행 모바일 배송 로봇 개발](https://jhson.dev/TP1-9.pdf). _제어로봇시스템학회 국내학술대회 논문집_, 2024 제39회 제어로봇시스템학회 학술대회, 929-930.</br>
손종희, 백종원, 신윤호, 황채민 (2024). [자율주행로봇 경로 계획에서 GNSS 활용에 관한 연구](https://jhson.dev/KIPS_C2024B0003.pdf). _ACK 2024(추계) 학술발표대회 논문집(제31권 제2호)_, ACK 2024(추계) 학술발표대회, 839-840.
