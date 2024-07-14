# diff_2dnav
Robot initialization & navigation stack meta package for 

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
