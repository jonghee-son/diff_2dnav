시연용 시나리오

출발 좌표: 37.3404810, 126.7338781
도착 좌표: 37.3401607, 126.7338753

```bash
roslaunch diff_2dnav device.launch
roslaunch diff_2dnav diff_2dnav_ukf.launch
roslaunch diff_2dnav imnav_goal.launch
roslaunch image_nav image_nav.launch
```
```bash
rostopic pub /robot_pose geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 37.3404810, y: 126.7338781 } } }' -1
```
```bash
rostopic pub /robot_pose geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 37.3401607, y: 126.7338753 } } }' -1
```
```bash
rosrun image_nav csv_out_node
```
