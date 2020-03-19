# 2020_MBZIRC
```
rosrun pump_serial pump_node_serial
rosrun offboard_drone offboard_node_mazirc
rosrun tracking_mission fire_mission
rosrun tensorflow_object_detect detect_ros_fire
rqt_image_view
topic으로 /ROS_RX_STATUS [0, 1] 로 계속 주기
```
