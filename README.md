# 2020_MBZIRC

## gazebo 검증
```
1. Q ground control 켜기

처음이라면 2. 톱니바퀴 누른 후 조이스틱 켈리브레이션 이후 조이스틱 연결

3. roslaunch px4 mavros_posix_sitl.launch

4. rosrun offboard_drone offboard_node_mazirc3

5. rosrun tracking_mission mission_fire.py

6. rosrun pump_serial pump_serial_node

7. rosrun tensorflow_object_detect detect_ros_fire

8. topic으로 /ROS_RX_STATUS [0, 1] 로 계속 주기

** (Optional) 
9. rqt_image_view
```

## Joystick Key Setting
```
mode 2 , 	좌 : disarm
		우 : offboard
		상 : position
		하 : arm
```
