# Welcome to eYRC Cosmo Logistic Theme 2023-24

### Packages
This repository contains ten packages:

1. *aws-robomaker-small-warehouse-world*: Contains warehouse rack and package models

2. *ebot_description*: Contains mobile robot (ebot) description model

3. *ebot_docking*: Contains script for the ebot to perform docking operation

4. *eyantra_warehouse*: Contains warehouse world model

5. *gazebo_ros2_control*:

6. *linkattacher_msgs*:

7. *pymoveit2*: Contains the package to perform arm manupulation

8. *realsense_gazebo_plugin*:

9. *ur5_moveit*:

10. *ur_description*:


To launch virtual environment, use this command-

```sh
ros2 launch ebot_description ebot_gazebo_launch.py
```

This should open gazebo application having mobile robot (*named as ebot*) spawned inside a warehouse.

To run the project
1. First launch the environment using
```sh
ros2 launch eyantra_warehouse task4c.launch.py
```
2. Next to enable controller for the ur5 arm run
```sh
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```
followed by
```sh
ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
```
   
4. Next enable navigation for the ebot using
```sh
ros2 launch ebot_nav2 ebot_bringup_launch.py
```

6. Run the following scripts
```sh
ros2 run ebot_docking dock_task4c.py
```
```sh
ros2 run ebot_nav2 nav_task4c.py
```
```sh
ros2 run pymoveit2 arm_task4c.py
```
```sh
ros2 run ur_description aruco_task4c.py
```

![eYRC-CL1691_Task4C](https://github.com/user-attachments/assets/6ab5b8ac-8c12-4b52-a0a1-4224004c4424)


### References

Pymoveit2 package link https://github.com/AndrejOrsula/pymoveit2

