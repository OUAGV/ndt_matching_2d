# ndt_matching_2d
## environment
- ROS 2 Humble 
- Ubuntu 22.04
- This program is tested only on gazebo. Not sure if it works in real environment.

## subscribe
- /scan (sensor_msgs::msg::LaserScan)
- /current_pose_twist (nav_msgs::msg::Odometry)
  - The topic should contain the information of x,y,yaw and the angular velocity of yaw. 
- /tf (/odom -> /base_link)

## publish
- /accumulated_cloud (pointcloud2)
- /ndt_pose (geometry_msgs::msg::PoseStamped)
- /tf (/map -> /odom)

## parameters
please see ```ndt_matching_2d_component.hpp``` and ```ndt_matching_2d_component.cpp```. 

I commented on the parameters.

## How to Use
- ndt_matching
```
ros2 launch ndt_matching_2d ndt_matching_2d.launch.xml
```
- ndt_localization
```
ros2 launch ndt_matching_2d ndt_localization_2d.launch.xml
```

- simulator
```
ros2 launch ouagv_robot_description show_diff_drive_robot.launch.py
```

## reference
https://github.com/OUXT-Polaris/pcl_apps
