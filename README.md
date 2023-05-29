# ndt_matching_2d
## subscribe
- /scan (sensor_msgs::msg::LaserScan)
- /odom (nav_msgs::msg::Odometry)
- /tf (/odom -> /base_link)

## publish
- /accumulated_cloud (pointcloud2)
- /current_pose (geometry_msgs::msg::PoseStamped)
- /tf (/map -> /odom)

## parameters
please see ndt_matching_2d_component.hpp.

I write comments about the parameters.

## How to Use
```
ros2 launch ndt_matching_2d ndt_matching_2d.launch.xml
```
