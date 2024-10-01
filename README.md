# ros2-vector-field-controller

This ROS2 package, vector field based, allows your robot to follow an attractive point while avoiding collision with repulsive points from a Laser-Scan. A speed command can also be send to override the point following. Allowing for an assissted teleoperation with collision avoidance.

<div style="text-align: center;">
<img src="materials/depth_to_scan_features.png" alt="image" style="max-height: 300px;">|
</div>

<table align="center">
  <tr>
    <td align="center">
      <a href="materials/">More materials</a>
    </td>
  </tr>
</table>

https://github.com/Noceo200/ros2-vector-field-controller.git

## Dependencies
* ROS2 (Tested on Humble): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

The following ROS2 packages are used and should be available once ROS2 is installed:
* ament_cmake
* rclcpp
* sensor_msgs
* tf2
* tf2_ros
* tf2_geometry_msgs
* rosgraph_msgs
* nav_msgs
* PCL
* pcl_conversions
* visualization_msgs

If needed, install the missing ones:
```
sudo apt-get install ros-<ros_version>-<PACKAGE_name>
```

## Installation

```
cd <your_ros2_workspace>/src/
git clone https://github.com/Noceo200/ros2-vector-field-controller.git
cd ..
colcon build
```

## API

### Subscribed topics
 | Topic  | Type | Description |
 |-----|----|----|
 | /cloud  | `sensor_msgs/PointCloud2` | The specified Point Cloud input from your sensor |
 | /odom  | `nav_msgs/Odometry` | The odometry of your robot is optionally used to compensate robot's motion and late data |
 | **tf** | N/A | A valid transform from your configured frames |

### Published topics
 | Topic  | Type | Description |
 |-----|----|----|
 | /scan  | `sensor_msgs/LaserScan` | Output Laser-Scan |

## Configuration

### Node Settings

`rate` - The frequency at which the node operates, in Hz. (Better if equal or inferior to the depth sensor publishing speed)

### Points Cloud Filtering Settings

`topic_in` - The topic name for the input point cloud.

`ref_frame` - The reference frame for the 3D position of points, should be a frame attached to the robot at the floor level and parallel to it. If you use a frame above the floor level like "base_link" usually, you can adjust the offset below with `height_offset`.

`height_offset` - The height offset from the floor, distance between the floor and `ref_frame`. (Following z axis in `ref_frame`, usually >0 as `ref_frame` above the floor)

`min_height` - The minimum height of a point to be considered (represent height following z axis in `ref_frame`).

`max_height` - The maximum height for points (represent height following z axis in `ref_frame`).

`angle_min` - The minimum angle of points to keep in `ref_frame` (not in depth sensor frame) [-pi, pi].

`angle_max` - The maximum angle of points to keep in `ref_frame` (not in depth sensor frame) [-pi, pi] and > `angle_min`.

`range_min` - The minimum range of the scan in meters (radius from `ref_frame` center).

`range_max` - The maximum range of the scan in meters (radius from `ref_frame` center, needs to be smaller than the maximum range of the depth camera, otherwise infinite points of the camera might be interpreted as obstacles).

`speed_up_h` - An integer value that allows skipping horizontal lines of points and having a better rate, 1 = all the points, 2 = 1/2 of the points, etc.

`speed_up_v` - An integer value that allows skipping vertical lines of points and having a better rate, 1 = all the points, 2 = 1/2 of the points, etc.

`compensate_move` - A boolean flag that, if true, compensates for temporary offsets on the points during movement if the computation time is not fast enough.

`odom_topic` - The topic name for odometry data, used if `compensate_move` is true to compute how `ref_frame` moved from `world_frame` during the computation, and apply a correction.

`publish_cloud` - A boolean flag to choose whether to publish the filtered points cloud or not. No need to publish it for the LaserScan, but can be used as debug to check which points are considered to compute the scan.

`topic_out_cloud` - The topic name for the output filtered point cloud.

### Scan Settings (based on filtered points above)

`publish_scan` - A boolean flag to choose whether to publish the LaserScan or not.

`topic_out_scan` - The topic name for the output LaserScan message, which will be published in the frame `ref_frame`.

`h_angle_increment` - The horizontal angle increment in radians for the scan message (e.g., 0.004363323 rad for 1-degree resolution). A smaller value allows for a scan message with more points (if the cloud allows it).

`cliff_detect` - A boolean flag to detect and consider holes or cliffs in the floor as obstacles.

`cliff_height` - The height of points that should be considered as holes and therefore as obstacles (usually <0).

### Advanced Output Scan Settings

`time_increment` - The time increment in seconds. Let it be 0.0 if unknown.

`scan_time` - The scan time in seconds. Let it be 0.0 if unknown.

### Debugging

`debug` - A boolean flag to enable or disable debugging.

`debug_file_path` - The file path where debug information will be saved.

`show_ranges` - A boolean flag to enable or disable the display of ranges of the output scan.


## Launch
Two parameters can be specified when launching the package.

`use_sim_time` - If True, the package will subscribed to `/clock` and use this time to publish the Laser-Scan and Point Cloud messages. Otherwise, the system's clock will be used.

`params_file` - YAML file tu consider for the configurations.

To launch with the default configurations in "config/depth_filter_scan_converter_params.yaml":
```
ros2 launch depth-filter-scan-converter depth_filter_scan_converter.launch.py use_sim_time:=<true/false>
```

To launch with your own configurations:
```
ros2 launch depth-filter-scan-converter depth_filter_scan_converter.launch.py params_file:=<your_yaml_file_path> use_sim_time:=<true/false>
```
