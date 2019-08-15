# Selfie mapping
`Selfie mapping` package provides launch and configuration files. It uses Google Cartographer or Gmapping.

# Google Cartographer
## Launching
To build a map you can choose between two options. Each of them can use one or two lidars.
#### Viz mapping
Launch mapping with real time visualisation in Rviz.
* one lidar
```
roslaunch selfie_mapping viz_mapping_1lidar.launch bag_filename:=${HOME}/path/bag_filename.bag
```
* two lidars
```
roslaunch selfie_mapping viz_mapping_2lidar.launch bag_filename:=${HOME}/path/bag_filename.bag
```

#### Fast mapping
Create a map as fast as possible with no visualisation.
* one lidar
```
roslaunch selfie_mapping fast_mapping_1lidar.launch bag_filenames:=${HOME}/path/bag_filename.bag
```
* two lidars
```
roslaunch selfie_mapping fast_mapping_2lidar.launch bag_filenames:=${HOME}/path/bag_filename.bag
```
##### Fast mapping is not working at present.

## Map saving
To save map use the following command:
```
rosrun map_server map_saver --occ 58 --free 48 -f map_name
```
The map will be saved in the current path.

## Subscribed topics
`odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))

`scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))

`tf` ([tf/tfMessage](http://docs.ros.org/melodic/api/tf/html/msg/tfMessage.html))

## Published topics

`map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))


## Provided tf transforms

`~map` → `~odom`
# Gmapping
You can use this for mapping in real time or play data from rosbag.
* two lidars
```
roslaunch selfie_mapping gmapping.launch 
```
* one lidar
```
rosrun gmapping slam_gmapping
```
