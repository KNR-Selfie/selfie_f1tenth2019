# Selfie mapping
`Selfie mapping` package provides Google Cartographer's launch and configuration files.

## Launching
To start building a map, execute the following command:
```
roslaunch selfie_mapping selfie_mapping.launch bag_filename:=${HOME}/path/bag_filename.bag
```

## Workspace setup
The package should be placed in the [source space](http://wiki.ros.org/catkin/workspaces#Source_Space) of your Cartographer's workspace.

## Subscribed topics
`odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))

`scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))

`tf` ([tf/tfMessage](http://docs.ros.org/melodic/api/tf/html/msg/tfMessage.html))

## Published topics

`map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))


## Provided tf transforms

`~map` → `~odom`
