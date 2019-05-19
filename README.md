# selfie_f1tenth2019
# Selfie Real Time Enemy Localization System


## Subscribed topics
- `/left_laser` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
- `/right_laser` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
- `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))
- `/camera/depth/color/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html))

## Published topics
- `/enemy` ([geometry_msgs/Point](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Point.html))

## Parameters

## Listened transform
`/map` - `/base_link` 
