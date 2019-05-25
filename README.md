# Selfie Real Time Obstacles Detection
points_publisher and obstacles_detector are coresponding nodes working together to provide a set of 3D points. Package provides a node with the same name that takes in camera image and cloud of points and calculates co-ordinates of detected obstacles.


## Topics subscibed by points_publisher
- `/points2d` ([selfie_msgs/Points2d])
- `/camera/depth/color/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html))
## Published topics by points_publisher
- `/points3d` ([selfie_msgs/Points2d])

## Topics subscibed by obstacles_detector
- `/camera/color/image_raw` ([sensor_msgs/Image] http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)
## Topics published by obstacles_detector
- `/points2d` ([selfie_msgs/Points2d])

