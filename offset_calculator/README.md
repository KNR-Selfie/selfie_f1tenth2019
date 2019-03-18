
# Offset calculator
`selfie_offset_calc` package provides a node with the same name responsible for
calculating a linear and an angular offset from the desired path. Results are
published in the `linear_offset`  and `angular_offset` topics.

## `selfie_offset_calc`

## Subscribed topics
`path`([nav_msgs/Path](docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html))

## Published topics
`linear_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
`angular_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
