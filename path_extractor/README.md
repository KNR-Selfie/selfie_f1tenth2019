# Path extractor

`path_extractor` package provides a node with the same name responsible for finding the closest points on the path based on current position of the car on the map.
Results are published onto the `closest_path_points` topic.

## `path_extractor`

## Subscribed topics
`tf` ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

## Published topics
`closest_path_points` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))

## Parameters

- `~path_points_backwards`(`int`, default: 2)
Number of points from the path behind the closest point
- `~path_points_forwards`(`int`, default: 5)
Number of points from the path ahead of the closest point

## Input file
The node takes a pickle file with resolution and origin coordinates of the map and full list of path waypoints as an input.
