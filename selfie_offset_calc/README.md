
# Offset calculator
Paczka `selfie_offset_calc` zapewnia node o takiej samej nazwie liczący liniowe i kątowe przesunięcie od wyznaczonej ścieżki. Wyniki są publikowane w topicach `linear_offset` i `angular_offset`.

## `selfie_offset_calc`

## Subscribed topics
`closest_path_points`([nav_msgs/Path](docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html))

`tf`([tf/tfMessage](http://docs.ros.org/melodic/api/tf/html/msg/tfMessage.html))

Required transforms:

`map` -> `base_link`
## Published topics
`linear_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

`angular_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

## Parameters
`~path_approximation_by_parabola` (`bool`, default: 0) Domyślnie ścieżka jest przybliżana prostą łączącą dwa pierwsze punkty ścieżki. Jeśli 1, to ścieżka jest przybliżana wielomianem drugiego stopnia.
