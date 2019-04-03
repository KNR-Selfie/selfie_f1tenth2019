
# speed_controller
`speed_controller` to paczka, wyznaczająca optymalną prędkość w oparciu o kąt odchylenia i odległość od optymalnego toru.
Przymuje trzy argumenty. 
- określający stromiznę krzywej (a)
- określający wpływ kąta na prędkość (b)
- określający wpływ odległości na prędkość (c)
# Wersja alfa
![equation](http://www.sciweavers.org/tex2img.php?eq=%5Cfrac%7Ba%7D%7Bbx%5E2%2Bcx%5E2%7D&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)
## `speed_controller`

## Subscribed topics

`linear_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

`angular_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

## Published topics
`speed_data` ([std_msgs/Float32 Message](http://docs.ros.org/lunar/api/std_msgs/html/msg/Float32.html))

