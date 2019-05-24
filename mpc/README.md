# Model Predictive Controller


## Subscribed topics
- `/speed` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
- `/closest_path_points` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))

## Published topics
- `/target_speed` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/steering_angle` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/optimal_path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))

## Parameters
- `prediction_horizon` (`int`, default: 10)
- `delta_time` (`double`, default: 0.05) Time step of mpc
- `loop_rate` (`int`, default: 10) Rate of the main loop
- `max_mod_delta` (`double`, default: 0.44) Maximum angle delta that the wheels can turn in radians
- `ref_v` (`double`, default: 2) Desired speed
- `max_v` (`double`, default: 0.5) Maximum speed
- `min_v` (`double`, default: -0.1) Minimum speed 
###### Cost function weights
- `cte_weight` (`int`, default: 100) Path position offset
- `epsi_weight` (`int`, default: 100) Path heading offset
- `v_weight` (`int`, default: 15) Velocity
- `delta_weight` (`int`, default: 2000) Steering angle
- `diff_delta_weight` (`int`, default: 100) Difference between sequential steering angle commands
- `angular_v_weight` (`int`, default: 2000) Weight of delta*v
- `diff_v_weight` (`int`, default: 40) Difference between sequential target_speed commands

## Listened transform
`/map` - `/base_link`


# Kontrolowane zmienne
- v - target_speed
- delta - steering_angle

# Stan pojazdu
Na stan pojazdu skladaja sie:
- polozenie wzgledem sciezki
- orientacja
- predkosc
- skret kol


# Model
Zaleznie od aktualnego stanu i zadanych na wejscie target_speed i steering_angle wyznacza nastepny stan pojazdu.

##### Hard constraints:
- Maksymalny skret
- Maksymalna predkosc
- Maksymalna zmiana skretu kol w jednostce czasu
- Maksymalna zmiana predkosci w jednostce czasu


# Cost function
Koszt stanu - suma kwadratow kosztow skladowych stanu pomnozonych przez wlasne wagi.
Wagi dobrane tak, by zminimalizowac szanse poslizgu.

Koszty skladowe:
##### predkosc
##### orientacja i pozycja wzgledem sciezki
##### kat skretu

# Optimizer
Minimalizuje koszt sekwencji komend (par kontrolowanych zmiennych) o dlugosci okreslonej przez parametr control_horizon.
Dla każdej sekwencji komend w kroku dzialania programu wyznaczana jest suma kosztow odpowiadajacych im stanow az do prediction_horizon.


## CppAD
Program korzysta z bibliotek CppAD i IPOPT.

`sudo apt-get install cppad coinor-libipopt-dev`


