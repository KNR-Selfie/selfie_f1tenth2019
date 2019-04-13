# Model Predictive Controller


## Subscribed topics
`/speed` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))

## Published topics
- `/target_speed` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/steering_angle` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/optimal_path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))

## Parameters
- `~control_horizon` (`int`, default: 3)
- `~prediction_horizon` (`int`, default: 10)
- `~delta_time` (`double`, default: 0.05)


# Kontrolowane zmienne
- target_speed
- steering_angle

# Stan pojazdu
Na stan pojazdu skladaja sie:
- polozenie na trasie
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
##### pozycja na trasie
Koszt wynikajacy z odleglosci od barierek wyznaczany na poczatku dzialania na podstawie mapy
i funkcji A*e^(1/x), gdzie A to odpowiednio dobrana stala, x to odleglosc danego punktu od przeszkody
##### predkosc i orientacja
##### kat skretu

# Optimizer
Minimalizuje koszt sekwencji komend (par kontrolowanych zmiennych) o dlugosci okreslonej przez parametr control_horizon.
Dla każdej sekwencji komend w kroku dzialania programu wyznaczana jest suma kosztow odpowiadajacych im stanow az do prediction_horizon. Na podstawie gradientu kosztu w danym punkcie dla kolejnego kroku dobierana jest odpowiednia nowa sekwencja komend. Uwzglednia ograniczenia modelu.
