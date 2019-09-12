
```bash
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
cd src
git clone https://github.com/KNR-Selfie/selfie_f1tenth2019.git
cd selfie_f1tenth2019
git checkout selfie_simulator
cd ../..
sudo apt-get install ros-kinetic-ackermann-msgs
sudo apt-get install ros-kinetic-controller-manager
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make
source devel/setup.bash
roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch
```
I normalnie se odpal jakiegoś rosbaga z topikiem /drive i porównaj z rvizem. Pozdro ziomeczku
## Topics subscibed
- `/drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
## Published topics by points_publisher
- `nie wiem`, pewnie cos publikuje ale nie sprawdzalem pozdro
