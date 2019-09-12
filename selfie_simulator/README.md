
```bash
mkdir ~/catkin_ws1
cd ~/catkin_ws1
mkdir src
cd src
git clone https://github.com/kdjosk/gazebo_plugin.git
cd gazebo_plugin
git checkout devel
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
