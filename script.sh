#!/bin/bash

# Open two terminals
gnome-terminal -e "bash -c 'sleep 8;rosbag record -o name /scan /odom /gap_drive __name:=my_bag'"
#gnome-terminal -e "bash -c 'catkin_make&source devel/setup.bash&roslaunch f1tenth_simulator simulator.launch'" &
echo catkin_make&source devel/setup.bash&roslaunch f1tenth_simulator simulator.launch; rosnode kill /my_bag
