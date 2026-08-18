# To run
---
## Terminal 1
```
cd ~/cwkj_PX4
make px4_sitl gazebo-classic
```

## Terminal 2
```
source ~/catkin_ws/devel/setup.bash
roslaunch px4 mavros_posix_sitl.launch
```

## Terminal 3
```
source ~/catkin_ws/devel/setup.bash
rosrun click_to_fly click_to_fly
```

## Terminal 4
```
rviz
```