# ROS-base_driver

## Install
1. sudo apt-get install ros-noetic-serial
2. mkdir -p catkin_ws/src && cd catkin_ws/src
3. git clone https://github.com/ChengTszYin/ROS-base_driver.git
4. cd ..
5. catkin_make
6. source devel/setup.bash

## Run
Open one terminal, 
```roslaunch base_driver robot.launch```

Open another terminal,
```roslaunch base_driver navigation.launch```

Open another terminal,
```roslaunch base_driver move_base_test.launch```
