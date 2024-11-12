# ros2_turtle_catcher
Very basic ROS2 project meant as an exercise.
## How to use
To run this project you will need **ROS2** with **colcon**.
First, clone the repository:
```sh
git clone https://github.com/Griger5/ros2_turtle_catcher.git
```
Then enter the cloned directory and build the project:
```sh
cd ros2_turtle_catcher
colcon build
```
Next:
```sh
cd src/turtle_catcher_launch/launch
ros2 launch turtle_catcher_launch.py
```
## Result:
![](https://github.com/Griger5/ros2_turtle_catcher/blob/master/docs/turtle_catcher.gif)