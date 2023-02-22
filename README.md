# ReinforcementLearningROS
The 'ROS' version of ReinforcementLearning.

Currently, it consists of two directories: environment and utility.

Environment contains some RL envs (but only one for now).

Utility contains some commonly used functions and classes.

In environment, there are two ROS packages: PIDControl and UAV.

One can run the demo by typing:
```
cd ReinforcementLearningROS
catkin build    # catkin_make also works, but you need to choose one
source devel/setup.bash
roslaunch UAV run.launch
```
