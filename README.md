# OE Logo Robot Control
Contains code for controlling UR robot.

## Usage
In order to use the code you need to install the [python-urx](https://github.com/jkur/python-urx/) python package and you need a physical UR robot or a simulated one.

## URController
The robot_control.py module defines a URController class that can be used to load robot configurations, to move the robot or perform calculations related to the system configuration (robot, camera, gripper setup).

Example:
```python
robot_controller = URController('CONFIG_FILE_PATH')   # create robot controller instance from config file
robot_controller.go_to_pose(pose, acc, vel)  # move the robot
print(robot_controller.rob.getj())    # print robot joint poses
robot_controller.disconnect()   # disconnect form robot
```
It is important to use the `disconnect` function to **close connection to the robot at all times** when the program terminates **otherwise the program might get stuck**.