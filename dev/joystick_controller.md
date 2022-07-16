[CLICK ME TO GO TO MAIN PAGE](../README.md#table-of-content-dev-notes)

# Joystick controller

## Connection
- Recognize the joystick: `ls /dev/input`
  - Genereally `js0` on local machine, and on `js2` virtual machine

## Start the joy_node
- Prerequisite: `sudo apt-get install ros-kinetic-joy`
- Test the joystick:
  - `sudo jstest /dev/input/js0`
- Run the node:
  - `rosrun  joy joy_node [device]`
    - default to be device js0 if unspecified
    - E.g.: `rosrun joy joy_node _dev:=/dev/input/js2`
- To get the joystick state data published over ROS.

## Topic - joy
  - axes[6]

## Controlling cmd_vel
- Example code: [joy_move.cpp](src/joy_control/src/joy_move.cpp)
- Turtlesim: [joy_turtle.launch](src/joy_control/launch/joy_turtle.launch)
  - Run: `roslaunch joy_control joy_turtle.launch`
  - Note: cmd_vel is remapped to a private namespace, /turtle1/cmd_vel
- RoboRTS base: [joy.launch](src/joy_control/launch/joy.launch)
  - Run: `roslaunch joy_control joy.launch`
- Note:
  - Change the [device] in the launch file if necessary (e.g.: to js2)

## Reference
- http://wiki.ros.org/joy
