# About this repo
- Stores my notes on RoboMaster ROS tutorials and development.

# Dev Notes
- [Robot base movement - cmd_vel publishing](dev/pub-cmd_vel.md)
- [Joystick controller - cmd_vel mapping](dev/joystick_controller.md)

# Learning Notes
- [Package](learn/package-node.md#package)
  * [Create a package](learn/package-node.md#create-a-package)
  * [Compile the new package](learn/package-node.md#compile-the-new-package)
- [Node Introduction](learn/package-node.md#node-introduction)
  * [What is a node?](learn/package-node.md#what-is-a-node-)
  * [Code](learn/package-node.md#code)
  * [Run](learn/package-node.md#run)
  * [Check node information](learn/package-node.md#check-node-information)
- [Node Communication](learn/package-node.md#node-communication)
  * [How to communicate?](learn/package-node.md#how-to-communicate-)
  * [Publisher](learn/package-node.md#publisher)
  * [Subscriber](learn/topic-message-service-param.md#subscriber)
- [ROS Namespace](learn/topic-message-service-param.md#ros-namespace)
  * [Public](learn/topic-message-service-param.md#public)
  * [Private](learn/topic-message-service-param.md#private)
- [Topic](learn/topic-message-service-param.md#topic)
  * [Basic Commands - Topic](learn/topic-message-service-param.md#basic-commands---topic)
- [Message](learn/topic-message-service-param.md#message)
  * [Write a .msg file](learn/topic-message-service-param.md#write-a-msg-file)
  * [Use the defined msg type](learn/topic-message-service-param.md#use-the-defined-msg-type)
  * [Basic Commands - Message](learn/topic-message-service-param.md#basic-commands---message)
- [Service](learn/topic-message-service-param.md#service)
  * [Write a .srv file](learn/topic-message-service-param.md#write-a-srv-file)
  * [Basic Commands - Service](learn/topic-message-service-param.md#basic-commands---service)
  * [Service Class](learn/topic-message-service-param.md#service-class)
  * [Server](learn/topic-message-service-param.md#server)
  * [Client](learn/topic-message-service-param.md#client)
- [Topic and Service](learn/topic-message-service-param.md#topic-and-service)
- [Launch file](learn/topic-message-service-param.md#launch-file)
  * [Write a .launch file](learn/topic-message-service-param.md#write-a-launch-file)
  * [Launch](learn/topic-message-service-param.md#launch)
- [Parameter](learn/topic-message-service-param.md#parameter)
- [Can modify during runtime](learn/topic-message-service-param.md#can-modify-during-runtime)

# Structure of ROS
- Folder (stack)
    - Subfolder (package)
        -  Node (programs)
            - Service (functions)

# Preparation
## Creating a catkin workspace
- Under the desired directory:
    - `mkdir -p ~/catkin_ws/src`
    - `cd ~/catkin_ws/`
    - `catkin init` / `catkin_make`
        - Complete the stack stucture
        - Note: if encounter: catkin: command not found
            - `sudo apt-get install python-catkin-tools`
- Under ~/.bashrc:
    - `source ~/catkin_ws/devel/setup.bash`

    ### ~/.bashrc: 
    - `source /opt/ros/melodic/setup.bash`
        - Find where ROS is installed on your computer
        - Enable us to use ROS functionalities (e.g.: roscore, rosrun)
    - `source ~/catkin_ws/devel/setup.bash`
        - Activate the catkin workspace
    - Note:
        - only one catkin_ws can run at one time, running source on a second catkin_ws will automatically deactivate the first one and activate the second.
        - To switch between workspaces, either `source` the corresponding setup.bash file (`targetWS$ source devel/setup.bash`) , or modify the ~/.bashrc file.
        - Run `source ~/.bashrc` after modification on it.

- In VSCode:
    - Generate task.json under .vscode
        - Terminal > Configure Default Build Task > catkin_make: build
    - Teminal > Run build task
        - Difference between catkin_make: errors will show under the "problems" tab in VSCode