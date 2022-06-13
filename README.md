# About this repo
- Stores my notes on RoboMaster ROS tutorials.

# Table of Content
- [Node Introduction](package-node.md#node-introduction)
  * [What is a node?](package-node.md#what-is-a-node-)
  * [Code](package-node.md#code)
  * [Run](package-node.md#run)
  * [Check node information](package-node.md#check-node-information)
- [Node Communication](package-node.md#node-communication)
  * [How to communicate?](package-node.md#how-to-communicate-)
  * [Publisher](package-node.md#publisher)
  * [Subscriber](topic-message-service-param.md#subscriber)
- [ROS Namespace](topic-message-service-param.md#ros-namespace)
  * [Public](topic-message-service-param.md#public)
  * [Private](topic-message-service-param.md#private)
- [Topic](topic-message-service-param.md#topic)
  * [Basic Commands - Topic](topic-message-service-param.md#basic-commands---topic)
- [Message](topic-message-service-param.md#message)
  * [Write a .msg file](topic-message-service-param.md#write-a-msg-file)
  * [Use the defined msg type](topic-message-service-param.md#use-the-defined-msg-type)
  * [Basic Commands - Message](topic-message-service-param.md#basic-commands---message)
- [Service](topic-message-service-param.md#service)
  * [Write a .srv file](topic-message-service-param.md#write-a-srv-file)
  * [Basic Commands - Service](topic-message-service-param.md#basic-commands---service)
  * [Service Class](topic-message-service-param.md#service-class)
  * [Server](topic-message-service-param.md#server)
  * [Client](topic-message-service-param.md#client)
- [Topic and Service](topic-message-service-param.md#topic-and-service)
- [Launch file](topic-message-service-param.md#launch-file)
  * [Write a .launch file](topic-message-service-param.md#write-a-launch-file)
  * [Launch](topic-message-service-param.md#launch)
- [Parameter](topic-message-service-param.md#parameter)
- [Can modify during runtime](topic-message-service-param.md#can-modify-during-runtime)

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
    - source /opt/ros/melodic/setup.bash
        - Find where ROS is installed on your computer
        - Enable us to use ROS functionalities (eg: roscore, rosrun)
    - source ~/catkin_ws/devel/setup.bash
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