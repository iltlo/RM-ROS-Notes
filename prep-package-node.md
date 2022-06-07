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
        - Any difference between catkin_make **?**

## Create a package
- Under workspace src:
    - `catkin_create_pkg [package] [dependencies]`
        - Eg: `catkin_create_pkg lab roscpp std_msgs`
            - roscpp: ROS implementation in C++
            - std_msgs: includes common ROS messages that mainly encapsulates primitive types
    - In VSCode:
        - Command Palette > ROS: Create Catkin Package > Input package name >  Input dependencies
- Add C++ files (Nodes) under package/src.
- Modify Package/CMakeList.txt:
    ```
    add_executable([node_executable] src/[node.cpp])
    target_link_libraries([node] ${catkin_LIBRARIES})
    add_dependencies(talker robomaster_msg_generate_messages_cpp)
    ```
    - catkin will build on top of cmake.
    - add dependencies for the executable targets to message generation targets. For self defined msg type.
- Modify the package.xml if some dependencies are missing at package creation.
    - Tags: <build_depend> <build_export_depend> <exec_depend>

## Allow ROS to recognize new package
- Under catkin_ws:
    - `catkin build [specific package]` / `catkin_make`
        - Programs will be compiled
        - For C++, will generate executables under catkin_ws/devel/lib/[package]/
        - Note:
            - catkin build can be used from any directory in the workspace， while catkin_make only works in the top level directory.
    - In VSCode:
        - Terminal >  Run Build Task
- Verify ROS recognize the new package:
    - `rospack list | grep [package to verify]`

# Node Introduction
## What is a node?
- Node is an executable placed inside paackages.
- Packages are placed under `catkin_ws/src/[some folder]`, Nodes are under `[package]/src`

## Code
- Under src directory of package:
    - For C++:
        - `#include "ros/ros.h"`
            - include all the headers necessary to use the most common public pieces of the ROS system.
        - `ros::init(argc, argv, "node_name")`
            - Initializes ROS, allows ROS to do name remapping through the command line
            - Specify a unique name of the node (Eg: node_name)
            - Provide argc and argv so that it can **perform any ROS arguments and name remapping** that were provided at the command line. (Q: why is argc argv responsible for name remapping) **?**
        - `ros::NodeHandle n;` **?**
            - Creates a ROS handle to the process node.
            - The main access point to communication in ROS system.
        - `ros::Rate loop_rate(10);`
            - run at 10Hz
            - can use together with sleep()
                - loop_rate.sleep();
        - `while (ros::ok()) { }`
            - Cases for ros::ok() to return false:
                1. a SIGNIT received (Ctrl-C) and called ros::shutdown()
                2. Kicked off the ROS network by another node with the same name
                3. ros::shutdown() called by another part of the application
                4. all ros::NodeHandles in the node have been destroyed
    - For Python:
        - `import rospy`
        - `rospy.init_node('node_basic')`
            - Tells rospy that this executable is a node, with the name node_basic
        - `while not rospy.is_shutdown():`
            - Similar to `while True`
            - Allow more smooth node killing, without using SIGINT
        - `rospy.loginfo("hello, world!")`
            - Print text to the screen
            - Logged into the node's log file

## Run
- `roscore`
    - Keep this process running when running ROS
    - Displays "... logging to [path]", indicates the path of the log file
    - If we run a node without this command, will get message: "Unable to register with master node"
- `chmod +x [node]`
    - For Python node: grant permission to execute
- `rosrun [package] [node]`

## Check node information
- `rosnode list`
    - Shows running nodes.
    - Extract the node name that is defined at `ros::init()`
- Note - running node: /rosout
    - Name of the console log reporting mechanism in ROS.
    - http://wiki.ros.org/rosout
- `rosnode info [/rosout]`
    - To show information of a node.
    - Including the topics to publish, topics that subscribe, and services, etc.

# Node Communication
## How to communicate?
- Via a **topic**.
- A topic is a named data bus that contains data. A node can write data onto this topic, and another node can read data from such topic.
- Each data packet on the bus is called a **message**. Every topic has a given msg type. Nodes can only read/write to a topic one message at a time.
- Node write/read to a topic: **publish**/**subscribe**

## Publisher
- `ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000)`
    - `advertise<>()` function tells ROS that we want to publish on a topic
        - First parameter: topic name (Eg: "chatter")
        - Second parameter: size of message queue for publishing (Eg: buffer up a max of 1000 messages)
        - Return value: ros::Publisher object (Eg: chatter_pub)
        - <topic_namespace::msg_object> (Q: to specify message type?) **?**
        - Master node will notify the given topic subscribers, and negotiate peer-to-peer connection with this node (http://wiki.ros.org/Master)
        - If all the returned Publisher objects are **destroyed**, topic will automatically unadvertise.  **?**
- `std::stringstream ss`
- `std_msgs::String msg` and `msg.data = ss.str()`
    - a std_msgs/String message object (defined in header: `#include "std_msgs/String.h"`) to stuff with data then publish.
    - Broadcast message on ROS using message-adapted class, String, or generally **generated from a msg file**.
- `chatter_pub.publish(msg)`
    - `publish()` function send messages
        - Parameter: the message object
        - The type of this object must **match with the type** given as a template parameter to the advertise<>() call. (Eg: `std_msgs::String)`
- `ROS_INFO("%s", msg.data.c_str())`
    - Replacement for printf/cout
    - http://wiki.ros.org/rosconsole
- `ros::spinOnce()`
    - Send out message!
    - callback: call and communicate with ros core (jump back to backend program logic)
    - Ensure that the callbacks can get called, if there is a subscription into this application.
    - Not neccesary if not receiving callbacks.

## Subscriber
- `void chatterCallback(const std_msgs::String::ConstPtr& msg) {...}`
    - Callback function
    - Get called when a message arrives on the topic.
    -  "The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data." **?**
- `ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback)`
    - `subscribe()` call tells ROS that we want to receive messages on a given topic
        - First and second parameter: same as the `advertise()` function above
        - Third parameter: callback function, where messages are passed to (Eg: chatterCallback)
        - Return value: ros::Subscriber object which we **must hold on to until unsubscribe**  **?** (Eg: sub)
        - When all the Subscriber object go out of scope, this call back will automatically unsubscribe from the topic
- `ros::spin()`
    - Enters a loop, calling message callbacks as fast as possible. (pumping callbacks)
    - Will exit once ros::ok() returns false (ros::shutdown() called)
    - Ensure nothing to be done after this command

# References
- https://docs.m2stud.io/cs/ros/topic/
- https://roboticsbackend.com/ros-multiple-catkin-workspaces/
- http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B)
- https://varhowto.com/cpp-ros-catkin-package/