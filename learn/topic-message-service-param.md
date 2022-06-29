[CLICK ME TO GO TO PREVIOUS PAGE](package-node.md)

# ROS Namespace
## Public
- starts with "/" are global

## Private
- starts with "~"
- define in `ros::NodeHandle nh1("~")` or `ros::NodeHandle nh2("~my_private_ns")`
    - Namespace of nh1: /[node_name] or nh2: /[node_name]/[my_private_ns]


# Topic
## Basic Commands - Topic
- `rostopic pub [topic]`
- `rostopic echo [topic]`
- `rostopic info [topic]`

# Message
## Write a .msg file
- Structure
    - `catkin_ws/src/[package directory]/msg/[file].msg file`
- [field_type] [field_name]
- 4 Field types
    1. Built-in type
        - http://wiki.ros.org/msg#Fields (check out 2.1.1)
    2. Names of self-defined Message descriptions (Q: message of message?) **?**
    3. Arrays (Eg: float32[] ranges)
    4. Header type, which maps to std_msgs/Header
## Use the defined msg type
- Under [msg_package_name] directory
    - Under package.xml
        - change the depend
            ```
            <!-- use at build time --> 
            <build_depend>message_generation</build_depend>
            <!-- use at runtime -->
            <exec_depend>message_runtime</exec_depend>
            ```
    - Under CMakeLists.txt
        - Under find_package(catkin REQUIRED COMPONENTS
            - Add **package component**: `message_generation`
                ```
                find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    std_msgs
                    message_generation  # add this
                )
                ```
        - Under add_message_files(
            - Include the **.msg file name**
                ```
                add_message_files(
                    FILES
                    Num.msg # add this
                )
                ```
        - Under generate_messages(
            - Include the **dependencies** required by the added messages (and services) above
                ```
                generate_messages(
                    DEPENDENCIES
                    std_msgs
                )
                ```
- Under [directory] where the user-defined msg is used
    - Under CMakeLists.txt
        - Under find_package(catkin REQUIRED COMPONENTS
            - Add **package component**: `[msg_package_name]`
                ```
                find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    std_msgs
                    lab_msg  // add this
                )
                ```
            - **Add dependency** for the node which uses the message
                ```
                add_dependencies(talker [msg_package_name]_generate_messages_cpp)
                ```
- Under the [node_src_file]
    - `#include "[msg_package_name]/[msg_filename].h"`
        - msg header file will be generated under devel/include/[package name], include the header file when we use those messages.
    - Declare a msg object and assign values
        ```
        # Num.msg file
        int32 val
        ```
        ```
        // node file
        lab_msg::Num num;
        num.val = 0;
        ```
## Basic Commands - Message
- `rostopic type [topic]`
    - Will output the message type of that topic
- `rosmsg show [message_type]`
    - Look up the fields of the msg_type

# Service
## Write a .srv file
- Structure
    - `catkin_ws/src/[package directory]/srv/[service_type].srv file`
- Fields
    - request field (update by client)
    - response field (update by server)
    - separated by "------"
- The corresponding modifications are **very similar** to those for `messages`.
    - But in CMakeLists.txt, instead of changing "add_message_files(", modify the "add_service_files(" block.

## Basic Commands - Service
- rossrv show [service_type]
    - eg: rossrv show AddTwoInts
    - Display the details of the .srv file
- rosservice list
    - Display all the currently **active** services that where launched after you started the ROS master

## Service Class
- Shall be instantiated at the Client (Q: any situation to instantiate at the Server) **?**
- 2 members
    - `request` and `response`
- 2 class definitions
    - `Request` and `Response`

## Server
- Information about request and reponse are logged
    - Instances involved as function parameter
        - [package_name]::[service_type]::Request  &req
        - [package_name]::[service_type]::Response  &res
- `ros::ServiceServer service = n.advertiseService("add_two_ints", add)`
    - `advertiseService()` create a service and advertise over ROS
        - First parameter: service name (Eg: add_two_ints)
        - Second parameter: service function (Eg: add)
        - Return value: ros::ServiceServer object (Eg: service)

## Client
- `ros::ServiceClient client = n.serviceClient<lab_msg::AddTwoInts>("add_two_ints")`
    - `serviceClient<>()` creates a ros::ServiceClient object
        - The object will be used to call the service
        - Will "subscribe" to the [service_name]: add_two_ints
        - Return value: ros::ServiceClient object (Eg: client)
- `lab_msg::AddTwoInts srv`
    - Instantiate a **autogenerated service class** **?**
- `srv.request.a = atoll(argv[1]);`
    - Assign value into the class's request member
- `if (client.call(srv))` calls the service
    - `client.call()`
    - Q: Can I change how frequent service is tried to be called? **?**

# Topic and Service
- Node enables us to: publish/subscribe topic, and advertise/subscribe service
- Node: info exchange, Service: under node, acts just like a function

| NodeHandle class (n) | Topic / Service                                                   | corr. return type  |
|----------------------|------------------------------------------------------------------|--------------------|
| Publish              | n.advertise<std_msgs::String>("[topic_name]", [msg_queue_size]); | ros::Publisher     |
|                      | n.advertiseService("[service_name]", add)                        | ros::ServiceServer |
| Subscribe            | n.subscribe("[topic_name]", [msg_queue_size], CallbackFunction); | ros::Subscriber    |
|                      | n.serviceClient<lab_msg::AddTwoInts>("[service_name]")           | ros::ServiceClient |


# Launch file
## Write a .launch file
- Structure
    - `catkin_ws/src/[package directory]/launch/[file].launch xml file`
- Tags
    - <launch></launch>
    - <node />
        - `pkg=" "`
        - `type=" "`
            - Node type, name that matches the executable
        - `name=" "`
            - Node name, defined at ros::init()
        - `ns="my_private_ns"(optional)`
            - Node name in "my_private_ns" namespace
        - `output="screen"` to show ROSINFO output
    - <node> </node> 
        - `<rosparam file="$(find [package_name])/param/[file].yaml" />`
        - `<remap from="chatter" to="/talker_node/chatter"/>`
- Under [bringup_package_name]/CMakeLists.txt
    - Modify "find_package(catkin REQUIRED COMPONENTS" according to package needs

## Launch
- `roslaunch [package_name] [file].launch`
    - Initializes the robot
- http://wiki.ros.org/roslaunch


# Parameter
- Structure
    - `catkin_ws/src/[package directory]/param/[file].yaml file`
- Commands
    - `rosparam list`
        - List all parameter names
    - `rosparam get /`
        - Display contents of the entire Parameter Server
    - `rosparam set my_param "hello world"`
    - `rosparam load lab/param/val.yaml`
        - Q: how to control which node the parameter loads to? **?**
    - `rosparam delete my_param`
- `std::string s;`
- `n.getParam("my_param", s);`
    - n: instance of NodeHandle
    - Return value: boolean
- `n.setParam("my_param", "hello world");`
- `n.hasParam("my_param")`
    - Return value: boolean
    - Check for parameter existence
- Q: Apart from using terminal commands and <rosparam /> in .launch file, anyways to load the parameter inside a node? **?**

# Can modify during runtime
- topic name
- node name

# References
- https://hackmd.io/@st9540808/ByAVGsa1S?type=view
- http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle
- http://wiki.ros.org/roscpp_tutorials/Tutorials/WritingServiceClient
- http://wiki.ros.org/roslaunch/XML/node
- http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters