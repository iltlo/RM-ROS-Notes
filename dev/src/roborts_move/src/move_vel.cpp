/*
rosrun roborts_move move_vel_node
OR
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[2, 0, 0]' '[0, 0, 1]'
*/

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int kfd = 0;
struct termios cooked, raw;

void CloseKeyboard(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ROS_INFO_STREAM("Shut Down!");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    uint8_t c;
    ros::init(argc,argv,"keyboard_node");
    ros::NodeHandle n;
    ros::Publisher keyboard_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    signal(SIGINT, CloseKeyboard);    //重定向shutdown函数，ROS默认调用ros::shutdown()

    while(ros::ok())
    {
        int num;
        geometry_msgs::Twist vel;
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return -1;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return -1;
            }
        }
        else
        {
            c = 0;
            vel.linear.x = 0;
            vel.linear.y = 0;
            vel.angular.z = 0;
        }

        if(c != 0)
        {
            ROS_INFO_STREAM("Push Down: " << c);
        }

        if(c == 'a')
        {
            vel.linear.x = -1;
        }
        else if(c == 'd')
        {
            vel.linear.x = 1;
        }
        else if(c == 'w')
        {
            vel.linear.y = 1;
        }
        else if(c == 's')
        {
            vel.linear.y = -1;
        }
        else if(c == 'q')
        {
            vel.angular.z = -1;
        }
        else if(c == 'e')
        {
            vel.angular.z = 1;
        }
        else if(c != 0)
        {
            ROS_INFO_STREAM("Invalid Command!");
            vel.linear.x = 0;
            vel.linear.y = 0;
            vel.angular.z = 0;
        }
        keyboard_pub.publish(vel);
    }
}


/*
#include "ros/ros.h"
#include "std_msgs/String.h"
// the message type header
#include "geometry_msgs/Twist.h"
#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel");
  ros::NodeHandle n;
  // match the message type: Twist
  // match the topic: cmd_vel
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate loop_rate(2);
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.angular.z = 0;
  while (ros::ok())
  {
    vel_pub.publish(vel);
    vel.linear.x += 2;
    vel.angular.z += 3;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
*/
