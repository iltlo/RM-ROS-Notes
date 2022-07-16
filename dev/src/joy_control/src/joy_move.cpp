#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

#include <math.h> //for tan()


float sensitivity(const float x) {
  return tan(1.2 * x)- pow(x, 3);
}

geometry_msgs::Twist twist_info;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  twist_info.linear.y = sensitivity(msg->axes[0]);
  twist_info.linear.x = sensitivity(msg->axes[1]);
  twist_info.angular.z = sensitivity(msg->axes[2]);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joy_controller");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("joy", 1000, chatterCallback);


  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    chatter_pub.publish(twist_info);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

