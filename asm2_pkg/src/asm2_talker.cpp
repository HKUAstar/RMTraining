#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include <sstream>
#include <stdio.h>
int main(int argc, char **argv)
{
ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Publisher asm2_pub = n.advertise<std_msgs::Int32MultiArray>("asm2", 1000);
ros::Rate loop_rate(10);
int input1, input2;


while (ros::ok())
{
std_msgs::Int32MultiArray msg;
std::cin>>input1;
std::cin>>input2;


msg.data.push_back(input1);
msg.data.push_back(input2);

ROS_INFO("%d,%d",input1,input2);
asm2_pub.publish(msg);
ros::spinOnce();
loop_rate.sleep();

}
return 0;

}

