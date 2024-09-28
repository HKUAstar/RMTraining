#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
void chatterCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    int total;
    total = msg->data[0] + msg->data[1];
    ROS_INFO("The sum is [%d]", total);
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "listener"); //
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("asm2", 1000, chatterCallback);
ros::spin();
return 0;
}
