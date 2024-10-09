#include "ros/ros.h"
#include "beginner_pkg/INT.h"
void chatterCallback(const beginner_pkg::INT::ConstPtr& msg)
{
    int sum = msg->int1 + msg->int2;
    ROS_INFO("I heard: sum=%d",  sum);
}
    int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}
