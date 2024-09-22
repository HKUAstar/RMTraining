#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>

int cache[2] = {0, 0};
int count = 0;

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    if (count == 0)
    {
        cache[count] = atoi(msg->data.c_str());
        count++;
    }
    else
    {
        cache[count] = atoi(msg->data.c_str());
        ROS_INFO("The sum of the two messages is %d", cache[0] + cache[1]);
        count = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}
