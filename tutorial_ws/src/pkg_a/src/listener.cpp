#include "ros/ros.h"
#include "std_msgs/Int32.h"

int number_a = 0;
int number_b = 0;

void callbackA(const std_msgs::Int32::ConstPtr& msg) {
    number_a = msg->data;
}
void callbackB(const std_msgs::Int32::ConstPtr& msg) {
    number_b = msg->data;

    int sum = number_a + number_b;
    ROS_INFO("Sum of numbers: %d", sum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub_a = n.subscribe("topic_a", 10, callbackA);
    ros::Subscriber sub_b = n.subscribe("topic_b", 10, callbackB);

    ros::spin();
    return 0;
}
