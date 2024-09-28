#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>

int input_a = 0;
int input_b = 0;
bool received_input_a = false;
bool received_input_b = false;

void input_a_Callback(const std_msgs::String::ConstPtr& msg)
{
    input_a = std::stoi(msg->data);
    received_input_a = true;
}

void input_b_Callback(const std_msgs::String::ConstPtr& msg)
{
    input_b = std::stoi(msg->data);
    received_input_b = true;
    
    if (received_input_a && received_input_b)
    {
        int sum = input_a + input_b;
        ROS_INFO("和: %d", sum);
        received_input_a = false;
        received_input_b = false;
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "sum_subscribe_node");
    ros::NodeHandle n;

    ros::Subscriber sub_a = n.subscribe("topic/input_a", 1000, input_a_Callback); 
    ros::Subscriber sub_b = n.subscribe("topic/input_b", 1000, input_b_Callback);
    ros::spin();

    return 0;
}