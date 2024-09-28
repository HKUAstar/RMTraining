#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "read_node");
    ros::NodeHandle n;
    ros::Publisher pub_a = n.advertise<std_msgs::String>("topic/input_a", 1000);
    ros::Publisher pub_b = n.advertise<std_msgs::String>("topic/input_b", 1000);

    ros::Rate r(10);
    while (ros::ok())
    {
        int a, b;
        std::cout << "请输入两个数字 A 和 B: ";
        std::cin >> a >> b;

        std_msgs::String msg_a;
        std_msgs::String msg_b;
        msg_a.data = std::to_string(a);
        msg_b.data = std::to_string(b);

        pub_a.publish(msg_a);
        pub_b.publish(msg_b);

        ROS_INFO("发布的数字 A: %s, B: %s", msg_a.data.c_str(), msg_b.data.c_str());

        r.sleep(); 
    }
    return 0;
}