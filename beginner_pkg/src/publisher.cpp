#include "ros/ros.h"
#include "beginner_pkg/INT.h"
#include <sstream>
#include <iostream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<beginner_pkg::INT>("chatter", 1000);
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        beginner_pkg::INT msg;
        std::cout << "Enter two numbers: ";
        std::cin >> msg.int1 >> msg.int2;
        ROS_INFO("Publishing: int1=%d, int2=%d", msg.int1, msg.int2);
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
