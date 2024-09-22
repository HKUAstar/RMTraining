#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok())

    {
        std_msgs::String msg1, msg2;
        std::string str1, str2;

        std::cout << "Enter the first number:";
        std::cin >> str1;
        std::cout << "Enter the second number:";
        std::cin >> str2;

        msg1.data = str1;
        msg2.data = str2;

        // ROS_INFO("%d", msg1.data.c_str());

        chatter_pub.publish(msg1);
        chatter_pub.publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
