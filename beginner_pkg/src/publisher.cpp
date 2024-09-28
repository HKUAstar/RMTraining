#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
int main(int argc, char **argv)
{
	int num1, num2;

    // Prompt the user to enter the first number
    std::cout << "Enter the first number: ";
    std::cin >> num1;

    // Prompt the user to enter the second number
    std::cout << "Enter the second number: ";
    std::cin >> num2;

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << num1<<' ' << num2;
        msg.data = ss.str();
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}

