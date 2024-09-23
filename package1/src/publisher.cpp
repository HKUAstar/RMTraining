#include "ros/ros.h" //conotains basic functions of ros
#include "std_msgs/String.h" //contains string
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Publisher"); //initializes node; node name
    ros::NodeHandle nh; //starts node basic functions

    ros::Publisher topic_pub = nh.advertise<std_msgs::Int32>("sum", 1000); //create publisher; publisher name; topic name; queue size
    ros::Rate loop_rate(1); //how often message is published

    int num1, num2;
    std::cout << "Enter two integers: ";
    std::cin >> num1 >> num2;

    while (ros::ok()) //loop runs as long as node is functioning
    {
        std_msgs::Int32 msg; //created message object
        msg.data = num1 + num2; //put data inside message

        topic_pub.publish(msg); //topic sends message
        ros::spinOnce(); //checks for callbacks
        loop_rate.sleep();
    }
    return 0;
}