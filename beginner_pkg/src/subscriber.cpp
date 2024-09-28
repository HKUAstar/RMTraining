#include "ros/ros.h"
#include "std_msgs/String.h"
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    
    std::string data = msg->data;
    
    std::stringstream ss(data);
    std::string firstPart, secondPart;

    if (std::getline(ss, firstPart, ' ') && std::getline(ss, secondPart, ' '))
    {
	    int num1 = std::stoi(firstPart);
	    int num2 = std::stoi(secondPart);
	    int num3 = num1 + num2;

	    std::cout << "The sum is " << num3 << std::endl;
    }
    else
    {
	    ROS_WARN("Failed to parse message: [%s]", msg->data.c_str());
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

