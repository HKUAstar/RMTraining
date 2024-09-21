#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher input_pub = n.advertise<std_msgs::String>("inputs", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg1, msg2;
    std::string input1, input2;

    std::cout << "Enter first input (character or number): ";
    std::cin >> input1;
    std::cout << "Enter second input (character or number): ";
    std::cin >> input2;

    msg1.data = input1;
    msg2.data = input2;

    input_pub.publish(msg1);
    input_pub.publish(msg2);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}