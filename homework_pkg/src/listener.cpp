/*
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string data = msg->data;
  std::stringstream ss(data);
  std::string num1, num2;
  std::getline(ss, num1, ',');
  std::getline(ss, num2, ',');

  int number1 = std::stoi(num1);
  int number2 = std::stoi(num2);


  ROS_INFO("接收到的数字之和是: %d", number1 + number2);
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("topic_numbers", 1000, chatterCallback);

  ros::spin();

  return 0;
}
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string data = msg->data;
  std::stringstream ss(data);
  std::string num1, num2;
  std::getline(ss, num1, ',');
  std::getline(ss, num2, ',');

  int number1 = std::stoi(num1);
  int number2 = std::stoi(num2);

  ROS_INFO("接收到的数字之和是: %d", number1 + number2);

  // 在接收到消息后关闭节点
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("topic_numbers", 1000, chatterCallback);

  // 使用ros::spin()来处理回调
  ros::spin();

  return 0;
}

