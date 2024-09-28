/*
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("topic_numbers", 1000);

  // 从键盘读取两个输入
  std::string num1, num2;
  std::cout << "请输入第一个数字: ";
  std::cin >> num1;
  std::cout << "请输入第二个数字: ";
  std::cin >> num2;

  // 创建消息并发布
  std_msgs::String msg;
  std::stringstream ss;
  ss << num1 << "," << num2;
  msg.data = ss.str();

  ROS_INFO("发布: %s", msg.data.c_str());
  pub.publish(msg);

  // 等待消息发布完成
  ros::spinOnce();

  return 0;
}
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("topic_numbers", 1000);

  // 从键盘读取两个输入
  std::string num1, num2;
  std::cout << "请输入第一个数字: ";
  std::cin >> num1;
  std::cout << "请输入第二个数字: ";
  std::cin >> num2;

  // 创建消息并发布
  std_msgs::String msg;
  std::stringstream ss;
  ss << num1 << "," << num2;
  msg.data = ss.str();

  ROS_INFO("发布: %s", msg.data.c_str());
  pub.publish(msg);

  // 等待消息发布完成
  ros::spinOnce();

  return 0;
}
