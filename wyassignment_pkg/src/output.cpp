#include "ros/ros.h"
#include "std_msgs/String.h"
std::string concatenated_inputs;
int count = 0;
void inputCallback(const std_msgs::String::ConstPtr& msg)
{
  concatenated_inputs += msg->data;
  count++;

  if (count == 2)
  {
    ROS_INFO("Concatenated inputs: %s", concatenated_inputs.c_str());
    concatenated_inputs.clear();
    count = 0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("inputs", 1000, inputCallback);

  ros::spin();

  return 0;
}