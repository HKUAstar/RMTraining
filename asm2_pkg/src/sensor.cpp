#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cmath>


std::vector<std::pair<float, float>> coordinates;
float current_angle, angle_increment;

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    coordinates.clear();
    current_angle = msg->angle_min;
    angle_increment = msg->angle_increment;

    float x,y;
    for(int i = 0; i < msg->ranges.size(); i++)
    {
       if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max)
       {
           x = msg->ranges[i] * cos(current_angle);
           y = msg->ranges[i] * sin(current_angle);
           coordinates.push_back(std::make_pair(x,y));
       }
        current_angle += angle_increment;
    }
    for (int i = 0; i < coordinates.size(); i++)
    {
        ROS_INFO("x: %f, y: %f", coordinates[i].first, coordinates[i].second);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_handler");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);
    ros::spin();
    return 0;
}