#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <utility>
#include <cmath>
#include <fstream>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::vector<std::pair<float, float>> points;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        float range = scan->ranges[i];
        if (range >= scan->range_min && range <= scan->range_max)
        {
            float angle = scan->angle_min + i * scan->angle_increment;
            float x = range * cos(angle);
            float y = range * sin(angle);
            points.emplace_back(x, y);
        }
    }
    std::ofstream outFile("points.txt", std::ios::app);
    if (outFile.is_open())
    {
        for (const auto& point : points)
        {
            outFile << "x=" << point.first << ", y=" << point.second << "\n";
        }
        outFile.close();
    }
    else
    {
        ROS_ERROR("Unable to open file");
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserlistener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 1000, scanCallback);
    ros::spin();
    return 0;
}