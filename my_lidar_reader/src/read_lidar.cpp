/*
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <utility>
#include <cmath>

// 定义一个向量来存储点对
std::vector<std::pair<float, float>> points;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    points.clear();  // 清空之前的点对
    float angle = msg->angle_min;
    for (const auto& r : msg->ranges) {
        if (r < msg->range_max && r > msg->range_min) {
            float x = r * cos(angle);
            float y = r * sin(angle);
            points.emplace_back(x, y);
        }
        angle += msg->angle_increment;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "read_lidar");    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);
    ros::spin();
    return 0;
}
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <iostream>

// 定义一个二维数组来存储点对
std::vector<std::vector<float>> points;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    points.clear();  // 清空之前的点对
    float angle = msg->angle_min;
    for (const auto& r : msg->ranges) {
        if (r < msg->range_max && r > msg->range_min) {
            float x = r * cos(angle);
            float y = r * sin(angle);
            points.push_back({x, y});
        }
        angle += msg->angle_increment;
    }

    // 打印二维数组
    for (const auto& point : points) {
        std::cout << "x: " << point[0] << ", y: " << point[1] << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_reader_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);
    ros::spin();
    return 0;
}
