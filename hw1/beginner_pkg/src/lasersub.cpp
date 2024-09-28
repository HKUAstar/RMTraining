#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include "string.h"
/*
steps:
    1. include headers --> (text in ROS: std_msgs/String.h)
    2. initialize ROS nodes
    3. create node handle
    4. create subscriber
    5. obtain & process data
*/
//double arr[] = new double(msg->ranges.size());

#define MAX_RANGES_SIZE 720
double *arrx = new double[MAX_RANGES_SIZE]; //data structure for storing the points' x-axis value
double *arry = new double[MAX_RANGES_SIZE]; //data structure for storing the points' y-axis value
int num_of_elements;

void printData() {
    ROS_INFO("==========\nthis is one batch of data:");
    for (int i = 0; i < num_of_elements; i++) {
        ROS_INFO("point %d: (%.2f, %.2f)", i, arrx[i], arry[i]);
    }
}

void doMsg(const sensor_msgs::LaserScan::ConstPtr &msg) {
    //obtain and operate subscribed data via msg
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;
    
    double x;
    double y;
    double cur_angle = angle_min;
    num_of_elements = msg->ranges.size();
    for (int i = 0; i < msg->ranges.size(); i++) {
        x = msg->ranges[i]*std::cos(cur_angle);
        y = msg->ranges[i]*std::sin(cur_angle);
        arrx[i] = x;
        arry[i] = y;
        //ROS_INFO("Angle: %.2f; distance: %.2f", cur_angle, msg->ranges[i]);
        //ROS_INFO("read point: (%.2f, %.2f)", x, y);
        cur_angle += angle_increment;
    }
    printData();
}

int main(int argc, char *argv[])
{
	//setlocale(LC_ALL, ""); //in case you need to show Chinese
    //2. initialize node
    ros::init(argc, argv, "laserSub"); //nodes should not have same names
    //3. node handle
    ros::NodeHandle nh;
    //4. subscriber
    ros::Subscriber sub = nh.subscribe("/scan", 10, doMsg);//无需添加泛型，可自动识别
    //5. process subscribed data
    
    //6. spin on the function (回调函数)
    ros::spin();
    return 0;
}