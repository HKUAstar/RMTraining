#include "ros/ros.h"
#include "std_msgs/String.h"
#include "communication/data.h"
/*
steps:
    1. include headers --> (text in ROS: std_msgs/String.h)
    2. initialize ROS nodes
    3. create node handle
    4. create subscriber
    5. obtain & process data
*/

void doMsg(const communication::data::ConstPtr &msg) {
    //obtain and operate subscribed data via msg
    ROS_INFO("The sum of the two numbers is: %ld", msg->num1 + msg->num2);
}

int main(int argc, char *argv[])
{
	//setlocale(LC_ALL, ""); //in case you need to show Chinese
    //2. initialize node
    ros::init(argc, argv, "cuiHua"); //nodes should not have same names
    //3. node handle
    ros::NodeHandle nh;
    //4. subscriber
    ros::Subscriber sub = nh.subscribe("data", 10, doMsg);//无需添加泛型，可自动识别
    //5. process subscribed data

    //6. spin on the function (回调函数)
    ros::spin();
    return 0;
}
