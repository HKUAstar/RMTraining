#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

void SumUp(const std_msgs::Int32::ConstPtr& msg) {
    int sum = msg->data;
    std::cout << "Sum of the integers: " << sum << std::endl;

}

    int main(int argc, char **argv) {

    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("sum", 1000, SumUp); //creates subscriber; call back function
    ros::spin();

    return 0;
}
