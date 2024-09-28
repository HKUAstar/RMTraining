#include "ros/ros.h"
#include "std_msgs/Float64.h"

double input1 = 0.0, input2 = 0.0;

void callbackInput1(const std_msgs::Float64::ConstPtr& msg) {
	input1 = msg->data;
}

void callbackInput2(const std_msgs::Float64::ConstPtr& msg) {
	input2 = msg->data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "sum_subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("input1", 10, callbackInput1);
	ros::Subscriber sub2 = n.subscribe("input2", 10, callbackInput2);
	ros::Rate loop_rate(1);
	while (ros::ok()) {
		ros::spinOnce();
		ROS_INFO("Sum: %f", input1 + input2);
		loop_rate.sleep();
								    }

	return 0;
}
