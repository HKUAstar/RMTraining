#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
int main(int argc, char **argv) {
	ros::init(argc, argv, "number_talker");
	ros::NodeHandle n;

	ros::Publisher pub_a = n.advertise<std_msgs::Float64>("input1", 10);
	ros::Publisher pub_b = n.advertise<std_msgs::Float64>("input2", 10);
	while (ros::ok()){
	std_msgs::Float64 msg_a, msg_b;
	
	double a, b;
	std::cin >> a;
	std::cin >> b;

	msg_a.data = a;
	msg_b.data = b;

	pub_a.publish(msg_a); 
	pub_b.publish(msg_b);
	ROS_INFO("Published: input1 = %.6f, input2 = %.6f", msg_a.data, msg_b.data);
	
	ros::Rate loop_rate(1);
	ros::spinOnce();
	loop_rate.sleep();
				    }
	return 0;
}
