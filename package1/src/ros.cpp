#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void CallBack(geometry_msgs::Twist cmdMSG) {
    std::cout<<"lock"<<cmdMSG.linear.x<<std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node1");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cme_vel",10);
    ros::Subscriber sub_cmd = nh.subscribe("cme_vel",10,CallBack);

    ros::Rate rate(10);
    while(ros::ok()) {
        std::cout<<"ksm"<<std::endl;
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 10;
        pub_cmd.publish(cmd_vel);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
