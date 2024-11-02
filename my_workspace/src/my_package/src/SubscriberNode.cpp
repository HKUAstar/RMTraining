#include "ros/ros.h"
#include "my_package/radarWarn.h"

void writeMsgToLog(const my_package::radarWarn &radarWarn) {
    ROS_INFO("dart_state=%i", radarWarn.dart_state);
    ROS_INFO("drone_state=%i", radarWarn.drone_state);
    ROS_INFO("engineer_state=%i", radarWarn.engineer_state);
    ROS_INFO("hero_state=%i", radarWarn.hero_state);
    ROS_INFO("sentry_state=%i", radarWarn.sentry_state);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle nh;

    ros::Subscriber topic_sub = nh.subscribe("topic", 1000, writeMsgToLog);

    ros::spin();

    return 0;
}