#include "ros/ros.h"
#include "my_package/radarWarn.h"

int main(int argc, char**argv) {
    ros::init(argc, argv, "Publisher");
    ros::NodeHandle nh;

    ros::Publisher topic_pub = nh.advertise<my_package::radarWarn>("topic", 1000);
    ros::Rate loop_rate(5);

    while(ros::ok()) {
        my_package::radarWarn radarWarn;
        radarWarn.dart_state = 0;
        radarWarn.drone_state = 1;
        radarWarn.engineer_state = 1;
        radarWarn.hero_state = 0;
        radarWarn.sentry_state = 1;

        topic_pub.publish(radarWarn);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}