#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <utility>

class LidarDecoder
{
public:
    LidarDecoder()
    {
        // Initialize ROS node handle and subscriber
        ros::NodeHandle nh;
        scan_sub_ = nh.subscribe("/scan", 10, &LidarDecoder::scanCallback, this);
    }

    void spin()
    {
        ros::spin();
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        std::vector<std::pair<float, float>> points;
        float angle = scan->angle_min;

        for (const auto &range : scan->ranges)
        {
            if (range >= scan->range_min && range <= scan->range_max)
            {
                float x = range * cos(angle);
                float y = range * sin(angle);
                points.emplace_back(x, y);
            }
            angle += scan->angle_increment;
        }

        // Store or process the points as needed
        // For demonstration, we just print the points
        for (const auto &point : points)
        {
            ROS_INFO("Point: (%f, %f)", point.first, point.second);
        }
    }

    ros::Subscriber scan_sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_decoder");
    LidarDecoder decoder;
    decoder.spin();
    return 0;
}