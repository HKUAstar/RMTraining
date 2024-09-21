//1. include header files
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "communication/data.h"
#include <sstream>


int main(int argc, char *argv[]) 
{
	//setlocale(LC_ALL, ""); //in case you need to use Chinese for messages
    //2. initialize ROS node
    ros::init(argc, argv, "ErGouZi");
    //3. create node handler
    ros::NodeHandle nh;
    //4. create publisher
    ros::Publisher pub = nh.advertise<communication::data>("data", 10);//<>是泛型， 10是队列长度
    //5. publish data
    //first prepare message and message count
    //std_msgs::String msg;
    communication::data msg;
    //int cnt = 0;
    //create rate: 10Hz
    ros::Rate rate(10);
    //then make a loop to publish data
    while (ros::ok()) //while the node is okay
    {
	    //format data with sstream
	    //std::stringstream ss;
        std::cin >> msg.num1 >> msg.num2;
	    //ss << "hello <--" << msg.num1 << " " << msg.num2 << " " << cnt;
	    //msg.data = ss.str();
	
        //publish data
        pub.publish(msg);
		ROS_INFO("published data: %d, %d.", msg.num1, msg.num2); //%s requires data in in c str style
        //cnt ++;
        
		//wait to be 10Hz
        rate.sleep();
    }

    return 0;
}