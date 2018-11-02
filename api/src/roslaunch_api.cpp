#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

void callback(std_msgs::String msg){
	std::stringstream ss;
	ss <<msg.data;
	std::system(ss.str().c_str());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "roslaunch_api");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/launch_cmd",1,callback);
	ros::spin();
	return 0;
}
