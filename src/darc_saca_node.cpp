#include <ros/ros.h>

#include "QuadrotorBase.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "darc_saca_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	QuadrotorBase quad;
  quad.setup();
	
	return 0;
}
