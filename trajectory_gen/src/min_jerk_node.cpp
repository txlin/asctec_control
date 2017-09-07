#include <min_jerk.h>
#include "min_jerk.cpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_jerk");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
	float rate = 20;
	bool continuous = true;
	ros::param::get("~rate", rate);
  ros::param::get("~continuous", continuous);

	MinJerk min_jerk;
	min_jerk.setPubSub(&nh, rate);
	min_jerk.setCont(continuous);

	ros::spin();
	return 0;
}
