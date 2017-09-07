#include <min_jerk.h>
#include "min_jerk.cpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_jerk");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
	bool continuous = true;
  ros::param::get("~continuous", continuous);

	MinJerk min_jerk(&nh, continuous);

	ros::spin();
	return 0;
}
