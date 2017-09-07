#include <min_accel.h>
#include "min_accel.cpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_accel");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
	float rate = 20;
	bool continuous = false;
  ros::param::get("~rate", rate);
	ros::param::get("~continuous", continuous);

	MinAccel min_accel;
	min_accel.setPubSub(&nh, rate);
	min_accel.setCont(continuous);
	ros::spin();

	return 0;
}
