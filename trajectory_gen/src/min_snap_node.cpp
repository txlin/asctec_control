#include <min_snap.h>
#include "min_snap.cpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_snap");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
	float rate = 20;
	bool continuous = false;
  ros::param::get("~rate", rate);
	ros::param::get("~continuous", continuous);

	MinSnap min_snap;
	min_snap.setPubSub(&nh, rate);
	min_snap.setCont(continuous);
	ros::spin();
	return 0;
}
