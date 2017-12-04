#include <ros/ros.h>
#include <asctec_msgs/SICmd.h>
#include <sensor_msgs/Joy.h>

float MAX_ANG = M_PI/16.0;
float MAX_ANG_V = 0.2;

ros::Publisher cmder;
asctec_msgs::SICmd cmd;

bool flight = false;
bool timeout = false;

/* -------------------- callbacks -------------------- */
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (flight) {
	cmd.roll = -std::min(std::max((msg->axes[2]),-MAX_ANG),MAX_ANG);
	cmd.pitch = -std::min(std::max((msg->axes[3]),-MAX_ANG),MAX_ANG);
	cmd.thrust = 0.5*msg->axes[1]+0.5;
	cmd.yaw = -std::min(std::max((msg->axes[0]),-MAX_ANG_V),MAX_ANG_V);

	}else {
		cmd.roll = 0.0;
		cmd.pitch = 0.0;
		cmd.thrust = 0.0;
		cmd.yaw = 0.0;
	}
	if (0.5*msg->axes[1]+0.5 == 0 && msg->axes[0] > 0.5 && !timeout) {
		flight = !flight;
		timeout = true;
		ROS_INFO("Flight mode changed: %i", int(flight));
	}else if (0.5*msg->axes[1]+0.5 > 0 && msg->axes[0] <= 0.5 && timeout) {
		timeout = false;
	}
}

void timerCallback(const ros::TimerEvent&)
{
	cmder.publish(cmd);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "learn2fly");
	ros::NodeHandle nh;
  
	/* -------------------- Publishers, and Subscribers -------------------- */
  cmder = nh.advertise<asctec_msgs::SICmd>(ros::this_node::getNamespace()+"/cmd_si", 10);
  ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);
	ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);

	cmd.cmd[0] = cmd.cmd[1] = cmd.cmd[2] = cmd.cmd[3] = true;
	
	ros::spin();
	return 0;
}
