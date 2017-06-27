#include <min_accel.h>
#include "min_accel.cpp"

ros::Publisher pos_goal, status;
MinAccel min_accel;

/* -------------------- callbacks -------------------- */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  min_accel.setState(*msg);
}

void waypointCallback(const asctec_msgs::MinAccelCmd::ConstPtr& msg)
{
	if(msg->reset) min_accel.resetWaypoints();
	min_accel.addWaypoint(*msg);
}

void timerCallback(const ros::TimerEvent& event)
{
	std_msgs::Bool temp;
	temp.data = min_accel.getStatus();
	status.publish(temp);
	if(!temp.data) pos_goal.publish(*min_accel.getNextCommand());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_accel");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
	float rate = 20;
  std::string topic_name;
  
  ros::param::get("~rate", rate);
  ros::param::get("~topic_name", topic_name);
  
	/* -------------------- Timer, Publishers, and Subscribers -------------------- */
  ros::Timer timer = nh.createTimer(ros::Duration(1/rate), timerCallback);
  pos_goal = nh.advertise<asctec_msgs::PositionCmd>(topic_name + "/position_cmd", 10); 																	// Position goals to linear and nonlinear controllers
  status = nh.advertise<std_msgs::Bool>(topic_name + "/status", 10);							 																			// Trajectory completion status
  
  ros::Subscriber odom_sub = nh.subscribe(topic_name + "/odom", 1, odomCallback);																				// Odometry data
  ros::Subscriber wpt_sub = nh.subscribe(topic_name + "/waypoints", 100, waypointCallback);															// Waypoint data

	/* -------------------- Min_accel class-------------------- */
	ROS_INFO("min_accel listening on: %s and %s", (topic_name + "/waypoints").c_str(), (topic_name + "/odom").c_str());
	ros::spin();

	return 0;
}
