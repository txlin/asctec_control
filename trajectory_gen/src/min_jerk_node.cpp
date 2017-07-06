#include <min_jerk.h>
#include "min_jerk.cpp"

ros::Publisher pos_goal, status, wp_viz;
MinJerk min_jerk;

/* -------------------- callbacks -------------------- */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  min_jerk.setState(msg);
}

void waypointCallback(const asctec_msgs::WaypointCmd::ConstPtr& msg)
{
	if(msg->reset) min_jerk.resetWaypoints();
	if(msg->reset) wp_viz.publish(*min_jerk.deleteMarker());
	min_jerk.addWaypoint(msg);
}

void timerCallback(const ros::TimerEvent& event)
{
	std_msgs::Bool temp;
	temp.data = min_jerk.getStatus();
	status.publish(temp);
	if(!temp.data) {
		pos_goal.publish(*min_jerk.getNextCommand());
		wp_viz.publish(*min_jerk.getMarker());
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_jerk");
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
  wp_viz = nh.advertise<visualization_msgs::Marker>(topic_name+"/asctec_viz", 10);

  ros::Subscriber odom_sub = nh.subscribe(topic_name + "/odom", 10, odomCallback);																				// Odometry data
  ros::Subscriber wpt_sub = nh.subscribe(topic_name + "/waypoints", 100, waypointCallback);															// Waypoint data

	/* -------------------- min_jerk class-------------------- */
	ROS_INFO("min_jerk listening on: %s and %s", (topic_name + "/waypoints").c_str(), (topic_name + "/odom").c_str());
	ros::spin();

	return 0;
}
