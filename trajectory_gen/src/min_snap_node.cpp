#include <min_snap.h>
#include "min_snap.cpp"

ros::Publisher pos_goal, status, wp_viz;
MinSnap min_snap;

/* -------------------- callbacks -------------------- */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  min_snap.setState(msg);
}

void waypointCallback(const asctec_msgs::WaypointCmd::ConstPtr& msg)
{
	if(msg->reset) min_snap.resetWaypoints();
	if(msg->reset) wp_viz.publish(*min_snap.deleteMarker());
	min_snap.addWaypoint(msg);
}

void timerCallback(const ros::TimerEvent& event)
{
	std_msgs::Bool temp;
	temp.data = min_snap.getStatus();
	status.publish(temp);
	if(!temp.data) {
		pos_goal.publish(*min_snap.getNextCommand());
		wp_viz.publish(*min_snap.getMarker());
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "min_snap");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
	float rate = 20;
  ros::param::get("~rate", rate);
  
	/* -------------------- Timer, Publishers, and Subscribers -------------------- */
  ros::Timer timer = nh.createTimer(ros::Duration(1/rate), timerCallback);
  pos_goal = nh.advertise<asctec_msgs::PositionCmd>(ros::this_node::getNamespace()+"/position_cmd", 10); 																	// Position goals to linear and nonlinear controllers
  status = nh.advertise<std_msgs::Bool>(ros::this_node::getNamespace()+"/status", 10);							 																			// Trajectory completion status
  wp_viz = nh.advertise<visualization_msgs::Marker>(ros::this_node::getNamespace()+"/asctec_viz", 10);

  ros::Subscriber odom_sub = nh.subscribe(ros::this_node::getNamespace()+"/odom", 1, odomCallback);																				// Odometry data
  ros::Subscriber wpt_sub = nh.subscribe(ros::this_node::getNamespace()+"/waypoints", 100, waypointCallback);															// Waypoint data

	ros::spin();
	return 0;
}
