#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <std_msgs/Bool.h>

ros::Publisher waypoints;

/* -------------------- callbacks -------------------- */
void landCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) {
		asctec_msgs::WaypointCmd waypoint;
		waypoint.header.stamp = ros::Time::now();
		waypoint.position.z = 0.0;
		waypoint.desV = 0.5;
		waypoint.desA = 0.2;
		waypoints.publish(waypoint);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "hover");
	ros::NodeHandle nh;
  
	/* -------------------- Publishers, and Subscribers -------------------- */
  waypoints = nh.advertise<asctec_msgs::WaypointCmd>(ros::this_node::getNamespace()+"/waypoints", 10); 			// Position goals to linear and nonlinear controllers
  ros::Subscriber status = nh.subscribe(ros::this_node::getNamespace()+"/land", 1, landCallback);						// Trajectory completion status
	ros::Duration(5.0).sleep();

	asctec_msgs::WaypointCmd waypoint;
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.z = 1.0;
	waypoint.desV = 0.5;
	waypoint.desA = 0.2;
	waypoints.publish(waypoint);

	ros::spin();
	return 0;
}
