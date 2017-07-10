#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

ros::Publisher waypoints;
ros::Timer timer;
asctec_msgs::WaypointCmd waypoint;
asctec_msgs::WaypointCmd last_waypoint;

const float maxV = 0.7;
const float maxA = 0.5;

bool timing = false;
bool first = true;

float timeEstimate(asctec_msgs::WaypointCmd *cmd0, asctec_msgs::WaypointCmd *cmdf, float desV, float desA)
{
	float tA0x = (cmd0->velocity.x - desV)/desA;
	float tA0y = (cmd0->velocity.y - desV)/desA;
	float tA0 = std::sqrt(std::pow(tA0x,2)+std::pow(tA0y,2));

	float tAfx = abs(cmdf->velocity.x - desV)/desA;
	float tAfy = abs(cmdf->velocity.y - desV)/desA;
	float tAf = std::sqrt(std::pow(tAfx,2)+std::pow(tAfy,2));

	float tD = std::sqrt(std::pow(cmdf->position.x-cmd0->position.x,2)+std::pow(cmdf->position.y-cmd0->position.y,2))/desV;

	return tD+tA0+tAf;
}

/* -------------------- callbacks -------------------- */
void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data && first) {
		first = false;
		waypoint.position.y = 1.0;
		waypoint.desV = maxV;
		waypoint.desA = maxA;
	}
  if(msg->data && !timing && !first) {
		waypoint.header.stamp = ros::Time::now();
		waypoint.position.y = -waypoint.position.y;
		//waypoint.time = timeEstimate(&last_waypoint, &waypoint, maxV, maxA);
		waypoints.publish(waypoint);
		last_waypoint = waypoint;
	}
}

void timerCallback(const ros::TimerEvent& event) 
{
	timer.stop();
	timing = false;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "speedy");
	ros::NodeHandle nh;
  
	/* -------------------- Publishers, and Subscribers -------------------- */
  timer = nh.createTimer(ros::Duration(1.0), timerCallback);
	timer.stop();

  waypoints = nh.advertise<asctec_msgs::WaypointCmd>(ros::this_node::getNamespace()+"/waypoints", 10); 			// Position goals to linear and nonlinear controllers
  ros::Subscriber status = nh.subscribe(ros::this_node::getNamespace()+"/status", 1, statusCallback);				// Trajectory completion status

	ros::Duration(2.0).sleep();
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.z = 1.0;
	waypoint.desV = 0.5;
	waypoint.desA = 0.5;
	waypoints.publish(waypoint);
	last_waypoint = waypoint;
	ros::spin();

	return 0;
}
