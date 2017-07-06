#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

ros::Publisher waypoints;
ros::Timer timer;
asctec_msgs::WaypointCmd waypoint;
bool timing = false;
bool first = true;

/* -------------------- callbacks -------------------- */
void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data && first) first = false;
  if(msg->data && !timing && !first) {
		waypoint.header.stamp = ros::Time::now();
		waypoint.position.y = -waypoint.position.y;
		waypoint.time = 2.0;
		timer.setPeriod(ros::Duration(1.0), true);
		timer.start();
		timing = true;
	}
}

void timerCallback(const ros::TimerEvent& event) 
{
	waypoints.publish(waypoint);
	timer.stop();
	timing = false;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "speedy");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
  std::string topic_name;
  ros::param::get("~topic_name", topic_name);
  
	/* -------------------- Publishers, and Subscribers -------------------- */
  timer = nh.createTimer(ros::Duration(1.0), timerCallback);
	timer.stop();

  waypoints = nh.advertise<asctec_msgs::WaypointCmd>(topic_name + "/waypoints", 10); 			// Position goals to linear and nonlinear controllers
  ros::Subscriber status = nh.subscribe(topic_name + "/status", 1, statusCallback);				// Trajectory completion status

	ros::Duration(2.0).sleep();
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.z = 1.0;
	waypoint.time = 4.0;
	waypoints.publish(waypoint);
	waypoint.position.y = 1.0;
	
	ROS_INFO("Publishing to %s", (topic_name + "/waypoints").c_str());
	ros::spin();

	return 0;
}
