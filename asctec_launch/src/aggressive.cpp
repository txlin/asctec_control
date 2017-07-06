#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <std_msgs/Bool.h>

#define maxV 1.7
#define maxA 1.7
#define D 1.0
#define Z 1.0

#define tV D/maxV
#define tA maxV/maxA

ros::Publisher waypoints;
ros::Timer timer;
asctec_msgs::WaypointCmd waypoint;
bool first = true, timing = false;
float x[3] = {0.0, -0.5, 1.0};
float y[3] = {0.0, 0.5, 1.0};
float vx[3] = {-1, 1, 0}; 
float vy[3] = {1, 1, 0};

/* -------------------- callbacks -------------------- */
void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data && first) first = false;
  if(msg->data && !first && !timing) {
		for(int i=0; i<3; i++) {
			waypoint.header.stamp = ros::Time::now();
			waypoint.position.x = x[i];
			waypoint.position.y = y[i];
			waypoint.velocity.x = maxV*vx[i]; 
			waypoint.velocity.y = maxV*vy[i];
			waypoint.position.z = Z;
			waypoint.time = tV+2*tA;
			waypoints.publish(waypoint);
		}
		//timer.start();
		//timing = true;
	}
}

void timerCallback(const ros::TimerEvent& event) 
{
	for(int i=0; i<3; i++) {
		waypoint.header.stamp = ros::Time::now();
		waypoint.position.x = x[i];
		waypoint.position.y = y[i];
		waypoint.velocity.x = maxV*vx[i]; 
		waypoint.velocity.y = maxV*vy[i];
		waypoint.position.z = Z;
		waypoint.time = tV+2*tA;
		waypoints.publish(waypoint);
	}
	timer.stop();
	timing = false;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "squarey");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
  std::string topic;
  ros::param::get("~topic", topic);
  
	/* -------------------- Publishers, and Subscribers -------------------- */
	timer = nh.createTimer(ros::Duration(3.0), timerCallback);
	timer.stop();
  waypoints = nh.advertise<asctec_msgs::WaypointCmd>(topic + "/waypoints", 10); 					// Position goals to linear and nonlinear controllers
  ros::Subscriber status = nh.subscribe(topic + "/status", 5, statusCallback);						// Trajectory completion status

	ros::Duration(3.0).sleep();
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.x = 1.0;
	waypoint.position.y = 1.0;
	waypoint.position.z = Z;
	waypoint.time = 6.0;
	waypoints.publish(waypoint);
	
	ROS_INFO("Publishing to %s", (topic + "/waypoints").c_str());
	ros::spin();
	return 0;
}
