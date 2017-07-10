#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <std_msgs/Bool.h>

#define maxV 1.0
#define maxA 0.5
#define Z 1.0

#define tV M_PI/(8*maxV)
#define tA maxV/maxA

ros::Publisher waypoints;
ros::Timer timer;
asctec_msgs::WaypointCmd waypoint;
float ang = 0.0;
bool first = true;
bool up = true;
float D = 1.0;
float time_event = D*M_PI/(4*maxV);

/* -------------------- callbacks -------------------- */
void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data && first) first = false;
  if(msg->data && !first) {
		waypoint.position.x = D*sin(ang);
		waypoint.position.y = D*cos(ang);
		waypoint.position.z = Z;
		
		ang += M_PI/8;
		ang = remainderf(ang,M_PI*2);
		waypoints.publish(waypoint);
		waypoint.time = tV;
		timer.start();
	}
}

void timerCallback(const ros::TimerEvent& event) 
{
	if(!first){
		first = true;
		timer.setPeriod(ros::Duration(D*M_PI/(4*maxV)),true);
	}
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.x = D*sin(ang);
	waypoint.position.y = D*cos(ang);
	waypoint.position.z = Z;
	waypoint.velocity.x = maxV*cos(ang);
	waypoint.velocity.y = -maxV*sin(ang);
	waypoint.accel.x = -maxA*sin(ang);
	waypoint.accel.y = -maxA*cos(ang);
	waypoint.time = D*M_PI/(4*maxV);

	ang += M_PI/4;
	ang = remainderf(ang,M_PI*2);
	waypoints.publish(waypoint);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "circley");
	ros::NodeHandle nh;
  
	/* -------------------- Publishers, and Subscribers -------------------- */
	timer = nh.createTimer(ros::Duration((tV+2*tA)/2), timerCallback);
	timer.stop();
  waypoints = nh.advertise<asctec_msgs::WaypointCmd>(ros::this_node::getNamespace()+"/waypoints", 10); 					// Position goals to linear and nonlinear controllers
  ros::Subscriber status = nh.subscribe(ros::this_node::getNamespace()+"/status", 1, statusCallback);						// Trajectory completion status

	ros::Duration(2.0).sleep();
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.z = Z;
	waypoint.time = 3.0;
	waypoints.publish(waypoint);
	waypoint.time = maxV/D+2*tA;

	ros::spin();
	return 0;
}
