#include <ros/ros.h>
#include <asctec_msgs/MinAccelCmd.h>
#include <std_msgs/Bool.h>

#define maxV 1.7
#define maxA 1.7
#define D 1.0
#define Z 1.0

#define tV D/maxV
#define tA maxV/maxA

ros::Publisher waypoints;
ros::Timer timer;
asctec_msgs::MinAccelCmd waypoint;
bool px = true;
bool py = true;
bool first = true;

/* -------------------- callbacks -------------------- */
void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data && first) first = false;
  if(msg->data && !first) {
		waypoint.position.x = -D+2*D*(int(px));
		waypoint.position.y = -D+2*D*(int(py));
		waypoint.position.z = Z;
		waypoint.time = tV+2*tA;
		bool temp = py;
		py = px;
		px = !temp;
		waypoints.publish(waypoint);
		timer.start();
	}
}

void timerCallback(const ros::TimerEvent& event) 
{
	if(!first){
		first = true;
		timer.setPeriod(ros::Duration((tV+2*tA)),true);
	}
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.x = -D+2*D*(int(px));
	waypoint.position.y = -D+2*D*(int(py));
	waypoint.position.z = Z;
	waypoint.time = tV+2*tA;
	bool temp = py;
	py = px;
	px = !temp;
	waypoints.publish(waypoint);
	//timer.stop();
	//timing = false;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "squarey");
	ros::NodeHandle nh;

	/* -------------------- roslaunch parameter values -------------------- */
  std::string topic;
  ros::param::get("~topic", topic);
  
	/* -------------------- Publishers, and Subscribers -------------------- */
	timer = nh.createTimer(ros::Duration((tV+2*tA)/2), timerCallback);
	timer.stop();
  waypoints = nh.advertise<asctec_msgs::MinAccelCmd>(topic + "/waypoints", 10); 					// Position goals to linear and nonlinear controllers
  ros::Subscriber status = nh.subscribe(topic + "/status", 1, statusCallback);						// Trajectory completion status

	ros::Duration(4.0).sleep();
	waypoint.header.stamp = ros::Time::now();
	waypoint.position.z = Z;
	waypoint.time = 4.0;
	waypoints.publish(waypoint);
	
	ROS_INFO("Publishing to %s", (topic + "/waypoints").c_str());
	ros::spin();
	return 0;
}
