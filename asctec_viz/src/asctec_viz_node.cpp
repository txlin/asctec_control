#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <asctec_msgs/MinAccelCmd.h>
#include <asctec_msgs/PositionCmd.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <string.h>

bool status_flag = true;
bool timer_flag = true;

int id = 1;

visualization_msgs::Marker waypoint, trajectory;
ros::Publisher waypoint_viz, trajectory_viz;
ros::Timer timer;

void waypointCallback(const asctec_msgs::MinAccelCmd::ConstPtr& msg)
{
	waypoint.action = visualization_msgs::Marker::ADD;
	waypoint.header.stamp = ros::Time::now();
	waypoint.pose.position.x = msg->position.x;
	waypoint.pose.position.y = msg->position.y;
	waypoint.pose.position.z = msg->position.z;

	waypoint.id = id;
	id++;
	waypoint_viz.publish(waypoint);
}

void cmdCallback(const asctec_msgs::PositionCmd::ConstPtr& msg)
{
	if(!status_flag) {
		geometry_msgs::Point p;
		p.x = msg->position.x;
		p.y = msg->position.y;
		p.z = msg->position.z;
		
		trajectory.action = visualization_msgs::Marker::ADD;
		trajectory.points.push_back(p);
		trajectory_viz.publish(trajectory);
	}
}

void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data && !status_flag) {
		status_flag = true;
		waypoint.action = 3;
		waypoint_viz.publish(waypoint);
		
		trajectory.action = 3;
		visualization_msgs::Marker empty;
		trajectory.points = empty.points;
		trajectory_viz.publish(trajectory);
		id = 1;

	}else if(!msg->data){
		status_flag = false;
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "asctec_viz");
	ros::NodeHandle nh;
	//timer = nh.createTimer(ros::Duration(2.0), timerCallback);
	//timer.stop();
	
	std::string topic_name = "/asctec";
	std::string world = "/odom";
	ros::param::get("~topic_name", topic_name);
	ros::param::get("~frame", world);

	trajectory_viz = nh.advertise<visualization_msgs::Marker>(topic_name+"/trajectory_viz", 10);
	waypoint_viz = nh.advertise<visualization_msgs::Marker>(topic_name+"/waypoint_viz", 10);

	ros::Subscriber wp = nh.subscribe(topic_name+"/waypoints", 10, waypointCallback);
	ros::Subscriber cmd = nh.subscribe(topic_name+"/position_cmd", 10, cmdCallback);
	ros::Subscriber status = nh.subscribe(topic_name+"/status", 10, statusCallback);

	waypoint.header.frame_id = world;
	waypoint.type = visualization_msgs::Marker::SPHERE;
	waypoint.scale.x = 0.1;
	waypoint.scale.y = 0.1;
	waypoint.scale.z = 0.1;
	waypoint.color.a = 1.0;
	waypoint.color.r = 1.0;

	trajectory.header.frame_id = world;
	trajectory.type = visualization_msgs::Marker::POINTS;
	trajectory.id = 0;
	trajectory.scale.x = 0.025;
	trajectory.scale.y = 0.025;
	trajectory.scale.z = 0.025;
	trajectory.color.a = 1.0;
	trajectory.color.g = 1.0;

	ROS_INFO("Listening on topic set: %s", topic_name.c_str());
	ros::spin();
	return 0;
}


