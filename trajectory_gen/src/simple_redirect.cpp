#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <asctec_msgs/MinAccelCmd.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#define maxV 0.5
#define maxA 0.3

std::string topic, frame;
asctec_msgs::MinAccelCmd goal;
nav_msgs::Odometry odom_;
ros::Publisher wpt_pub, obs_pub;

void publishRedirect(struct node *n0, struct node *ng, std::pair<std::vector<std::pair<float,float> >,float> *path)
{
	float angle = atan2((*ng).y-(*n0).y, (*ng).x-(*n0).x);
	bool first = true;
	for(std::vector<std::pair<float,float> >::iterator i=path->first.begin(); i != path->first.end()-1; i++) {
		asctec_msgs::MinAccelCmd cmd;
		if(first) {cmd.reset = true; first = false;}
		cmd.position.x = (i+1)->first;
		cmd.position.y = (i+1)->second;
		cmd.position.z = odom_.pose.pose.position.z;
		cmd.time = std::sqrt(std::pow(i->first-(i+1)->first,2)+std::pow(i->second-(i+1)->second,2));
		cmd.time += maxV/maxA;
		if(i != path->first.end()-2) {
			cmd.velocity.x = maxV*cos(angle);
			cmd.velocity.y = maxV*sin(angle);
		}
		
		wpt_pub.publish(cmd);
	}
}

void redirectCallback(const asctec_msgs::MinAccelCmd::ConstPtr& msg)
{
	struct node n0, ng;
	n0.x = odom_.pose.pose.position.x;
	n0.y = odom_.pose.pose.position.y;

	ng.x = msg->position.x;
	ng.y = msg->position.y;
	ng.goal = true;
	
	std::vector<struct obstacle> obs;
	for(int i=0; i<4; i++) {
		struct obstacle *o = new struct obstacle;
		o->x = -1 + 6*float(rand() % 100) / 100;
		o->y = -1 + 6*float(rand() % 100) / 100;
		o->r = 1.2;
		o->sr = 0.75;
		o->ar = 0.3;
		obs.push_back(*o);
	}
	obs = *orderObstacles(&obs, &ng);
	visualization_msgs::Marker obstacles;
	obstacles.action = 3;
	visualization_msgs::Marker empty;
	obstacles.points = empty.points;
	obs_pub.publish(obstacles);

	obstacles.header.frame_id = frame;
	obstacles.type = visualization_msgs::Marker::POINTS;
	obstacles.id = 0;
	obstacles.scale.x = 0.025;
	obstacles.scale.y = 0.025;
	obstacles.scale.z = 0.025;
	obstacles.color.a = 0.7;
	obstacles.color.r = 0.7;
	obstacles.action = visualization_msgs::Marker::ADD;

	for(std::vector<obstacle>::iterator i = obs.begin(); i != obs.end(); i++) {
		for(float j=-M_PI; j<M_PI; j+=0.1) {
			geometry_msgs::Point p;
			p.x = i->x+cos(j)*i->r;
			p.y = i->y+sin(j)*i->r;
			p.z = odom_.pose.pose.position.z;
			obstacles.points.push_back(p);
		}
		for(float j=-M_PI; j<M_PI; j+=0.1) {
			geometry_msgs::Point p;
			p.x = i->x+cos(j)*i->ar;
			p.y = i->y+sin(j)*i->ar;
			p.z = odom_.pose.pose.position.z;
			obstacles.points.push_back(p);
		}
		for(float j=-M_PI; j<M_PI; j+=0.1) {
			geometry_msgs::Point p;
			p.x = i->x+cos(j)*i->sr;
			p.y = i->y+sin(j)*i->sr;
			p.z = odom_.pose.pose.position.z;
			obstacles.points.push_back(p);
		}
	}
	obs_pub.publish(obstacles);

	std::vector<std::pair<struct node, struct node> > nodes;
	nodes = *generateNodes(&n0, &ng, &nodes, &obs);
	n0 = *buildTree(&n0, &ng, &obs, &nodes);

	std::pair<std::vector<std::pair<float,float> >,float> path;
	path = *searchTree(&path, &n0);
	if(path.second < 1000) {publishRedirect(&n0, &ng, &path);}
	else {ROS_INFO("No safe path to %.2f, %.2f", ng.x, ng.y);}
	obs.clear();
	
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_ = *msg;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "redirect_node");
	ros::NodeHandle nh;

  ros::param::get("~topic", topic);
  ros::param::get("~world_frame", frame);

	/* -------------------- Publishers, and Subscribers -------------------- */
  wpt_pub = nh.advertise<asctec_msgs::MinAccelCmd>(topic + "/waypoints", 10); 																			// Position goals to linear and nonlinear controllers 
  obs_pub = nh.advertise<visualization_msgs::Marker>(topic + "/obstacles", 10); 																		// Obstacle positions
  ros::Subscriber redirect_sub = nh.subscribe(topic + "/redirect", 1, redirectCallback);														// Redirect data
  ros::Subscriber odom_sub = nh.subscribe(topic + "/odom", 1, odomCallback);																				// odom data

	ros::spin();
}
