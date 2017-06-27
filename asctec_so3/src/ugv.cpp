#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <asctec_msgs/MinAccelCmd.h>
#include <asctec_msgs/PositionCmd.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>

#include <math.h>
#include <string.h>

#define freq 10
#define maxV 0.4

#define XBOUND_H 1.0
#define XBOUND_L -1.0
#define YBOUND_H 1.5
#define YBOUND_L -1.5
#define BORDER_NUMB 2
#define BORDER_TOP 0.5

using namespace std;

int state = 0;
bool tracking = false;
bool flying = false;
bool isDone = true;

float robot_x, robot_y, robot_yaw;
string world, ugv, topic, quad_frame;
nav_msgs::Odometry odom_quad_, odom_ugv_;

ros::Publisher wpt_pub, border_pub, pos_pub;
ros::Subscriber status_sub, joy_sub, quad_sub, ugv_sub;

void quadCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_quad_ = *msg;
}

void ugvCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_ugv_ = *msg;
}

void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[2] && flying) {
		tracking = !tracking;
		ROS_INFO("Switch Tracking state!");

	}else if(msg->buttons[3] && state == 0) {
		flying = true;
		ROS_INFO("Started!!");

	}else if(msg->buttons[3] && state == 2) {
		flying = false;
		
	}
}

void showBorder(bool outOf, float z)
{
	visualization_msgs::Marker border;
	geometry_msgs::Point corner;

	border.header.frame_id = world;
	border.header.stamp = ros::Time::now();
	border.id = 2;
	border.action = visualization_msgs::Marker::ADD;
	border.type = visualization_msgs::Marker::LINE_LIST;

	if(outOf) {
		border.color.a = 0.6;
		border.color.r = 1.0;	
	}else {
		border.color.a = 0.6;
		border.color.g = 1.0;	
	}
			
	border.scale.x = 0.05;
	border.scale.y = 1.0;
	border.scale.z = 0.5;

	corner.x = XBOUND_L;
	corner.y = YBOUND_L;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_L;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_H;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_H;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_H;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_H;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_L;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_L;	
	corner.z = z;
	border.points.push_back(corner);

	for(int i=0; i<=BORDER_NUMB; i++) {
		corner.x = XBOUND_L;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);	
	}
	border_pub.publish(border);
}

void sendTrajectory(float time, float x, float y, float z, float yaw) 
{
	asctec_msgs::MinAccelCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.yaw[0] = yaw;
	cmd.time = time;

	wpt_pub.publish(cmd);
	isDone = false;
}

void sendPoint(float x, float y, float vx, float vy, float z, float yaw)
{
	asctec_msgs::PositionCmd next;
	next.position.x = x;
	next.position.y = y;
	next.velocity.x = vx;
	next.velocity.y = vy;
	next.position.z = z;
	next.yaw[0] = yaw;
	pos_pub.publish(next);
}

void sendLandTrajectory(float time) 
{
	asctec_msgs::MinAccelCmd cmd;
	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 1.0;
	cmd.time = time;
	wpt_pub.publish(cmd);

	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 0.0;
	cmd.time = 3;
	wpt_pub.publish(cmd);

	isDone = false;
}

float limit(float value, float upper, float lower)
{
	float temp = value;
	if(value > upper) {
		temp = upper;

	}else if(value < lower) {
		temp = lower;
	}

	return temp;
}

bool outBorder(void)
{
	bool temp = false;
	if(odom_quad_.pose.pose.position.x > XBOUND_H || odom_quad_.pose.pose.position.x < XBOUND_L || odom_quad_.pose.pose.position.y > YBOUND_H || odom_quad_.pose.pose.position.y < YBOUND_L) {
		temp = true;
	}
	return temp;
}

bool outUGVBorderX(void)
{
	bool temp = false;
	if(odom_ugv_.pose.pose.position.x > XBOUND_H || odom_ugv_.pose.pose.position.x < XBOUND_L) {
		temp = true;
	}
	return temp;
}
bool outUGVBorderY(void)
{
	bool temp = false;
	if(odom_ugv_.pose.pose.position.y > YBOUND_H || odom_ugv_.pose.pose.position.y < YBOUND_L) {
		temp = true;
	}
	return temp;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "UGV Tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;
	
	ros::param::get("~topic", topic);
	ros::param::get("~w_frame", world);
	ros::param::get("~ugv_odom", ugv);

	wpt_pub = nh.advertise<asctec_msgs::MinAccelCmd>(topic + "/waypoints", 10);
	pos_pub = nh.advertise<asctec_msgs::PositionCmd>(topic + "/position_cmd", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(topic + "/border", 10);

	status_sub = nh.subscribe(topic + "/status", 10, statusCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);
	quad_sub = nh.subscribe(topic + "/odom", 10, quadCallback);
	ugv_sub = nh.subscribe(ugv, 10, ugvCallback);

	ROS_INFO("Running: UGV Tracker");

	while(ros::ok()) {
		ros::spinOnce();
		showBorder(outBorder(), BORDER_TOP);

		switch(state) {
			case 0:
				//Check exit conditions
				if(flying && isDone) {
					state = 1;			
				}
				break;

			case 1:
				//Check exit conditions
				if(isDone) {
					float tTravel = sqrt(pow(odom_quad_.pose.pose.position.x,2) + pow(odom_quad_.pose.pose.position.y,2) + pow((1 - odom_quad_.pose.pose.position.z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 0.0, 0.0, 1.0, 0.0);

					ROS_INFO("Taking off to 0.0, 0.0, 1.0, time of travel: %f", tTravel);
					state = 2;
				}
				break;

			case 2:
				//Check exit conditions
				if(isDone && !flying) {
					float tTravel = sqrt(pow(odom_quad_.pose.pose.position.x,2) + pow(odom_quad_.pose.pose.position.y,2) + pow(odom_quad_.pose.pose.position.z,2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendLandTrajectory(tTravel);

					ROS_INFO("Landing at 0.0, 0.0, 0.0, time of travel: %f", tTravel);
					state = 0;

				}else if(tracking && isDone) {
					float xNew = limit(odom_ugv_.pose.pose.position.x, XBOUND_H, XBOUND_L);
					float yNew = limit(odom_ugv_.pose.pose.position.y, YBOUND_H, YBOUND_L);
					float yawNew = limit(odom_ugv_.pose.pose.orientation.z, M_PI, -M_PI);

					float tTravel = sqrt(pow((xNew - odom_quad_.pose.pose.position.x),2) + pow((yNew - odom_quad_.pose.pose.position.y),2)) / maxV;
	
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, xNew, yNew, 1.0, 0.0);		//Yaw controller needs to be tuned!!

					ROS_INFO("Begin tracking of UGV");
					state = 3;
				}
				break;

			case 3:
				if(isDone) {
					float xNew = limit(odom_ugv_.pose.pose.position.x, XBOUND_H, XBOUND_L);
					float yNew = limit(odom_ugv_.pose.pose.position.y, YBOUND_H, YBOUND_L);
					float vxNew = 0.0;
					float vyNew = 0.0;
					if(!outUGVBorderX()) {
						vxNew = odom_ugv_.twist.twist.linear.x;
					}
					if(!outUGVBorderY()) {
						vyNew = odom_ugv_.twist.twist.linear.y;
					}
					float yawNew = limit(odom_ugv_.pose.pose.orientation.z, M_PI, -M_PI);
					sendPoint(xNew, yNew, vxNew, vyNew, 1.0, 0.0);										//Yaw controller needs to be tuned!!
				}

				//Check exit conditions
				if(!tracking) {
					ROS_INFO("Halting tracking of UGV");		
					state = 2;
				}
				break;		
		}
		loop_rate.sleep();
	}
}
