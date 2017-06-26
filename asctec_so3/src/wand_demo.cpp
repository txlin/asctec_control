#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <asctec_msgs/MinAccelCmd.h>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/SICmd.h>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <string.h>

#define freq 40
#define maxV 0.2

#define XBOUND_H 1.0
#define XBOUND_L -1.0
#define YBOUND_H 1.5
#define YBOUND_L -1.5

#define sXBOUND_H 0.75 * XBOUND_H
#define sXBOUND_L 0.75 * XBOUND_L
#define sYBOUND_H 0.75 * YBOUND_H
#define sYBOUND_L 0.75 * YBOUND_L

#define BORDER_NUMB 2
#define BORDER_TOP 0.8

#define k_ang 0.9
#define ANG_LIMIT M_PI/8

#define kp 
#define kd
#define mg 0.25

using namespace std;

int state = 0;
bool isDone = true;

string world, quad_name, wand_frame, topic;
nav_msgs::Odometry odom_;
asctec_msgs::SICmd nl_cmd;

ros::Publisher wpt_pub, sborder_pub, border_pub, si_pub, cmd_pub;
ros::Subscriber si_sub, status_sub, odom_sub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_ = *msg;
}

void statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void siCallback(const asctec_msgs::SICmd::ConstPtr& msg)
{
  nl_cmd = *msg;
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

asctec_msgs::SICmd * limitXY(asctec_msgs::SICmd *cmd)
{
	if(odom_.pose.pose.position.x > sXBOUND_H) {
		cmd->pitch = limit(cmd->pitch, ANG_LIMIT, 0);
		
		if(odom_.pose.pose.position.y > sYBOUND_H) {
			cmd->roll = limit(cmd->roll, ANG_LIMIT, 0);
			
		}else if(odom_.pose.pose.position.y < sYBOUND_L){
			cmd->roll = limit(cmd->roll, 0, -ANG_LIMIT);
		}

	}else if(odom_.pose.pose.position.x < sXBOUND_L) {
		cmd->roll = limit(cmd->roll, 0, -ANG_LIMIT);

		if(odom_.pose.pose.position.y > sYBOUND_H) {
			cmd->pitch = limit(cmd->pitch, ANG_LIMIT, 0);

		}else if(odom_.pose.pose.position.y < sYBOUND_L){
			cmd->pitch = limit(cmd->pitch, 0, -ANG_LIMIT);
		}
	}
  return cmd;
}

void showSBorder(bool outOf, float z)
{
	visualization_msgs::Marker border;
	geometry_msgs::Point corner;

	border.header.frame_id = world;
	border.header.stamp = ros::Time::now();
	border.id = 2;
	border.action = visualization_msgs::Marker::ADD;
	border.type = visualization_msgs::Marker::LINE_LIST;

	if(outOf) {
		border.color.a = 1.0;
		border.color.r = 1.0;	
		border.color.g = 1.0;
	}else {
		border.color.a = 1.0;
		border.color.g = 1.0;	
	}
			
	border.scale.x = 0.05;
	border.scale.y = 0.05;
	border.scale.z = 0.05;

        /// Soft Border
	corner.x = sXBOUND_L;
	corner.y = sYBOUND_L;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_L;
	corner.y = sYBOUND_L;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = sXBOUND_L;
	corner.y = sYBOUND_H;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_L;
	corner.y = sYBOUND_H;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_H;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_H;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_L;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_L;	
	corner.z = z;
	border.points.push_back(corner);

	for(int i=0; i<=BORDER_NUMB; i++) {

		//Soft Border	
		corner.x = sXBOUND_L;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_L;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_L;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_L;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);
	}
	sborder_pub.publish(border);
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
		border.color.a = 1.0;
		border.color.r = 1.0;	
	}else {
		border.color.a = 1.0;
		border.color.g = 1.0;	
	}
			
	border.scale.x = 0.05;
	border.scale.y = 0.05;
	border.scale.z = 0.05;

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

void sendTrajectory(float x, float y, float z) 
{
	float tTravel = sqrt(pow(x-odom_.pose.pose.position.x,2) + pow(y-odom_.pose.pose.position.y,2) + pow((z-odom_.pose.pose.position.z),2)) / maxV;
	tTravel = limit(tTravel, 10, 1);

	asctec_msgs::MinAccelCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.time = tTravel;

	wpt_pub.publish(cmd);
	isDone = false;
}

void sendBorderTrajectory()
{
	asctec_msgs::MinAccelCmd cmd;
	cmd.reset = true;
	cmd.position.x = odom_.pose.pose.position.x;
	cmd.position.y = odom_.pose.pose.position.y;
	cmd.position.z = 1.0;
	cmd.time = 1.0;

	if(odom_.pose.pose.position.x > XBOUND_H) {
		cmd.position.x = sXBOUND_H;

	}else if(odom_.pose.pose.position.x < XBOUND_L) {
		cmd.position.x = sXBOUND_L;
	}

	if(odom_.pose.pose.position.y > YBOUND_H) {
		cmd.position.y = sYBOUND_H;

	}else if(odom_.pose.pose.position.y < YBOUND_L) {
		cmd.position.y = sYBOUND_L;
	}
	wpt_pub.publish(cmd);
	isDone = false;
}

void sendLandTrajectory() 
{
	float tTravel = sqrt(pow(odom_.pose.pose.position.x,2) + pow(odom_.pose.pose.position.y,2) + pow((1 - odom_.pose.pose.position.z),2)) / maxV;
	tTravel = limit(tTravel, 10, 1);

	asctec_msgs::MinAccelCmd cmd;
	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 1.0;
	cmd.time = tTravel;
	wpt_pub.publish(cmd);

	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 0.0;
	cmd.time = 3;
	wpt_pub.publish(cmd);

	isDone = false;
}

bool outBorder(void)
{
	if(odom_.pose.pose.position.x > XBOUND_H || odom_.pose.pose.position.x < XBOUND_L || odom_.pose.pose.position.y > YBOUND_H || odom_.pose.pose.position.y < YBOUND_L) {
		return true;
	}
	return false;
}

bool outSBorder(void)
{
	if(odom_.pose.pose.position.x > sXBOUND_H || odom_.pose.pose.position.x < sXBOUND_L || odom_.pose.pose.position.y > sYBOUND_H || odom_.pose.pose.position.y < sYBOUND_L) {
		return true;
	}
	return false;
}

asctec_msgs::SICmd * setHRIBehavior(tf::StampedTransform * transform)
{
	tf::Quaternion q = transform->getRotation();
	tf::Matrix3x3 m(q);

	double wYaw, wRoll, wPitch;
	m.getRPY(wRoll, wPitch, wYaw);
  asctec_msgs::SICmd * cmd = new asctec_msgs::SICmd;
  
	cmd->roll = (limit(k_ang*wRoll, ANG_LIMIT, -ANG_LIMIT)) * cos(wYaw) + (-limit(k_ang*wPitch, ANG_LIMIT, -ANG_LIMIT)) * sin(wYaw);
	cmd->pitch = (-limit(k_ang*wRoll, ANG_LIMIT, -ANG_LIMIT)) * sin(wYaw) + (-limit(k_ang*wPitch, ANG_LIMIT, -ANG_LIMIT)) * cos(wYaw);
	return cmd;
}

bool isWandDown(tf::StampedTransform * transform)
{
	if(transform->getOrigin().z() < 0.2) {
		return true;
	}
	return false;
}

bool isWandUp(tf::StampedTransform * transform)
{
	if(transform->getOrigin().z() > 1.5) {
		return true;
	}
	return false;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Wand Tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;

	ros::param::get("~q_name", quad_name);
	ros::param::get("~wand_frame", wand_frame);
	ros::param::get("~w_frame", world);
	ros::param::get("~topic_name", topic);

	si_pub = nh.advertise<asctec_msgs::SICmd>(topic + "/si_remap", 10);
	cmd_pub = nh.advertise<asctec_msgs::PositionCmd>(topic + "/position_cmd", 10);

	wpt_pub = nh.advertise<asctec_msgs::MinAccelCmd>(topic + "/waypoints", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);
	sborder_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/soft_border", 10);

	si_sub = nh.subscribe(topic + "/si_command", 10, siCallback);
	status_sub = nh.subscribe(topic + "/status", 10, statusCallback);
	odom_sub = nh.subscribe(topic + "/odom", 10, odomCallback);

	tf::TransformListener listener;	
	tf::StampedTransform transform;
	listener.waitForTransform(world, wand_frame, ros::Time(0), ros::Duration(3.0));

	ROS_INFO("Running: Wand Listener");

	while(ros::ok()) {
		ros::spinOnce();
		showBorder(outBorder(), BORDER_TOP);
		showSBorder(outSBorder(), BORDER_TOP);

		listener.lookupTransform(world, wand_frame, ros::Time(0), transform);
    asctec_msgs::SICmd new_cmd = nl_cmd;
    
		switch(state) {
			case 0:
			  {
				  //Check exit conditions
				  if(isWandUp(&transform) && isDone) {
					  sendTrajectory(0.0, 0.0, 1.0);
					  ROS_INFO("Taking off to 0.0, 0.0, 1.0");				
					  state = 1;			
				  }
				  break;
        }
        
			case 1:
			  {
			    if(isDone) {
				    new_cmd = *setHRIBehavior(&transform);
				    new_cmd = *limitXY(&new_cmd);
				    new_cmd.thrust = nl_cmd.thrust;
//						new_cmd.thrust = kp*(1.0-odom_.pose.pose.position.z)+kd*(-odom_.twist.twist.linear.z)+mg;
				    new_cmd.yaw = nl_cmd.yaw;
				
				    asctec_msgs::PositionCmd nl_pcmd;
				    nl_pcmd.position.x = odom_.pose.pose.position.x;
				    nl_pcmd.position.y = odom_.pose.pose.position.y;
				    nl_pcmd.position.z = 1.0;
				    cmd_pub.publish(nl_pcmd);
				
				    //Check exit conditions
				    if(isWandDown(&transform)) {
					    ROS_INFO("Landing...");
					    sendLandTrajectory();
					    state = 0;
				    }

				    if(outBorder()) {
					    ROS_INFO("Border breached!");
					    sendBorderTrajectory();
				    }	
			  }
				break;
		  }
		}
		si_pub.publish(new_cmd);
		loop_rate.sleep();
	}
}
