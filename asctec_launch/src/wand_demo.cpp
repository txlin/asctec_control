#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <asctec_msgs/WaypointCmd.h>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/SICmd.h>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <string.h>

#define freq 40
#define maxV 0.5
#define maxA 1.0	//		m/s^2

#define XBOUND_H 1.0
#define XBOUND_L -1.0
#define YBOUND_H 1.5
#define YBOUND_L -1.5

#define sXBOUND_H 0.75 * XBOUND_H
#define sXBOUND_L 0.75 * XBOUND_L
#define sYBOUND_H 0.75 * YBOUND_H
#define sYBOUND_L 0.75 * YBOUND_L

#define BORDER_NUMB 1
#define BORDER_TOP 0.5

#define k_ang 0.9
#define ANG_LIMIT M_PI/8

int state = 0;
bool isDone = true;

std::string world, wand;
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
		border.color.a = 0.7;
		border.color.r = 1.0;	
		border.color.g = 1.0;
	}else {
		border.color.a = 0.7;
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
		border.color.a = 0.7;
		border.color.r = 1.0;	
	}else {
		border.color.a = 0.7;
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
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;

	float tx = odom_.twist.twist.linear.x/maxA;
	float ty = odom_.twist.twist.linear.y/maxA;
	cmd.time = sqrt(pow(tx,2)+pow(ty,2));
	cmd.time += sqrt(pow(x-odom_.pose.pose.position.x,2) + pow(y-odom_.pose.pose.position.y,2) + pow((z-odom_.pose.pose.position.z),2)) / maxV;

	wpt_pub.publish(cmd);
	isDone = false;
}

void sendBorderTrajectory()
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = std::min(odom_.pose.pose.position.x, sXBOUND_H);
	cmd.position.x = std::max(cmd.position.x, sXBOUND_L);
	cmd.position.y = std::min(odom_.pose.pose.position.y, sYBOUND_H);
	cmd.position.y = std::max(cmd.position.y, sYBOUND_L);
	if((sXBOUND_H-odom_.pose.pose.position.x) < 0) {
		cmd.velocity.x = -maxV;

	}else if((sXBOUND_L-odom_.pose.pose.position.x) > 0) { 
		cmd.velocity.x = maxV;
	}
	if((sYBOUND_H-odom_.pose.pose.position.y) < 0) {
		cmd.velocity.y = -maxV;

	}else if((sYBOUND_L-odom_.pose.pose.position.y) > 0) { 
		cmd.velocity.y = maxV;
	}
	cmd.position.z = 1.0;

	float tx = odom_.twist.twist.linear.x/maxA;
	float ty = odom_.twist.twist.linear.y/maxA;
	cmd.time = sqrt(pow(tx,2)+pow(ty,2));
	cmd.time += sqrt(pow(cmd.position.x-odom_.pose.pose.position.x,2)+pow(cmd.position.y-odom_.pose.pose.position.y,2))/maxV;
	cmd.reset = true;

	wpt_pub.publish(cmd);
	isDone = false;
}

void sendLandTrajectory() 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.reset = true;
	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 1.0;
	float tx = odom_.twist.twist.linear.x/maxA;
	float ty = odom_.twist.twist.linear.y/maxA;
	cmd.time = sqrt(pow(tx,2)+pow(ty,2));
	cmd.time += sqrt(pow(odom_.pose.pose.position.x,2) + pow(odom_.pose.pose.position.y,2) + pow((1-odom_.pose.pose.position.z),2)) / maxV;
	wpt_pub.publish(cmd);

	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 0.0;
	cmd.reset = false;
	cmd.time = 3;
	wpt_pub.publish(cmd);

	isDone = false;
}

bool outBorder(void)
{
	return odom_.pose.pose.position.x > XBOUND_H || odom_.pose.pose.position.x < XBOUND_L || odom_.pose.pose.position.y > YBOUND_H || odom_.pose.pose.position.y < YBOUND_L;
}

bool outSBorder(void)
{
	return odom_.pose.pose.position.x > sXBOUND_H || odom_.pose.pose.position.x < sXBOUND_L || odom_.pose.pose.position.y > sYBOUND_H || odom_.pose.pose.position.y < sYBOUND_L;
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
	return transform->getOrigin().z() < 0.3;
}

bool isWandUp(tf::StampedTransform * transform)
{
	return transform->getOrigin().z() > 1.5;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Wand Tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;

	ros::param::get("~wand", wand);
	ros::param::get("~world", world);

	si_pub = nh.advertise<asctec_msgs::SICmd>(ros::this_node::getNamespace()+"/si_remap", 10);
	cmd_pub = nh.advertise<asctec_msgs::PositionCmd>(ros::this_node::getNamespace()+"/position_cmd", 10);

	wpt_pub = nh.advertise<asctec_msgs::WaypointCmd>(ros::this_node::getNamespace()+"/waypoints", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(ros::this_node::getNamespace()+"/border", 10);
	sborder_pub = nh.advertise<visualization_msgs::Marker>(ros::this_node::getNamespace()+"/soft_border", 10);

	si_sub = nh.subscribe(ros::this_node::getNamespace()+"/cmd_si", 1, siCallback);
	status_sub = nh.subscribe(ros::this_node::getNamespace()+"/status", 10, statusCallback);
	odom_sub = nh.subscribe(ros::this_node::getNamespace()+"/odom", 10, odomCallback);

	tf::TransformListener listener;	
	tf::StampedTransform transform;
	listener.waitForTransform(world, wand, ros::Time(0), ros::Duration(3.0));
	ros::Duration(1.0).sleep();

	ROS_INFO("Running: Wand Listener");

	while(ros::ok()) {
		ros::spinOnce();
		showBorder(outBorder(), BORDER_TOP);
		showSBorder(outSBorder(), BORDER_TOP);

		listener.lookupTransform(world, wand, ros::Time(0), transform);
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
				    new_cmd.yaw = nl_cmd.yaw;
						new_cmd.cmd[0] = new_cmd.cmd[1] = new_cmd.cmd[2] = new_cmd.cmd[3] = true;

				    asctec_msgs::PositionCmd nl_pcmd;
				    nl_pcmd.position.x = odom_.pose.pose.position.x;
				    nl_pcmd.position.y = odom_.pose.pose.position.y;
//				    nl_pcmd.velocity.x = odom_.twist.twist.linear.x;
//				    nl_pcmd.velocity.y = odom_.twist.twist.linear.y;
				    nl_pcmd.position.z = 1.0;
				    cmd_pub.publish(nl_pcmd);
				
				    //Check exit conditions
				    if(isWandDown(&transform)) {
					    ROS_INFO("Landing...");
					    sendLandTrajectory();
					    state = 0;

				    }else if(outBorder()) {
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
