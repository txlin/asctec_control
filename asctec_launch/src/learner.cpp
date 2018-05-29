#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/SICmd.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <math.h>

ros::Publisher wpt_pub, si_pub, pos_pub;
bool isDone = true, dead_switch = false, land = false;

float v_bar, x_bar, y_bar, z_bar;
int r_axis, p_axis, y_axis, t_axis;
int d_switch, l_switch;

asctec_msgs::SICmd user_cmd, controller_cmd;
nav_msgs::Odometry odom_;

bool isVelSafe()
{
	float vel_norm = sqrt(pow(odom_.twist.twist.linear.x,2) + pow(odom_.twist.twist.linear.y, 2) + pow(odom_.twist.twist.linear.z, 2));
	return (vel_norm < v_bar);
}

bool isPosSafe()
{
	return (odom_.pose.pose.position.x < x_bar && 
		odom_.pose.pose.position.x > -x_bar && 
		odom_.pose.pose.position.y < y_bar && 
		odom_.pose.pose.position.y > -y_bar && 
		odom_.pose.pose.position.z > -0.1 && 
		odom_.pose.pose.position.z < z_bar);	
	
}

void sendLandTrajectory(float time) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 1.0;
	cmd.yaw[0] = 0;
	cmd.lock_yaw = true;
	cmd.time = time;

	wpt_pub.publish(cmd);
	isDone = false;
	ROS_INFO("Returning to 0.0, 0.0, 1.0, time: %.02f", time);
}

void sendTrajectory(float time, float x, float y, float z, float yaw) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.yaw[0] = yaw;
	cmd.lock_yaw = true;
	cmd.time = time;

	wpt_pub.publish(cmd);
	isDone = false;
	ROS_INFO("Moving to %.02f, %.02f, %.02f, time: %.02f", x, y, z, time);
}

void siCallback(const asctec_msgs::SICmd::ConstPtr &msg) 
{
	controller_cmd = *msg;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	user_cmd.roll = -0.5*msg->axes[r_axis];
	user_cmd.pitch = -0.5*msg->axes[p_axis];
	user_cmd.yaw = 0.5*msg->axes[y_axis];	
	user_cmd.thrust = 0.25*(0.4 + 0.4*msg->axes[t_axis]) + 0.3;
	user_cmd.cmd[0] = user_cmd.cmd[1] = user_cmd.cmd[2] = user_cmd.cmd[3] = true;
	dead_switch = msg->buttons[d_switch];
	land = isDone && (land || msg->buttons[l_switch]);
}

void statusCallback(const std_msgs::Bool::ConstPtr &msg)
{
	isDone = msg->data;
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

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
	odom_ = *msg;
	static nav_msgs::Odometry safeOdom;
	asctec_msgs::SICmd cmd = controller_cmd;
	
	if(land) {
		sendTrajectory(8, 0, 0, 1, 0);
		sendTrajectory(3, 0, 0, 0, 0);
		land = false;
		return;
	}
	if (isPosSafe() && isDone) {
		if (isVelSafe()) {
			safeOdom = *msg;
			if(dead_switch) {
				cmd = user_cmd;
				asctec_msgs::PositionCmd dummy_cmd;
				dummy_cmd.position.x = safeOdom.pose.pose.position.x;
				dummy_cmd.position.y = safeOdom.pose.pose.position.y;
				dummy_cmd.position.z = safeOdom.pose.pose.position.z;
				dummy_cmd.header.stamp = ros::Time::now();
				pos_pub.publish(dummy_cmd);
			}
		}
		else{
			// velocity action
			float tTravel = sqrt(pow(safeOdom.pose.pose.position.x,2) + pow(safeOdom.pose.pose.position.y,2) + pow(safeOdom.pose.pose.position.z,2)) / v_bar;
			tTravel = limit(tTravel, 10, 1);
			sendTrajectory(tTravel,  safeOdom.pose.pose.position.x, safeOdom.pose.pose.position.y, safeOdom.pose.pose.position.z, 0);	
		}

	}else if (!isPosSafe() && isDone){
		float tTravel = sqrt(pow(odom_.pose.pose.position.x,2) + pow(odom_.pose.pose.position.y,2) + pow(odom_.pose.pose.position.z - 1,2)) / v_bar;
		tTravel = limit(tTravel, 10, 1);
		sendLandTrajectory(tTravel);
	}

	si_pub.publish(cmd);
}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "learner");
	ros::NodeHandle nh;
	
	ros::Subscriber odom_sub = nh.subscribe(ros::this_node::getNamespace() + "/odom", 1, odomCallback);
	ros::Subscriber joy_sub = nh.subscribe(ros::this_node::getNamespace() + "/joy", 1, joyCallback);
	ros::Subscriber status_sub = nh.subscribe(ros::this_node::getNamespace() + "/status", 1, statusCallback);
	ros::Subscriber si_sub = nh.subscribe(ros::this_node::getNamespace() + "/cmd_si", 1, siCallback);

	wpt_pub = nh.advertise<asctec_msgs::WaypointCmd>(ros::this_node::getNamespace()+"/waypoints", 10);
	si_pub = nh.advertise<asctec_msgs::SICmd>(ros::this_node::getNamespace()+"/si_remap", 10);
	pos_pub = nh.advertise<asctec_msgs::PositionCmd>(ros::this_node::getNamespace()+"/position_cmd", 10);

	ros::param::param<float>("~bounds/velocity", v_bar, 0.2);
	ros::param::param<float>("~bounds/x", x_bar, 1.0);
	ros::param::param<float>("~bounds/y", y_bar, 1.5);
	ros::param::param<float>("~bounds/z", z_bar, 1.5);
	ros::param::param<int>("~axis/roll", r_axis, 2);
	ros::param::param<int>("~axis/pitch", p_axis, 3);
	ros::param::param<int>("~axis/yaw", y_axis, 0);
	ros::param::param<int>("~axis/thrust", t_axis, 1);
	ros::param::param<int>("~dead_man_switch", d_switch, 5);
	ros::param::param<int>("~land_switch", l_switch, 4);

	ros::spin();
	return 0;

}
