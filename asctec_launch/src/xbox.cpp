#include <ros/ros.h>
#include <asctec_msgs/PositionCmd.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#define RB 5
#define LB 4

#define LSH 0
#define LSV 1
#define RSH 2
#define RSV 3

ros::Publisher cmd_pub;
asctec_msgs::PositionCmd cmd;
nav_msgs::Odometry odom;

/* -------------------- callbacks -------------------- */
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons[RB]) {
		// change position cmd in z
		cmd.position.z = odom.pose.pose.position.z + 0.5*msg->axes[LSV];
	}

	if (msg->buttons[LB]) {
		// change position cmd in x-y-yaw
		cmd.position.x = odom.pose.pose.position.x + 0.5*msg->axes[RSV];
		cmd.position.y = odom.pose.pose.position.y + 0.5*msg->axes[RSH];
		tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
				 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

		double d1, d2, yaw;
		tf::Matrix3x3(q).getRPY(d1, d2, yaw);
		cmd.yaw[0] = yaw + 0.3*msg->axes[LSH];
	}
	
	if (msg->buttons[LB] || msg->buttons[RB]) {
		cmd_pub.publish(cmd);
		ROS_INFO("Updated: %.02f, %.02f, %.02f, %.02f", cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw[0]);
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "xbox");
	ros::NodeHandle nh;
  
	/* -------------------- Publishers, and Subscribers -------------------- */
	cmd_pub = nh.advertise<asctec_msgs::PositionCmd>(ros::this_node::getNamespace()+"/position_cmd", 10);
	ros::Subscriber joy_sub = nh.subscribe(ros::this_node::getNamespace()+"/joy", 1, joyCallback);
	ros::Subscriber odom_sub = nh.subscribe(ros::this_node::getNamespace()+"/odom", 1, odomCallback);

	ros::spin();
	return 0;
}
