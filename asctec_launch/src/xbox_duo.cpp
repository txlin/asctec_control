#include <ros/ros.h>
#include <asctec_msgs/PositionCmd.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#define XB 0
#define RB 5
#define LB 4

#define LSH 0
#define LSV 1
#define RSH 2
#define RSV 3

bool quad = true;
ros::Publisher cmd_pub[2];
asctec_msgs::PositionCmd cmd[2];
nav_msgs::Odometry odom[2];

/* -------------------- callbacks -------------------- */
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons[XB]) {
		quad = !quad;
		ROS_INFO("Switched quadrotors");
	}

	if (msg->buttons[RB]) {
		// change position cmd in z
		cmd[quad].position.z = odom[quad].pose.pose.position.z + 0.5*msg->axes[LSV];
	}

	if (msg->buttons[LB]) {
		// change position cmd in x-y-yaw
		cmd[quad].position.x = odom[quad].pose.pose.position.x + 0.5*msg->axes[RSV];
		cmd[quad].position.y = odom[quad].pose.pose.position.y + 0.5*msg->axes[RSH];
		tf::Quaternion q(odom[quad].pose.pose.orientation.x, odom[quad].pose.pose.orientation.y,
				 odom[quad].pose.pose.orientation.z, odom[quad].pose.pose.orientation.w);

		double d1, d2, yaw;
		tf::Matrix3x3(q).getRPY(d1, d2, yaw);
		cmd[quad].yaw[0] = yaw + 0.3*msg->axes[LSH];
	}
	
	if (msg->buttons[LB] || msg->buttons[RB]) {
		cmd_pub[quad].publish(cmd[quad]);
		ROS_INFO("Updated %i: %.02f, %.02f, %.02f, %.02f", quad, cmd[quad].position.x, cmd[quad].position.y, cmd[quad].position.z, cmd[quad].yaw[0]);
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom[0] = *msg;
}

void odom2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom[1] = *msg;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "xbox");
	ros::NodeHandle nh;
  
	/* -------------------- Publishers, and Subscribers -------------------- */
	cmd_pub[0] = nh.advertise<asctec_msgs::PositionCmd>(ros::this_node::getNamespace()+"/position_cmd", 10);
	cmd_pub[1] = nh.advertise<asctec_msgs::PositionCmd>("/asctec2/position_cmd", 10);
	ros::Subscriber joy_sub = nh.subscribe(ros::this_node::getNamespace()+"/joy", 1, joyCallback);
	ros::Subscriber ff_odom_sub = nh.subscribe(ros::this_node::getNamespace()+"/odom", 1, odomCallback);
	ros::Subscriber hb_odom_sub = nh.subscribe("/asctec2/odom", 1, odom2Callback);

	ros::spin();
	return 0;
}
