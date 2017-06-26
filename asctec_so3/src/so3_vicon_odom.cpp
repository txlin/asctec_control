#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Time past;
ros::Publisher odom_pub;
nav_msgs::Odometry odom;
std::string q_name, q_frame, w_frame;

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
	double dt = msg->header.stamp.toSec() - past.toSec();
	past = msg->header.stamp;
	odom.header.frame_id = w_frame;
	odom.header.stamp = msg->header.stamp;
	odom.child_frame_id = q_frame;

	odom.twist.twist.linear.x = (msg->transform.translation.x-odom.pose.pose.position.x)/dt;
	odom.twist.twist.linear.y = (msg->transform.translation.y-odom.pose.pose.position.y)/dt;
	odom.twist.twist.linear.z = (msg->transform.translation.z-odom.pose.pose.position.z)/dt;
	odom.pose.pose.position.x = msg->transform.translation.x;
	odom.pose.pose.position.y = msg->transform.translation.y;
	odom.pose.pose.position.z = msg->transform.translation.z;
	odom.pose.pose.orientation = msg->transform.rotation;

	odom_pub.publish(odom);	
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_odom");
	ros::NodeHandle n;

	ros::param::get("~q_name", q_name);
	ros::param::get("~q_frame", q_frame);
	ros::param::get("~w_frame", w_frame);

	odom_pub = n.advertise<nav_msgs::Odometry>(q_name+"/odom", 10);
	ros::Subscriber vicon_sub = n.subscribe(q_frame, 10, viconCallback);

	tf::StampedTransform transform;
	tf::TransformListener listener;
	listener.waitForTransform(w_frame, q_frame, ros::Time(0), ros::Duration(3.0));
	listener.lookupTransform(w_frame, q_frame, ros::Time(0), transform);

	odom.pose.pose.position.x = transform.getOrigin().x();
	odom.pose.pose.position.y = transform.getOrigin().y();
	odom.pose.pose.position.z = transform.getOrigin().z();
	past = transform.stamp_;
	ROS_INFO("Broadcasting %s odom from vicon...", q_name.c_str());
	
	ros::spin();
	return 0;
}
