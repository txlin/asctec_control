#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int rollover = 0;
ros::Time past;
ros::Publisher odom_pub;
nav_msgs::Odometry odom;

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
	double dt = msg->header.stamp.toSec() - past.toSec();
	past = msg->header.stamp;

	odom.header = msg->header;
	odom.twist.twist.linear.x = (msg->transform.translation.x-odom.pose.pose.position.x)/dt;
	odom.twist.twist.linear.y = (msg->transform.translation.y-odom.pose.pose.position.y)/dt;
	odom.twist.twist.linear.z = (msg->transform.translation.z-odom.pose.pose.position.z)/dt;

	nav_msgs::Odometry dummy;
	dummy.pose.pose.orientation = msg->transform.rotation;
	tf::Pose pose;
  tf::poseMsgToTF(dummy.pose.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());
	tf::poseMsgToTF(odom.pose.pose, pose);
  double yaw2 = tf::getYaw(pose.getRotation());
	if (yaw - yaw2 > M_PI)	{
		rollover++;
	}else if (yaw - yaw2 < -M_PI) {
		rollover--;
	}else {
		rollover = 0;
	}
	yaw2 += 2*M_PI*rollover;
	odom.twist.twist.angular.z = (yaw-yaw2)/dt;

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

	std::string frame, world;
	ros::param::get("~frame", frame);
	ros::param::get("~world", world);

	odom_pub = n.advertise<nav_msgs::Odometry>(ros::this_node::getNamespace()+"/odom", 10);
	ros::Subscriber vicon_sub = n.subscribe(frame, 10, viconCallback);

	tf::StampedTransform transform;
	tf::TransformListener listener;
	listener.waitForTransform(world, frame, ros::Time(0), ros::Duration(3.0));
	listener.lookupTransform(world, frame, ros::Time(0), transform);

	odom.pose.pose.position.x = transform.getOrigin().x();
	odom.pose.pose.position.y = transform.getOrigin().y();
	odom.pose.pose.position.z = transform.getOrigin().z();
	past = transform.stamp_;
	
	ros::spin();
	return 0;
}
