#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Time past;
ros::Publisher odom_pub;
nav_msgs::Odometry odom;
std::string ugv_name, ugv_frame, w_frame;

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
	static int rollover = 0;
	double dt = msg->header.stamp.toSec() - past.toSec();
	past = msg->header.stamp;
	odom.header.frame_id = w_frame;
	odom.header.stamp = msg->header.stamp;
	odom.child_frame_id = ugv_frame;

	odom.twist.twist.linear.x = (msg->transform.translation.x-odom.pose.pose.position.x)/dt;
	odom.twist.twist.linear.y = (msg->transform.translation.y-odom.pose.pose.position.y)/dt;
	odom.twist.twist.linear.z = (msg->transform.translation.z-odom.pose.pose.position.z)/dt;
	odom.pose.pose.position.x = msg->transform.translation.x;
	odom.pose.pose.position.y = msg->transform.translation.y;
	odom.pose.pose.position.z = msg->transform.translation.z;
	odom.pose.pose.orientation = msg->transform.rotation;

	tf::Pose pose;
  tf::poseMsgToTF(odom.pose.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());
	static double yaw0 = yaw;

	if (yaw - yaw0 < -M_PI) {
		rollover++;

	}else if (yaw - yaw0 > M_PI) {
		rollover--;
	}
	yaw += 2*M_PI*rollover;
	yaw0 = yaw-2*M_PI*rollover;

	tf::Quaternion q;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
	tf::Matrix3x3 m(q);

	double r,p,dy;
	m.getRPY(r,p,dy);
	q.setRPY(r,p,yaw);
	tf::quaternionTFToMsg(q, odom.pose.pose.orientation);

	odom_pub.publish(odom);	
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ugv_odom");
	ros::NodeHandle n;

	ros::param::get("~ugv_name", ugv_name);
	ros::param::get("~ugv_frame", ugv_frame);
	ros::param::get("~world", w_frame);

	odom_pub = n.advertise<nav_msgs::Odometry>(ugv_name+"/odom", 10);
	ros::Subscriber vicon_sub = n.subscribe(ugv_frame, 10, viconCallback);

	tf::StampedTransform transform;
	tf::TransformListener listener;
	listener.waitForTransform(w_frame, ugv_frame, ros::Time(0), ros::Duration(3.0));
	listener.lookupTransform(w_frame, ugv_frame, ros::Time(0), transform);

	odom.pose.pose.position.x = transform.getOrigin().x();
	odom.pose.pose.position.y = transform.getOrigin().y();
	odom.pose.pose.position.z = transform.getOrigin().z();
	past = transform.stamp_;
	
	ros::spin();
	return 0;
}
