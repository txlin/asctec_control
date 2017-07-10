#include <ros/ros.h>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/SICmd.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

ros::Publisher odom_pub;
geometry_msgs::TransformStamped odom;
std::string frame = "/asctec_dummy"; 
std::string world = "/odom";

tf::TransformBroadcaster *br;
tf::Transform transform;

void cmdCallback(const asctec_msgs::PositionCmd::ConstPtr &msg)
{
	odom.transform.translation.x = msg->position.x;
	odom.transform.translation.y = msg->position.y;
	odom.transform.translation.z = msg->position.z;
	if(!isnan(msg->yaw[0])){
		tf::Quaternion q;
		q.setRPY(odom.transform.rotation.x, odom.transform.rotation.y, msg->yaw[0]);
		tf::quaternionTFToMsg(q,odom.transform.rotation);
	}
}
void siCallback(const asctec_msgs::SICmd::ConstPtr &msg)
{
	if(!isnan(msg->roll) && !isnan(msg->pitch)){
		tf::Quaternion q;
		q.setRPY(msg->roll, -msg->pitch, odom.transform.rotation.z);
		tf::quaternionTFToMsg(q,odom.transform.rotation);
	}
}

void odomCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
	odom = *msg;
	ROS_INFO("Broadcasting dummy position: %f, %f, %f", msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
}

void timerCallback(const ros::TimerEvent& event) 
{
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = world;
	odom.child_frame_id = frame;
  tf::Quaternion q;
	q.setRPY(odom.transform.rotation.x, odom.transform.rotation.y, odom.transform.rotation.z);
  transform.setOrigin(tf::Vector3(odom.transform.translation.x, odom.transform.translation.y, odom.transform.translation.z));
  transform.setRotation(q);
	tf::quaternionTFToMsg(q,odom.transform.rotation);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), world, frame));
	odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dummy_odom");
	ros::NodeHandle n;

	float rate = 40;
	bool track = false;
	ros::param::get("~rate", rate);
	ros::param::get("~track", track);
	ros::param::get("~frame", frame);
	ros::param::get("~world", world);

	odom_pub = n.advertise<geometry_msgs::TransformStamped>(frame, 10); 
	ros::Subscriber odom_sub = n.subscribe(ros::this_node::getNamespace()+"/dummy", 10, odomCallback);
	ros::Subscriber pos_sub, ang_sub;
	if(track) pos_sub = n.subscribe(ros::this_node::getNamespace()+"/position_cmd", 10, cmdCallback);
	if(track) ang_sub = n.subscribe(ros::this_node::getNamespace()+"/cmd_si", 10, siCallback);
	if(track) ROS_INFO("Updating according to position commands");
  ros::Timer timer = n.createTimer(ros::Duration(1/rate), timerCallback);
	timer.start();

	br = new tf::TransformBroadcaster;
	ros::spin();
	return 0;
}
