#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

nav_msgs::Odometry odom1_;

std::string mesh_resource = "package://mesh_visualization/meshes/hummingbird.mesh";
ros::Publisher pub_vis, pub_vis_plan;
visualization_msgs::Marker marker_quad, marker_avoid;
bool obs = false, avoid = false;
float life = 30.0;

void odomCallback1(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom1_ = *msg;
	if (avoid){
		marker_avoid.action = visualization_msgs::Marker::ADD;
		marker_avoid.header = odom1_.header;
		marker_avoid.pose = odom1_.pose.pose;
		marker_avoid.id++;
  		pub_vis.publish(marker_avoid);
	}

}

void obsCallback(const std_msgs::Bool::ConstPtr &msg)
{
	obs = msg->data;	
}

void avoidCallback(const std_msgs::Bool::ConstPtr &msg)
{
	avoid = msg->data;	
}

void adaptCallback(const std_msgs::Bool::ConstPtr &msg)
{
	if (msg->data){
		marker_quad.action = visualization_msgs::Marker::ADD;
		marker_quad.header = odom1_.header;
		marker_quad.pose = odom1_.pose.pose;
		marker_quad.id++;
  		pub_vis_plan.publish(marker_quad);	
		ROS_INFO("Adapt signal received! %i", marker_quad.id);	
	}

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "dynamic_obs_viz");
	ros::NodeHandle nh;

	std::string group_ns1;

	//ros::param::get("~group_ns1", group_ns1);

	ros::Subscriber odom_sub = nh.subscribe("asctec1/odom", 10, odomCallback1);
	ros::Subscriber adapt_sub = nh.subscribe("asctec1/adapt", 10, adaptCallback);
	ros::Subscriber obs_sub = nh.subscribe("asctec1/obs", 10, obsCallback);
	ros::Subscriber avoid_sub = nh.subscribe("asctec1/avoid", 10, avoidCallback);

  	pub_vis = nh.advertise<visualization_msgs::Marker>("asctec1/asctec_avoid", 20, true);
	pub_vis_plan = nh.advertise<visualization_msgs::Marker>("asctec1/asctec_plan", 20, true);

	marker_quad.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker_quad.color.a = 0.7;
	marker_quad.color.r = 1.0;
	marker_quad.color.g = 0.3;
	marker_quad.color.b = 0.0;
	marker_quad.scale.x = 1;
	marker_quad.scale.y = 1;
	marker_quad.scale.z = 1;
	marker_quad.ns = "hbird_mesh_history";
	marker_quad.id = 0;
	marker_quad.frame_locked = true;
	marker_quad.lifetime = ros::Duration(life);		
	marker_quad.mesh_resource = mesh_resource;

	marker_avoid.type = visualization_msgs::Marker::CUBE;
	marker_avoid.ns = "trajectory_history";
	marker_avoid.id = 0;
	marker_avoid.scale.x = 0.0125;
	marker_avoid.scale.y = 0.0125;
	marker_avoid.scale.z = 0.0125;
	marker_avoid.color.a = 0.65;
	marker_avoid.color.g = 0.0;
	marker_avoid.color.r = 1.0;
	marker_avoid.color.b = 1.0;
	marker_avoid.lifetime = ros::Duration(life);
	marker_avoid.header.frame_id = "/odom";
	pub_vis.publish(marker_avoid);

	ros::spin();
	return 0;
}
