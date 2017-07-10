#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

static std::string mesh_resource = "package://mesh_visualization/meshes/hummingbird.mesh";
static ros::Publisher pub_vis;
static visualization_msgs::Marker marker_quad;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  marker_quad.header = msg->header;
	marker_quad.pose = msg->pose.pose;
  pub_vis.publish(marker_quad);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mesh_visualization");
  ros::NodeHandle nh;

	ros::param::get("~mesh_resource", mesh_resource);

  ros::Subscriber odom_sub = nh.subscribe(ros::this_node::getNamespace()+"/odom", 10, odomCallback);
  pub_vis = nh.advertise<visualization_msgs::Marker>(ros::this_node::getNamespace()+"/asctec_viz", 10);

	marker_quad.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker_quad.color.a = 0.7;
	marker_quad.color.r = 0.3;
	marker_quad.color.g = 0.0;
	marker_quad.color.b = 0.9;
	marker_quad.scale.x = 1;
	marker_quad.scale.y = 1;
	marker_quad.scale.z = 1;
	marker_quad.ns = "hbird_mesh";
	marker_quad.id = 0;
	marker_quad.mesh_resource = mesh_resource;

  ros::spin();
  return 0;
}
