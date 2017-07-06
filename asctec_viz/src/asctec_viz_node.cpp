#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <asctec_msgs/WaypointCmd.h>
#include <asctec_msgs/PositionCmd.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <string.h>

int idw = 0;
int idt = 0;
double tTime = 0;
float life = 2.5;
static std::string mesh_resource = "package://mesh_visualization/meshes/hummingbird.mesh";

ros::Time last;
visualization_msgs::Marker waypoint, trajectory, marker_quad;
ros::Publisher waypoint_viz;

void waypointCallback(const asctec_msgs::WaypointCmd::ConstPtr& msg)
{
	if(msg->reset) {
		waypoint.action = 3;
		waypoint_viz.publish(waypoint);
		
		trajectory.action = 3;
		waypoint_viz.publish(trajectory);

		idw = 0;
		idt = 0;
		tTime = 0.0;
	}
	tTime -= ros::Time::now().toSec() - last.toSec();
	tTime = std::max(tTime, 0.0);
	last = ros::Time::now();

	waypoint.lifetime = ros::Duration(life+tTime+msg->time);
	waypoint.action = visualization_msgs::Marker::ADD;
	waypoint.pose.position.x = msg->position.x;
	waypoint.pose.position.y = msg->position.y;
	waypoint.pose.position.z = msg->position.z;
	waypoint.header.stamp = ros::Time::now();
	waypoint.id = idw;

	waypoint_viz.publish(waypoint);
	tTime += msg->time;
	idw++;
}

void cmdCallback(const asctec_msgs::PositionCmd::ConstPtr& msg)
{
  tf::Quaternion q;
	q.setRPY(0.0, 0.0, msg->yaw[0]);
	tf::quaternionTFToMsg(q,trajectory.pose.orientation);
	trajectory.action = visualization_msgs::Marker::ADD;
	trajectory.pose.position.x = msg->position.x;
	trajectory.pose.position.y = msg->position.y;
	trajectory.pose.position.z = msg->position.z;
	trajectory.header.stamp = ros::Time::now();
	trajectory.id = idt;
	idt++;
	waypoint_viz.publish(trajectory);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  marker_quad.header = msg->header;
	marker_quad.pose = msg->pose.pose;
  waypoint_viz.publish(marker_quad);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "asctec_viz");
	ros::NodeHandle nh;
	
	std::string topic_name = "/asctec";
	std::string world = "/odom";
	ros::param::get("~topic_name", topic_name);
	ros::param::get("~frame", world);
	ros::param::get("~mesh_resource", mesh_resource);
	ros::param::get("~decay", life);

	waypoint_viz = nh.advertise<visualization_msgs::Marker>(topic_name+"/asctec_viz", 10);

	ros::Subscriber wp = nh.subscribe(topic_name+"/waypoints", 10, waypointCallback);
	ros::Subscriber cmd = nh.subscribe(topic_name+"/position_cmd", 10, cmdCallback);
  ros::Subscriber odom = nh.subscribe(topic_name+"/odom", 10, odomCallback);

	waypoint.header.frame_id = world;
	waypoint.type = visualization_msgs::Marker::SPHERE;
	waypoint.ns = "waypoints";
	waypoint.scale.x = 0.1;
	waypoint.scale.y = 0.1;
	waypoint.scale.z = 0.1;
	waypoint.color.a = 0.6;
	waypoint.color.b = 1.0;
	waypoint.color.r = 1.0;

	trajectory.header.frame_id = world;
	trajectory.type = visualization_msgs::Marker::CUBE;
	trajectory.ns = "trajectory";
	trajectory.id = 0;
	trajectory.scale.x = 0.0125;
	trajectory.scale.y = 0.0125;
	trajectory.scale.z = 0.0125;
	trajectory.color.a = 0.65;
	trajectory.color.g = 1.0;
	trajectory.lifetime = ros::Duration(life);
	last = ros::Time::now();

	marker_quad.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker_quad.color.a = 0.6;
	marker_quad.color.r = 0.3;
	marker_quad.color.g = 0.2;
	marker_quad.color.b = 0.7;
	marker_quad.scale.x = 1;
	marker_quad.scale.y = 1;
	marker_quad.scale.z = 1;
	marker_quad.ns = "hbird_mesh";
	marker_quad.id = 0;
	marker_quad.mesh_resource = mesh_resource;

	ROS_INFO("Listening on topic set: %s", topic_name.c_str());
	ros::spin();
	return 0;
}


