#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <asctec_msgs/WaypointCmd.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <string.h>

#define obsNumb 10

#define f_x 0.0
#define f_y 2.0

#define repel_pts 3	  //always integer
#define repel 0.8       //radius
#define repel_t 5.0
#define repel_rings 2
#define end_off 0.5

#define real_r  0.14     //11 inches diameter
#define quad_r  0.27     //21 inches diameter

#define freq 10
#define maxV 0.5
#define maxVZ 0.2
#define maxA 0.3

using namespace std;

struct obs {
	string frame;
	float x;
	float y;
	float r;

	obs():frame(""), x(0), y(0), r(0) {}
}OBS;

struct Isection {

	double t;
	int spline;
	Isection * next;
	
}ISECTION;

enum states {
waitStart,
waitHover,
waitA,
waitB
};

states state = waitStart;

bool isDone = true;
bool nextTraj = false;
bool isFlying = false;

obs * obs_ptr = new obs[obsNumb];
int points;

string world, topic, obs_frame;
nav_msgs::Odometry odom_;

ros::Publisher traj_pub, tiki_pub;
ros::Subscriber traj_sub, joy_sub, odom_sub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_ = *msg;
}

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0]) {
		isFlying = !isFlying;

	}else if(msg->buttons[1]) {
		if(state != waitStart) {
			nextTraj = true;
		}
	}
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

float getTravTime(float xst, float x, float yst, float y, float zst, float z)
{
	float tTravel = sqrt(pow(xst - x,2) + pow(yst - y,2) + pow(zst - z,2)) / maxV;
	return limit(tTravel, 12, 0.1);
}

float getTravTimeZ(float xst, float x, float yst, float y, float zst, float z)
{
	float tTravel = sqrt(pow(xst - x,2) + pow(yst - y,2) + pow(zst - z,2)) / maxVZ;
	return limit(tTravel, 20, 0.1);
}

void sendAvoidABNew(float wait, float x, float y, float z, float yaw, tf::StampedTransform * transform)
{
	/* Aversion trajectory from -y to +y
	 * Default trajectory to inject from is x = 0 from -1.5 <= y <= 1.5
	 * gamma: angle portion of avoidance circle
	 * theta: angle of first point relative to trajectory
	 * avoidR: avoid radius length
	 */

	float obsx = transform->getOrigin().x();
	float obsy = transform->getOrigin().y();
	float theta = asin(abs(obsx)/repel);
	float gamma = M_PI - 2*theta;

	asctec_msgs::WaypointCmd cmd, last;
	//Set first point of obstacle radius
	cmd.position.x = 0.0;
	cmd.position.y = limit(obsy - repel*cos(theta) - end_off,0,odom_.pose.pose.position.y);
	cmd.velocity.x = 0.0;
	cmd.velocity.y = maxV;
	cmd.accel.x = 0.0;
	cmd.accel.y = 0.0;
	cmd.position.z = 1.0;
	float a_t = abs(odom_.twist.twist.linear.y-cmd.velocity.y)/maxA;
	cmd.time = getTravTime(odom_.pose.pose.position.x, cmd.position.x, odom_.pose.pose.position.y, cmd.position.y, odom_.pose.pose.position.z, cmd.position.z)+a_t;
	traj_pub.publish(cmd);
	last = cmd;

	//Set mid points of obstacle radius
	if(obsx >= 0) {
		//Set second point of obstacle radius
		/*cmd.position.x = -0.1*cos(theta);
		cmd.position.y = obsy - repel*cos(theta) - end_off/8;
		cmd.velocity.x = 0.0;
		cmd.velocity.y = maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.position.x = obsx - repel*sin(newAng);
			cmd.velocity.x = -maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.accel.x = -maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.y = obsy - repel*cos(newAng);
			cmd.velocity.y = maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.accel.y = -maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.z = 1.0;
			cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
			last = cmd;
			traj_pub.publish(cmd);
		}

		//Set second to last point of obstacle radius
		/*cmd.position.x = -0.1*cos(theta);
		cmd.position.y = obsy + repel*cos(theta) + end_off/8;
		cmd.velocity.x = 0.0;
		cmd.velocity.y = maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(last.position.x, cmd.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/

	}else {
		//Set second point of obstacle radius
		/*cmd.position.x = 0.1*cos(theta);
		cmd.position.y = obsy - repel*cos(theta) - end_off/8;
		cmd.velocity.x = 0.0;
		cmd.velocity.y = maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.position.x = obsx + repel*sin(newAng);
			cmd.velocity.x = maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.accel.x = maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.y = obsy - repel*cos(newAng);
			cmd.velocity.y = maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.accel.y = -maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.z = 1.0;
			cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
			traj_pub.publish(cmd);
			last = cmd;
		}

		//Set second to last point of obstacle radius
		/*cmd.position.x = 0.1*cos(theta);
		cmd.position.y = obsy + repel*cos(theta) + end_off/8;
		cmd.velocity.x = 0.0;
		cmd.velocity.y = maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/
	}

	//Set final point of obstacle radius
	cmd.position.x = 0.0;
	cmd.position.y = obsy + repel*cos(theta) + end_off;
	cmd.velocity.x = 0.0;
	cmd.velocity.y = maxV;
	cmd.accel.x = 0.0;
	cmd.accel.y = 0.0;
	cmd.position.z = 1.0;
	cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
	traj_pub.publish(cmd);
	last = cmd;

	//Set end point of path
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.velocity.x = 0.0;
	cmd.velocity.y = 0.0;
	cmd.accel.x = 0.0;
	cmd.accel.y = 0.0;
	cmd.yaw[0] = yaw;
	a_t = (maxV-cmd.velocity.y)/maxA;
	cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z)+a_t;
	traj_pub.publish(cmd);

	isDone = false;
}

void sendAvoidBANew(float wait, float x, float y, float z, float yaw, tf::StampedTransform * transform)
{
	/* Aversion trajectory from +y to -y
	 * Default trajectory to inject from is x = 0 from -1.5 <= y <= 1.5
	 * gamma: angle portion of avoidance circle
	 * theta: angle of first point relative to trajectory
	 */

	float obsx = transform->getOrigin().x();
	float obsy = transform->getOrigin().y();
	float theta = asin(abs(obsx)/repel);
	float gamma = M_PI - 2*theta;

	asctec_msgs::WaypointCmd cmd, last;
	//Set first point of obstacle radius
	cmd.position.x = 0.0;
	cmd.position.y = limit(obsy + repel*cos(theta) + end_off,odom_.pose.pose.position.y,0);
	cmd.velocity.x = 0.0;
	cmd.velocity.y = -maxV;
	cmd.position.z = 1.0;
	float a_t = abs(odom_.twist.twist.linear.y-cmd.velocity.y)/maxA;
	cmd.time = getTravTime(odom_.pose.pose.position.x, cmd.position.x, odom_.pose.pose.position.y, cmd.position.y, odom_.pose.pose.position.z, cmd.position.z)+a_t;
	traj_pub.publish(cmd);
	last = cmd;

	//Set mid points of obstacle radius
	if(obsx >= 0) {
		//Set second point of obstacle radius
		/*cmd.position.x = -0.1*cos(theta);
		cmd.position.y = obsy + repel*cos(theta) + end_off/8;
		cmd.velocity.x = 0.0;
		cmd.velocity.y = -maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.position.x = obsx - repel*sin(newAng);
			cmd.velocity.x = -maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.accel.x = -maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.y = obsy + repel*cos(newAng);
			cmd.velocity.y = -maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.accel.y = maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.z = 1.0;
			cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
			traj_pub.publish(cmd);
			last = cmd;
		}

		//Set second to last point of obstacle radius
		/*cmd.position.x = -0.1*cos(theta);
		cmd.position.y = obsy - repel*cos(theta) - end_off/8;
		cmd.velocity.x = 0.0;
		cmd.velocity.y = -maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/

	}else {
		//Set second point of obstacle radius
		/*cmd.position.x = 0.1*cos(theta);
		cmd.position.y = obsy + repel*cos(theta) + end_off/8;
		cmd.velocity.y = -maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.position.x = obsx + repel*sin(newAng);
			cmd.velocity.x = maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.accel.x = maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.y = obsy + repel*cos(newAng);
			cmd.velocity.y = -maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.accel.y = maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.position.z = 1.0;
			cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
			traj_pub.publish(cmd);
			last = cmd;
		}

		//Set second to last point of obstacle radius
		/*cmd.position.x = 0.1*cos(theta);
		cmd.position.y = obsy - repel*cos(theta) - end_off/8;
		cmd.velocity.x = 0.0;
		cmd.velocity.y = -maxV;
		cmd.accel.x = 0.0;
		cmd.accel.y = 0.0;
		cmd.position.z = 1.0;
		cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
		traj_pub.publish(cmd);
		last = cmd;*/
	}

	//Set final point of obstacle radius
	cmd.position.x = 0.0;
	cmd.position.y = obsy - repel*cos(theta) - end_off;
	cmd.velocity.x = 0.0;
	cmd.velocity.y = -maxV;
	cmd.accel.x = 0.0;
	cmd.accel.y = 0.0;
	cmd.position.z = 1.0;
	cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z);
	traj_pub.publish(cmd);
	last = cmd;

	//Set end point of path
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.velocity.x = 0.0;
	cmd.velocity.y = 0.0;
	cmd.accel.x = 0.0;
	cmd.accel.y = 0.0;
	cmd.position.z = z;
	cmd.yaw[0] = yaw;
	a_t = (maxV-cmd.velocity.y)/maxA;
	cmd.time = getTravTime(cmd.position.x, last.position.x, cmd.position.y, last.position.y, cmd.position.z, last.position.z)+a_t;
	traj_pub.publish(cmd);

	isDone = false;
}

void sendTrajectory(float wait, float x, float y, float z, float yaw) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.lock_yaw = true;
	cmd.yaw[0] = yaw;
	cmd.time = getTravTime(odom_.pose.pose.position.x, x, odom_.pose.pose.position.y, y, odom_.pose.pose.position.z, z);

	isDone = false;
	traj_pub.publish(cmd);
}

void sendRiseTrajectory() 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = 0;
	cmd.position.y = 0;
	cmd.position.z = 1;
	cmd.lock_yaw = true;
	cmd.yaw[0] = 0;
	cmd.time = getTravTimeZ(odom_.pose.pose.position.x, 0, odom_.pose.pose.position.y, 0, odom_.pose.pose.position.z, 1);

	isDone = false;
	traj_pub.publish(cmd);
}

void sendLandTrajectory() 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 1.0;
	cmd.lock_yaw = true;
	cmd.yaw[0] = 0.0;
	cmd.time = 5;
	traj_pub.publish(cmd);

	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 0.0;
	cmd.yaw[0] = 0.0;
	cmd.time = 5;
	traj_pub.publish(cmd);

	isDone = false;
}

bool obstacleExists(tf::StampedTransform * transform)
{
	/* Obstacle is observed if line traj intersects;
	 * obsx+rad > trajx (0) & obsx-rad < trajx (0)
	 */

	if(transform->getOrigin().x()+repel > 0 && 
		 transform->getOrigin().x()-repel < 0 &&
		 transform->getOrigin().y() < f_y &&
		 transform->getOrigin().y() > -f_y) {
		ROS_INFO("Obstacled detected, adapting trajectory, %f, %f", transform->getOrigin().x(), transform->getOrigin().y());
		return true;
	}
	return false;
}

bool targetPointOpen(float x, float y, tf::StampedTransform * transform)
{
	float obsx = transform->getOrigin().x();
	float obsy = transform->getOrigin().y();
	float dist = sqrt(pow((obsx-x),2) + pow((obsy-y),2));

	if(dist > repel) {
		return true;
	}else {
		return false;
	}
}

void showRange(float x, float y, float z)
{
	visualization_msgs::Marker ring;
	geometry_msgs::Point vis_ring;

	ring.header.frame_id = world;
	ring.header.stamp = ros::Time::now();
	ring.ns = "avoid_radius";
	ring.id = 2;
	ring.action = visualization_msgs::Marker::ADD;
	ring.type = visualization_msgs::Marker::LINE_LIST;
	ring.color.a = 0.4;
	ring.color.g = 1.0;				

	ring.scale.x = 0.05;
	ring.scale.y = 0.05;

	for(int j=0; j<=repel_rings; j++) {
		float i = 0.0;
		while(i < 2*M_PI) {
			vis_ring.x = cos(i) * repel + x;
			vis_ring.y = sin(i) * repel + y;
			vis_ring.z = z*j/repel_rings;
			ring.points.push_back(vis_ring);
			i += 0.05;

			vis_ring.x = cos(i) * repel + x;
			vis_ring.y = sin(i) * repel + y;
			vis_ring.z = z*j/repel_rings;
			ring.points.push_back(vis_ring);
		}
	}

	tiki_pub.publish(ring);
}

void showRealRadius(float x, float y, float z)
{
	visualization_msgs::Marker ring;
	geometry_msgs::Point vis_ring;

	ring.header.frame_id = world;
	ring.header.stamp = ros::Time::now();
	ring.ns = "obs_radius";
	ring.id = 2;
	ring.action = visualization_msgs::Marker::ADD;
	ring.type = visualization_msgs::Marker::LINE_LIST;
	ring.color.a = 0.4;
	ring.color.r = 1.0;				

	ring.scale.x = 0.05;
	ring.scale.y = 0.05;

	for(int j=0; j<=repel_rings; j++) {
		float i = 0.0;
		while(i < 2*M_PI) {
			vis_ring.x = cos(i) * real_r + x;
			vis_ring.y = sin(i) * real_r + y;
			vis_ring.z = z*j/repel_rings;
			ring.points.push_back(vis_ring);
			i += 0.05;

			vis_ring.x = cos(i) * real_r + x;
			vis_ring.y = sin(i) * real_r + y;
			vis_ring.z = z*j/repel_rings;
			ring.points.push_back(vis_ring);
		}
	}

	tiki_pub.publish(ring);
}

void showQuadRadius(float x, float y, float z)
{
	visualization_msgs::Marker ring;
	geometry_msgs::Point vis_ring;

	ring.header.frame_id = world;
	ring.header.stamp = ros::Time::now();
	ring.ns = "quad_radius";
	ring.id = 2;
	ring.action = visualization_msgs::Marker::ADD;
	ring.type = visualization_msgs::Marker::LINE_LIST;
	ring.color.a = 0.4;
	ring.color.b = 1.0;				

	ring.scale.x = 0.05;
	ring.scale.y = 0.05;

	float i = 0.0;
	while(i < 2*M_PI) {
		vis_ring.x = cos(i) * quad_r + x;
		vis_ring.y = sin(i) * quad_r + y;
		vis_ring.z = z;
		ring.points.push_back(vis_ring);
		i += 0.05;

		vis_ring.x = cos(i) * quad_r + x;
		vis_ring.y = sin(i) * quad_r + y;
		vis_ring.z = z;
		ring.points.push_back(vis_ring);
	}

	tiki_pub.publish(ring);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Obs_avoid");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;
   
	tf::StampedTransform transform;
	tf::TransformListener listener;

	ros::param::get("~world", world);
	ros::param::get("~obs_frame", obs_frame);

	traj_pub = nh.advertise<asctec_msgs::WaypointCmd>(ros::this_node::getNamespace()+"/waypoints", 10);
	tiki_pub = nh.advertise<visualization_msgs::Marker>(ros::this_node::getNamespace()+"/asctec_viz", 10);

	traj_sub = nh.subscribe(ros::this_node::getNamespace()+"/status", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);
	odom_sub = nh.subscribe(ros::this_node::getNamespace()+"/odom", 10, odomCallback);
	
	listener.waitForTransform(world, obs_frame, ros::Time(0), ros::Duration(3.0));	

	ROS_INFO("Running: Obstacle Avoider");

	while(ros::ok()) {

		ros::spinOnce();
		listener.lookupTransform(world, obs_frame, ros::Time(0), transform);
		showRange(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
		showRealRadius(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

		obs_ptr->x = transform.getOrigin().x();
		obs_ptr->y = transform.getOrigin().y();
		obs_ptr->r = repel;

		switch(state) {
			case waitStart:
				if(isFlying) {
					if(targetPointOpen(0,0,&transform)) {
						ROS_INFO("Taking off!");
						state = waitHover;
						sendRiseTrajectory();

					}else {
						ROS_INFO("Remove obstacle from center first!");
						isFlying = false;
					}
				}
				break;

			case waitHover:
				if(isDone) {
					if(nextTraj) {
						if(targetPointOpen(0,-2.0,&transform)) {
							ROS_INFO("Moving to point A");
							state = waitA;
							asctec_msgs::WaypointCmd cmd;
							cmd.position.x = 0;
							cmd.position.y = -2;
							cmd.position.z = 1;
							cmd.time = 5;
							traj_pub.publish(cmd);
						}else {
							ROS_INFO("Remove obstacle from A first!");
						}
						nextTraj = false;
					}

					if(!isFlying) {
						ROS_INFO("Landing!");
						state = waitStart;

						sendLandTrajectory();
					}
				}
				break;

			case waitA:
				if(isDone) {
					if(nextTraj) {
						ROS_INFO("Observing obstacle(s) from A->B");
						nextTraj = false;
						state = waitB;

						if(obstacleExists(&transform)) {
							sendAvoidABNew(0,f_x,f_y,1,0,&transform);

						}else {
							sendTrajectory(0,f_x,f_y,1,0);
						}
					}

					if(!isFlying) {
						if(targetPointOpen(0,0,&transform)) {
							ROS_INFO("Landing!");
							nextTraj = false;
							state = waitStart;

							sendLandTrajectory();
						}else {
							ROS_INFO("Not safe to land! Remove object from center first!");
							isFlying = true;
						}
					}
				}
				break;

			case waitB:
				if(isDone) {
					if(nextTraj) {
						ROS_INFO("Observing obstacle(s) from B->A");
						nextTraj = false;
						state = waitA;

						if(obstacleExists(&transform)) {
							sendAvoidBANew(0,f_x,-f_y,1,0,&transform);

						}else {
							sendTrajectory(0,f_x,-f_y,1,0);
						}
					}

					if(!isFlying) {
						if(targetPointOpen(0,0,&transform)) {
							ROS_INFO("Landing!");
							nextTraj = false;
							state = waitStart;

							sendLandTrajectory();
						}else {
							ROS_INFO("Not safe to land! Remove object from center first!");
							isFlying = true;
						}
					}
				}
				break;
		}
	loop_rate.sleep();
	}

	return 0;
}
