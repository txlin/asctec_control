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

#define repel_pts 3.0	//always integer
#define repel 0.7       //radius
#define repel_t 5.0
#define repel_rings 5
#define end_off 0.5

#define real_r  0.14    //11 inches diameter
#define quad_r 0.27     //21 inches diameter

#define freq 10
#define maxV 0.25
#define maxVZ 0.2
#define maxA maxV

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

Trajectory_Accel * path_gen = new Trajectory_Accel(1.0);

string world, topic, obs_frame;
pc_asctec_sim::pc_state q_st;
pc_asctec_sim::pc_traj_cmd raw;
pc_asctec_sim::CMatrix C;

ros::Publisher traj_pub, start_pub, obs_pub, tiki_pub, quad_pub;
ros::Subscriber rawtraj_sub, traj_sub, joy_sub, state_sub;

void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg)
{
	q_st = *msg;
}

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void rawtrajCallback(const pc_asctec_sim::pc_traj_cmd::ConstPtr& msg)
{
	NEW_PATH temp;
	temp.state = q_st;
	temp.cmd = *msg;
	points = msg->points;

	path_gen->getPathConstants(&temp,&C);
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

	asctec_msgs::Waypoint cmd;
	//Set first point of obstacle radius
	cmd.position.x = 0.0;
	cmd.position.y = obsy - repel*cos(theta) - end_off;
	cmd.velocity.y = maxV;
	cmd.position.z = 1.0;
	cmd.time = getTravTime(q_st.x, cmd.x[0], q_st.y, cmd.y[0], q_st.z, cmd.z[0]);

	//Set mid points of obstacle radius
	if(obsx > 0) {
		//Set second point of obstacle radius
		cmd.x[1] = -0.1*cos(theta);
		cmd.y[1] = obsy - repel*cos(theta) - end_off/8;
		cmd.vy[1] = maxV;
		cmd.z[1] = 1.0;
		cmd.duration[1] = getTravTime(cmd.x[0], cmd.x[1], cmd.y[0], cmd.y[1], cmd.z[0], cmd.z[1]);

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.x[i+1] = obsx - repel*sin(newAng);
			cmd.vx[i+1] = -maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.ax[i+1] = -maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.y[i+1] = obsy - repel*cos(newAng);
			cmd.vy[i+1] = maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.ay[i+1] = -maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.z[i+1] = 1.0;
			cmd.duration[i+1] = getTravTime(cmd.x[i], cmd.x[i+1], cmd.y[i], cmd.y[i+1], cmd.z[i], cmd.z[i+1]);
		}

		//Set second to last point of obstacle radius
		cmd.x[repel_pts+2] = -0.1*cos(theta);
		cmd.y[repel_pts+2] = obsy + repel*cos(theta) + end_off/8;
		cmd.vy[repel_pts+2] = maxV;
		cmd.z[repel_pts+2] = 1.0;
		cmd.duration[repel_pts+2] = getTravTime(cmd.x[repel_pts+1], cmd.x[repel_pts+2], cmd.y[repel_pts+1], cmd.y[repel_pts+2], cmd.z[repel_pts+1], cmd.z[repel_pts+2]);

	}else {
		//Set second point of obstacle radius
		cmd.x[1] = 0.1*cos(theta);
		cmd.y[1] = obsy - repel*cos(theta) - end_off/8;
		cmd.vy[1] = maxV;
		cmd.z[1] = 1.0;
		cmd.duration[1] = getTravTime(cmd.x[0], cmd.x[1], cmd.y[0], cmd.y[1], cmd.z[0], cmd.z[1]);

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.x[i+1] = obsx + repel*sin(newAng);
			cmd.vx[i+1] = maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.ax[i+1] = maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.y[i+1] = obsy - repel*cos(newAng);
			cmd.vy[i+1] = maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.ay[i+1] = -maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.z[i+1] = 1.0;
			cmd.duration[i+1] = getTravTime(cmd.x[i], cmd.x[i+1], cmd.y[i], cmd.y[i+1], cmd.z[i], cmd.z[i+1]);
		}

		//Set second to last point of obstacle radius
		cmd.x[repel_pts+2] = 0.1*cos(theta);
		cmd.y[repel_pts+2] = obsy + repel*cos(theta) + end_off/8;
		cmd.vy[repel_pts+2] = maxV;
		cmd.z[repel_pts+2] = 1.0;
		cmd.duration[repel_pts+2] = getTravTime(cmd.x[repel_pts+1], cmd.x[repel_pts+2], cmd.y[repel_pts+1], cmd.y[repel_pts+2], cmd.z[repel_pts+1], cmd.z[repel_pts+2]);
	}

	//Set final point of obstacle radius
	cmd.x[repel_pts+3] = 0.0;
	cmd.y[repel_pts+3] = obsy + repel*cos(theta) + end_off;
	cmd.vy[repel_pts+3] = maxV;
	cmd.z[repel_pts+3] = 1.0;
	cmd.duration[repel_pts+3] = getTravTime(cmd.x[repel_pts+2], cmd.x[repel_pts+3], cmd.y[repel_pts+2], cmd.y[repel_pts+3], cmd.z[repel_pts+2], cmd.z[repel_pts+3]);

	//Set end point of path
	cmd.x[repel_pts+4] = x;
	cmd.y[repel_pts+4] = y;
	cmd.z[repel_pts+4] = z;
	cmd.yaw[repel_pts+4] = yaw;
	cmd.wait_time[repel_pts+4] = wait;
	cmd.duration[repel_pts+4] = getTravTime(cmd.x[repel_pts+3], cmd.x[repel_pts+4], cmd.y[repel_pts+3], cmd.y[repel_pts+4], cmd.z[repel_pts+3], cmd.z[repel_pts+4]);

	cmd.points = repel_pts+5;
	isDone = false;
	traj_pub.publish(cmd);
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

	pc_asctec_sim::pc_traj_cmd cmd;
	//Set first point of obstacle radius
	cmd.x[0] = 0.0;
	cmd.y[0] = obsy + repel*cos(theta) + end_off;
	cmd.vy[0] = -maxV;
	cmd.z[0] = 1.0;
	cmd.duration[0] = getTravTime(q_st.x, cmd.x[0], q_st.y, cmd.y[0], q_st.z, cmd.z[0]);

	//Set mid points of obstacle radius
	if(obsx > 0) {
		//Set second point of obstacle radius
		cmd.x[1] = -0.1*cos(theta);
		cmd.y[1] = obsy + repel*cos(theta) + end_off/8;
		cmd.vy[1] = -maxV;
		cmd.z[1] = 1.0;
		cmd.duration[1] = getTravTime(cmd.x[0], cmd.x[1], cmd.y[0], cmd.y[1], cmd.z[0], cmd.z[1]);

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.x[i+1] = obsx - repel*sin(newAng);
			cmd.vx[i+1] = -maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.ax[i+1] = -maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.y[i+1] = obsy + repel*cos(newAng);
			cmd.vy[i+1] = -maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.ay[i+1] = maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.z[i+1] = 1.0;
			cmd.duration[i+1] = getTravTime(cmd.x[i], cmd.x[i+1], cmd.y[i], cmd.y[i+1], cmd.z[i], cmd.z[i+1]);
		}

		//Set second to last point of obstacle radius
		cmd.x[repel_pts+2] = -0.1*cos(theta);
		cmd.y[repel_pts+2] = obsy - repel*cos(theta) - end_off/8;
		cmd.vy[repel_pts+2] = -maxV;
		cmd.z[repel_pts+2] = 1.0;
		cmd.duration[repel_pts+2] = getTravTime(cmd.x[repel_pts+1], cmd.x[repel_pts+2], cmd.y[repel_pts+1], cmd.y[repel_pts+2], cmd.z[repel_pts+1], cmd.z[repel_pts+2]);

	}else {
		//Set second point of obstacle radius
		cmd.x[1] = 0.1*cos(theta);
		cmd.y[1] = obsy + repel*cos(theta) + end_off/8;
		cmd.vy[1] = -maxV;
		cmd.z[1] = 1.0;
		cmd.duration[1] = getTravTime(cmd.x[0], cmd.x[1], cmd.y[0], cmd.y[1], cmd.z[0], cmd.z[1]);

		for(int i=1; i<=repel_pts; i++) {
			float newAng = theta + i*gamma/(repel_pts+1);
			cmd.x[i+1] = obsx + repel*sin(newAng);
			cmd.vx[i+1] = maxV*sin(M_PI/2 * i/(repel_pts-1) + M_PI/2);
			cmd.ax[i+1] = maxV*cos(M_PI/2 * i/(repel_pts-1) + M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.y[i+1] = obsy + repel*cos(newAng);
			cmd.vy[i+1] = -maxV*cos(M_PI/2 * i/(repel_pts-1) - M_PI/2);
			cmd.ay[i+1] = maxV*sin(M_PI/2 * i/(repel_pts-1) - M_PI/2) * M_PI/(2*(repel_pts-1));

			cmd.z[i+1] = 1.0;
			cmd.duration[i+1] = getTravTime(cmd.x[i], cmd.x[i+1], cmd.y[i], cmd.y[i+1], cmd.z[i], cmd.z[i+1]);
		}

		//Set second to last point of obstacle radius
		cmd.x[repel_pts+2] = 0.1*cos(theta);
		cmd.y[repel_pts+2] = obsy - repel*cos(theta) - end_off/8;
		cmd.vy[repel_pts+2] = -maxV;
		cmd.z[repel_pts+2] = 1.0;
		cmd.duration[repel_pts+2] = getTravTime(cmd.x[repel_pts+1], cmd.x[repel_pts+2], cmd.y[repel_pts+1], cmd.y[repel_pts+2], cmd.z[repel_pts+1], cmd.z[repel_pts+2]);
	}

	//Set final point of obstacle radius
	cmd.x[repel_pts+3] = 0.0;
	cmd.y[repel_pts+3] = obsy - repel*cos(theta) - end_off;
	cmd.vy[repel_pts+3] = -maxV;
	cmd.z[repel_pts+3] = 1.0;
	cmd.duration[repel_pts+3] = getTravTime(cmd.x[repel_pts+2], cmd.x[repel_pts+3], cmd.y[repel_pts+2], cmd.y[repel_pts+3], cmd.z[repel_pts+2], cmd.z[repel_pts+3]);

	//Set end point of path
	cmd.x[repel_pts+4] = x;
	cmd.y[repel_pts+4] = y;
	cmd.z[repel_pts+4] = z;
	cmd.yaw[repel_pts+4] = yaw;
	cmd.wait_time[repel_pts+4] = wait;
	cmd.duration[repel_pts+4] = getTravTime(cmd.x[repel_pts+3], cmd.x[repel_pts+4], cmd.y[repel_pts+3], cmd.y[repel_pts+4], cmd.z[repel_pts+3], cmd.z[repel_pts+4]);

	cmd.points = repel_pts+5;
	isDone = false;
	traj_pub.publish(cmd);
}

void sendTrajectory(float wait, float x, float y, float z, float yaw) 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = x;
	cmd.y[0] = y;
	cmd.z[0] = z;
	cmd.yaw[0] = yaw;
	cmd.wait_time[0] = wait;
	cmd.duration[0] = getTravTime(q_st.x, x, q_st.y, y, q_st.z, z);
	cmd.points = 1;

	isDone = false;
	traj_pub.publish(cmd);
}

void sendRiseTrajectory() 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0;
	cmd.y[0] = 0;
	cmd.z[0] = 1;
	cmd.yaw[0] = 0;
	cmd.wait_time[0] = 0;
	cmd.duration[0] = getTravTimeZ(q_st.x, 0, q_st.y, 0, q_st.z, 1);
	cmd.points = 1;

	isDone = false;
	traj_pub.publish(cmd);
}

void sendLandTrajectory() 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0.0;
	cmd.y[0] = 0.0;
	cmd.z[0] = 1.0;
	cmd.yaw[0] = 0.0;
	cmd.wait_time[0] = 0.5;
	cmd.duration[0] = getTravTime(q_st.x, 0, q_st.y, 0, q_st.z, 1);

	cmd.x[1] = 0.0;
	cmd.y[1] = 0.0;
	cmd.z[1] = 0.0;
	cmd.yaw[1] = 0.0;
	cmd.wait_time[1] = 0.0;
	cmd.duration[1] = 3;

	cmd.points = 2;

	isDone = false;
	traj_pub.publish(cmd);
}

bool obstacleExists(tf::StampedTransform * transform)
{
	/* Obstacle is observed if line traj intersects;
	 * obsx+rad > trajx (0) & obsx-rad < trajx (0)
	 */

	if(transform->getOrigin().x()+repel > 0 && transform->getOrigin().x()-repel < 0) {
		ROS_INFO("Obstacled detected, adapting trajectory");
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
	ring.id = 2;
	ring.action = visualization_msgs::Marker::ADD;
	ring.type = visualization_msgs::Marker::LINE_LIST;
	ring.color.a = 1.0;
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

	obs_pub.publish(ring);
}

void showRealRadius(float x, float y, float z)
{
	visualization_msgs::Marker ring;
	geometry_msgs::Point vis_ring;

	ring.header.frame_id = world;
	ring.header.stamp = ros::Time::now();
	ring.id = 2;
	ring.action = visualization_msgs::Marker::ADD;
	ring.type = visualization_msgs::Marker::LINE_LIST;
	ring.color.a = 1.0;
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
	ring.id = 2;
	ring.action = visualization_msgs::Marker::ADD;
	ring.type = visualization_msgs::Marker::LINE_LIST;
	ring.color.a = 1.0;
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

	quad_pub.publish(ring);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Obs_avoid");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;
   
	tf::StampedTransform transform;
	tf::TransformListener listener;

	ros::param::get("~w_frame", world);
	ros::param::get("~topic", topic);
	ros::param::get("~obs_frame", obs_frame);

	traj_pub = nh.advertise<asctec_msgs::WaypointCmd>(topic + "/waypoints", 10);
	tiki_pub = nh.advertise<visualization_msgs::Marker>(topic + "/asctec_viz", 10);

	traj_sub = nh.subscribe(topic + "/status", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);
	odom_sub = nh.subscribe(topic + "/odom", 10, odomCallback);
	
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

						std_msgs::Bool start;
						start.data = true;
						start_pub.publish(start);

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
							sendTrajectory(0,0,-2.0,1,0);
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
							sendAvoidABNew(0,0,2.0,1,0,&transform);

						}else {
							sendTrajectory(0,0,2.0,1,0);
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
							sendAvoidBANew(0,0,-2.0,1,0,&transform);

						}else {
							sendTrajectory(0,0,-2.0,1,0);
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
