#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <math.h>

#include <asctec_msgs/WaypointCmd.h>
#include <asctec_msgs/SICmd.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>

ros::Publisher wpt_pub1, wpt_pub2;
ros::Publisher final_pub, obs_vmax_pub, new_wpt_pub;

nav_msgs::Odometry odom1_, odom2_;

bool isDone1 = true, isFinal1 = false, isStart1 = true, isDone2 = true, isFinal2 = false;
bool hover1 = false, hover2 = false, move1 = false, move2 = false, land2 = false, land1 = false;
bool flying1 = false, flying2 = false;
bool ready1 = false, ready2 = false;
bool avoid1 = false;

// for collision 0.3, 0.05, for passing 0.3, 0.3
float max_v1 = 0.3, max_v2 = 0.05;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0] && !flying1 ) {
		// Button X hover
		hover1 = true;
		ROS_INFO("quad 1 hovering!");

	}else if(msg->buttons[1]) {
		// Button A
		move1 = true;
		ROS_INFO("quad 1 moving!");
	}else if(msg->buttons[0] && flying1){	
		// Button X land
		land1 = true;
		ROS_INFO("quad 1 landing!");	

	}else if(msg->buttons[2] && !flying2){
		// Button B hover
		hover2 = true;
		ROS_INFO("quad 2 hovering!");

	}else if(msg->buttons[2] && flying2){
		// Button B land
		land2 = true;
		ROS_INFO("quad 2 landing!");
	}else if(msg->buttons[3]){
		// Button Y move
		move2 = true;
		ROS_INFO("quad 2 moving!");	
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

void sendLandTrajectory1(float time) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 1.0;
	cmd.yaw[0] = 0;
	cmd.lock_yaw = true;
	cmd.time = time;
	cmd.reset = true;

	wpt_pub1.publish(cmd);
	isDone1 = false;
	ROS_INFO("Returning to 0.0, 0.0, 1.0, time: %.02f", time);
}

void sendFinalTrajectory1(float time, float x, float y, float z, float yaw, bool reset, bool hold) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.yaw[0] = yaw;
	cmd.lock_yaw = true;
	cmd.time = time;
	cmd.reset = reset;
	cmd.hold_z = hold;
	wpt_pub1.publish(cmd);

	nav_msgs::Odometry wpt_cmd;
	wpt_cmd.pose.pose.position.x = x;
	wpt_cmd.pose.pose.position.y = y;
	wpt_cmd.pose.pose.position.z = z;
	wpt_cmd.twist.twist.linear.x = 0;
	wpt_cmd.twist.twist.linear.y = 0;
	wpt_cmd.twist.twist.linear.z = 0;
	new_wpt_pub.publish(wpt_cmd);

	isDone1 = false;
	isFinal1 = true;
	ROS_INFO("Quad 1 Moving to %.02f, %.02f, %.02f, time: %.02f", x, y, z, time);
}

void sendStartTrajectory1(float time, float x, float y, float z, float vel_x, float vel_y, float vel_z, float yaw, bool reset) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.velocity.x  = vel_x;
	cmd.velocity.y = vel_y;
	cmd.velocity.z = vel_z;
	cmd.yaw[0] = yaw;
	cmd.lock_yaw = true;
	cmd.time = time;
	cmd.reset = reset;

	wpt_pub1.publish(cmd);

	isDone1 = false;
	isFinal1 = false;
	isStart1 = false;
	ROS_INFO("Moving to %.02f, %.02f, %.02f, time: %.02f", x, y, z, time);
}

void sendTrajectory1(float time, float x, float y, float z, float vel_x, float vel_y, float vel_z, float yaw, bool reset, bool hold) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.velocity.x  = vel_x;
	cmd.velocity.y = vel_y;
	cmd.velocity.z = vel_z;
	cmd.yaw[0] = yaw;
	cmd.lock_yaw = true;
	cmd.time = time;
	cmd.reset = reset;
	cmd.hold_z = hold;

	wpt_pub1.publish(cmd);

	isDone1 = false;
	isFinal1 = false;
	ROS_INFO("Moving to %.02f, %.02f, %.02f, time: %.02f", x, y, z, time);
}

void sendLandTrajectory2(float time) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = 0.0;
	cmd.position.y = 0.0;
	cmd.position.z = 1.0;
	cmd.yaw[0] = 0;
	cmd.lock_yaw = true;
	cmd.time = time;

	wpt_pub2.publish(cmd);
	isDone2 = false;
	ROS_INFO("Returning to 0.0, 0.0, 1.0, time: %.02f", time);
}

void sendTrajectory2(float time, float x, float y, float z, float yaw) 
{
	asctec_msgs::WaypointCmd cmd;
	cmd.position.x = x;
	cmd.position.y = y;
	cmd.position.z = z;
	cmd.yaw[0] = yaw;
	cmd.lock_yaw = true;
	cmd.time = time;

	wpt_pub2.publish(cmd);
	isDone2 = false;
	ROS_INFO("Moving to %.02f, %.02f, %.02f, time: %.02f", x, y, z, time);
}

void statusCallback1(const std_msgs::Bool::ConstPtr &msg)
{
	isDone1 = msg->data;
	isStart1 = msg->data;
}

void statusCallback2(const std_msgs::Bool::ConstPtr &msg)
{
	isDone2 = msg->data;
}

void odomCallback1(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom1_ = *msg;
	float goal_x = 0, goal_y = 2, goal_z = 1;
	float start_x = 0, start_y = -2, start_z = 1;
	float obs_dist, obs_threshold = 1.5;
	static bool toGoal = false;
	std_msgs::Bool reached_cmd;
	reached_cmd.data = (isFinal1 && isDone1); 
	final_pub.publish(reached_cmd);	

	obs_dist = sqrt(pow(odom1_.pose.pose.position.x-odom2_.pose.pose.position.x, 2) + pow(odom1_.pose.pose.position.y-odom2_.pose.pose.position.y, 2));
	//ROS_INFO("%i, %i, %i", isDone1, toGoal, ready2);
	if (isDone1 && hover1){
		hover1 = false;
		sendStartTrajectory1(4, 0, 0, 1, 0, 0, 0, 0, false);
	}else if(isDone1 && move1){
		// move to the initial hovering position
		flying1 = true;
		move1 = false;
		ROS_INFO("quad1 moving to starting position");
		float tTravel = sqrt(pow(odom1_.pose.pose.position.x - start_x,2) + pow(odom1_.pose.pose.position.y - start_y, 2) + pow(odom1_.pose.pose.position.z - start_z, 2))/max_v1;
		tTravel = limit(tTravel, 10, 1);
		sendStartTrajectory1(tTravel,  start_x, start_y, start_z, 0, 0, 0, 0, false);
		toGoal = true;
	}else if(isDone1 && toGoal && ready2){
		// start the motion by moving to the final goal
		flying1 = true;
		ready1 = true;
		ROS_INFO("quad1 moving to the goal position, traj updated");
		float tTravel = sqrt(pow(odom1_.pose.pose.position.x - goal_x,2) + pow(odom1_.pose.pose.position.y - goal_y, 2) + pow(odom1_.pose.pose.position.z - goal_z, 2))/max_v1;
		tTravel = limit(tTravel, 10, 1);
		sendFinalTrajectory1(tTravel,  goal_x, goal_y, goal_z, 0, 1, true);	
	}else if(ready1 && obs_dist < obs_threshold){
		// when the distance to the obstacle is smaller than certain threshold, avoid it
		ROS_INFO("quad1 moving to the avoid position");
		flying1 = true;
		ready1 = false;
		avoid1 = true;
		float avoid_rad = 0.2;
		float obstacle_x = odom2_.pose.pose.position.x;
		float obstacle_y = odom2_.pose.pose.position.y;
		float obstacle_speed = sqrt(pow(odom2_.twist.twist.linear.x, 2) + pow(odom2_.twist.twist.linear.y, 2));
		float obstacle_dir_x = odom2_.twist.twist.linear.x/obstacle_speed;
		float obstacle_dir_y = odom2_.twist.twist.linear.y/obstacle_speed;
		
		float avoid_x = obstacle_x - avoid_rad * obstacle_dir_x;
		float avoid_y = obstacle_y - avoid_rad * obstacle_dir_y; 
		float avoid_z = 1.0;

		float avoid_vel_x = 0.0;
		float avoid_vel_y = 0.3;
		float avoid_vel_z = 0.0;

		// first go to avoidance waypoint then go to the final goal			
		float avoid_dist = sqrt(pow(avoid_x - odom1_.pose.pose.position.x,2) + pow(avoid_y - odom1_.pose.pose.position.y,2));
		float avoid_dir_x = avoid_x - odom1_.pose.pose.position.x / avoid_dist;
		float avoid_dir_y = avoid_y - odom1_.pose.pose.position.y / avoid_dist;
		
		sendTrajectory1(1.35, odom1_.pose.pose.position.x-0.25, odom1_.pose.pose.position.y+0.4, odom1_.pose.pose.position.z, 0.3*avoid_dir_x, 0.3*avoid_dir_y, 0, 0, true, true);
		float tTravel = sqrt(pow(odom1_.pose.pose.position.x-0.25 - avoid_x,2) + pow(odom1_.pose.pose.position.y+0.4 - avoid_y, 2) + pow(odom1_.pose.pose.position.z - avoid_z, 2))/max_v1;	
		tTravel = tTravel + 0.25;
		tTravel = limit(tTravel, 10, 1);
		sendTrajectory1(tTravel,  avoid_x, avoid_y, avoid_z, avoid_vel_x, avoid_vel_y, avoid_vel_z, 0, false, true);	

		tTravel = sqrt(pow(odom1_.pose.pose.position.x - goal_x,2) + pow(odom1_.pose.pose.position.y - goal_y, 2) + pow(odom1_.pose.pose.position.z - goal_z, 2))/max_v1;		
		tTravel = limit(tTravel, 10, 1);
		sendTrajectory1(tTravel,  goal_x, goal_y, goal_z, 0, 0, 0, 0, false, true);			
	}else if(avoid1 && obs_dist > obs_threshold){
		// after avoiding the obstacle, when the distance becomes larger again, move to the final goal
		flying1 = true;
		ready1 = false;
		avoid1 = false;
		ROS_INFO("quad1 moving to the goal position, traj updated");
		float tTravel = sqrt(pow(odom1_.pose.pose.position.x - goal_x,2) + pow(odom1_.pose.pose.position.y - goal_y, 2) + pow(odom1_.pose.pose.position.z - goal_z, 2))/max_v1;
		tTravel = limit(tTravel, 10, 1);
		sendFinalTrajectory1(tTravel,  goal_x, goal_y, goal_z, 0, 1, true);		
	}else if(isDone1 && land1){
		sendTrajectory1(8, 0, 0, 1, 0, 0, 0, 0, true, false);
		sendLandTrajectory1(3);
		return;			
	}

}

void odomCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom2_ = *msg;	
	static bool toFinal = false, toStart = false;
	float start_x = -1,  final_x = 1;
	float start_y = 0, final_y = 0;
	float start_z = 1,  final_z = 1;

	std_msgs::Float64 v_max;
	v_max.data = max_v2;
	obs_vmax_pub.publish(v_max);

	if (isDone2 && hover2){
		hover2 = false;
		sendTrajectory2(4, 0, 0, 1, 0);
		toStart = true;
	}else if(isDone2 && move2){
		flying2 = true;
		
		if (toStart){
			ROS_INFO("quad2 moving to starting position");
			toStart = false;
			float tTravel = sqrt(pow(odom2_.pose.pose.position.x - start_x,2) + pow(odom2_.pose.pose.position.y - start_y, 2) + pow(odom2_.pose.pose.position.z - start_z, 2))/max_v2;
			tTravel = limit(tTravel, 10, 1);
			sendTrajectory2(tTravel,  start_x, start_y, start_z, 0);
			toFinal = true;
		}else if(toFinal){
			ROS_INFO("quad2 moving to final position");
			ready2 = true;			
			toFinal = false;
			// for collision wait 2 sec, for passing wait 0.75 sec
			//sendTrajectory2(0.5, odom2_.pose.pose.position.x, odom2_.pose.pose.position.y, odom2_.pose.pose.position.z, 0);
			float tTravel = sqrt(pow(odom2_.pose.pose.position.x - final_x,2) + pow(odom2_.pose.pose.position.y - final_y, 2) + pow(odom2_.pose.pose.position.z - final_z,2))/max_v2;
			tTravel = limit(tTravel, 10, 1);
			sendTrajectory2(tTravel,  final_x, final_y, final_z, 0);					
		}
		
	}else if(isDone2 && land2){
		sendTrajectory2(8, 0, 0, 1, 0);
		sendLandTrajectory2(3);
		return;			
	}
	
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "dynamic_obs");
	ros::NodeHandle nh;

	std::string group_ns1, group_ns2;
	
	ros::param::get("~group_ns1", group_ns1);
	ros::param::get("~group_ns2", group_ns2);

	ros::Subscriber odom_sub1 = nh.subscribe(group_ns1 + "/odom", 10, odomCallback1);
	ros::Subscriber odom_sub2 = nh.subscribe(group_ns2 + "/odom", 10, odomCallback2);
	ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);
	ros::Subscriber status_sub1 = nh.subscribe(group_ns1 + "/status", 1, statusCallback1);
	ros::Subscriber status_sub2 = nh.subscribe(group_ns2 + "/status", 1, statusCallback2);

	wpt_pub1 = nh.advertise<asctec_msgs::WaypointCmd>(group_ns1 + "/waypoints", 10);
	wpt_pub2 = nh.advertise<asctec_msgs::WaypointCmd>(group_ns2 + "/waypoints", 10);

	final_pub = nh.advertise<std_msgs::Bool>(group_ns1 + "/final", 1);
	obs_vmax_pub = nh.advertise<std_msgs::Bool>(group_ns2 + "/obs_max_vel", 1);

	ros::spin();
	return 0;
}
