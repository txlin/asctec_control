#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#define y_offset 0.0		// defines the center offset only for y
#define A 1

bool init[2] = {false, false};
bool isDone[2] = {true, true};
bool flying = false;

nav_msgs::Odometry odom[2];
ros::Publisher waypointsA_pub, waypointsB_pub;

/* -------------------- callbacks -------------------- */
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[A]) {
		if(!isDone[0] || !isDone[1]) ROS_INFO("Finishing trajectory, command ignored...");
		else flying = !flying;
	}
}

void odomACallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom[0] = *msg;
}

void odomBCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom[1] = *msg;
}

void statusACallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone[0] = msg->data;
	init[0] = true;
}

void statusBCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone[1] = msg->data;
	init[1] = true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "link_lab");
	ros::NodeHandle nh;
	ros::Rate rate = 5.0;

	float z0 = 1.2, y0 = 1.5, x0 = 0.0;
	std::string A_tag, B_tag;
	ros::param::get("~z0", z0);
	ros::param::get("~y0", y0);
	ros::param::get("~x0", x0);
	if(!ros::param::get("~A_namespace", A_tag) || !ros::param::get("~B_namespace", B_tag)) {
		ROS_INFO("namespace tags not set, exiting...");
		return 0;
	}

	/* -------------------- Publishers, and Subscribers -------------------- */
	waypointsA_pub = nh.advertise<asctec_msgs::WaypointCmd>(A_tag+"/waypoints", 10);
	waypointsB_pub = nh.advertise<asctec_msgs::WaypointCmd>(B_tag+"/waypoints", 10);
	
	ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);

	ros::Subscriber statusA_sub = nh.subscribe(A_tag+"/status", 1, statusACallback);
	ros::Subscriber statusB_sub = nh.subscribe(B_tag+"/status", 1, statusBCallback);
	
	ros::Subscriber odomA_sub = nh.subscribe(A_tag+"/odom", 1, odomACallback);
	ros::Subscriber odomB_sub = nh.subscribe(B_tag+"/odom", 1, odomBCallback);

	int state = 0;
	while(ros::ok()) {
		rate.sleep();
		ros::spinOnce();
		
		switch(state) {
			case 0:
				if(init[0] && init[1] && flying && isDone[0] && isDone[1]) {
					asctec_msgs::WaypointCmd cmd;
					cmd.position.x = x0;
					cmd.position.y = y0 + y_offset;
					cmd.position.z = z0;
					cmd.lock_yaw = true;
					cmd.time = 5.0;
					waypointsA_pub.publish(cmd);

					cmd.position.x = x0;
					cmd.position.y = -y0 + y_offset;
					waypointsB_pub.publish(cmd);
					ROS_INFO("Taking off!");
					state = 1;
				}
			break;

			case 1:
				if(init[0] && init[1] && !flying && isDone[0] && isDone[1]) {
					asctec_msgs::WaypointCmd cmd;
					cmd.position.x = odom[0].pose.pose.position.x;
					cmd.position.y = odom[0].pose.pose.position.y;
					cmd.position.z = 0.0;
					cmd.lock_yaw = true;
					cmd.time = 5.0;
					waypointsA_pub.publish(cmd);

					cmd.position.x = odom[1].pose.pose.position.x;
					cmd.position.y = odom[1].pose.pose.position.y;
					waypointsB_pub.publish(cmd);
					ROS_INFO("Landing!");
					state = 0;
				}
			break;
		}
	}
	return 0;
}
