#ifndef MINSNAP_H
#define MINSNAP_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/WaypointCmd.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>

using Eigen::MatrixXf;

/* -------------------- Class Definition ---------------- */
class MinSnap
{
	public:
		MinSnap();
		asctec_msgs::PositionCmd* getNextCommand(void);
		void setState(const nav_msgs::Odometry::ConstPtr& odom);
		visualization_msgs::Marker *getMarker(void);
		visualization_msgs::Marker *deleteMarker(void);
		float setTime(const asctec_msgs::WaypointCmd::ConstPtr& cmd, float desV, float desA);
		void addWaypoint(const asctec_msgs::WaypointCmd::ConstPtr& wp);
		void resetWaypoints(void);
		bool getStatus(void);
		void setPubSub(ros::NodeHandle *n, float rate);
		void setCont(bool cont_);	

	private:
		bool init;
		bool continuous;
  	ros::Time t0;
    std::vector<MatrixXf *> Xx, Xy, Xz, Xyaw;

		std::vector<bool> lock;
		std::vector<float> T;
		MatrixXf A;
  	MatrixXf Bx, By, Bz, Byaw;
    nav_msgs::Odometry odom_;
  	asctec_msgs::PositionCmd cmd_;
		visualization_msgs::Marker path_;

		ros::Timer timer;
		ros::Publisher pos_goal, status, wp_viz;
		ros::Subscriber wpt_sub, odom_sub;

		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
		void waypointCallback(const asctec_msgs::WaypointCmd::ConstPtr& msg);
		void timerCallback(const ros::TimerEvent& event);
};
#endif
