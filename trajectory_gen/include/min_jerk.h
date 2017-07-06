#ifndef MINJERK_H
#define MINJERK_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/WaypointCmd.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>

using Eigen::MatrixXf;

/* -------------------- Class Definition ---------------- */
class MinJerk
{
	public:
		MinJerk();
		asctec_msgs::PositionCmd* getNextCommand(void);
		void setState(const nav_msgs::Odometry::ConstPtr& odom);
		visualization_msgs::Marker *getMarker(void);
		visualization_msgs::Marker *deleteMarker(void);
		void addWaypoint(const asctec_msgs::WaypointCmd::ConstPtr& wp);
		void resetWaypoints(void);
		bool getStatus(void);

	private:
  	ros::Time t0;
    std::vector<MatrixXf *> Xx, Xy, Xz, Xyaw;

		std::vector<float> T;
		MatrixXf A;
  	MatrixXf Bx, By, Bz, Byaw;
    nav_msgs::Odometry odom_;
  	asctec_msgs::PositionCmd cmd_;
		visualization_msgs::Marker path_;
};
#endif
