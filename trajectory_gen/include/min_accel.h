#ifndef MINACC_H
#define MINACC_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/MinAccelCmd.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>

using Eigen::MatrixXf;

/* -------------------- Class Definition ---------------- */
class MinAccel
{
	public:
		MinAccel();
		asctec_msgs::PositionCmd* getNextCommand(void);
		void setState(nav_msgs::Odometry odom);
		void addWaypoint(asctec_msgs::MinAccelCmd wp);
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
};
#endif
