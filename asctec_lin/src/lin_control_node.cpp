#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <asctec_msgs/SICmd.h>
#include <asctec_msgs/PositionCmd.h>
#include <std_msgs/Bool.h>
#include "PIDControl.h"
#include "PIDControl.cpp"
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <math.h>

#define MAX_ANGLE M_PI/16
#define MAX_THRUST 13.184d
#define MIN_THRUST 0.0d

PIDControl controller_;
ros::Publisher trpy_command_pub_;
ros::Subscriber odom_sub_;
ros::Subscriber position_cmd_sub_;

bool position_cmd_updated_ = false, position_cmd_init_ = false;
Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_, ki_;
double des_yaw_ = 0.0, ki_yaw_ = 5.0, k_yaw_ = 1.0, current_yaw_ = 0;

void publishTRPYCommand(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, kx_, kv_, ki_, ki_yaw_, k_yaw_);
  const Eigen::Vector4d &trpy = controller_.getControls();

  asctec_msgs::SICmd trpy_command;
  trpy_command.thrust = std::min(double(trpy(0)),MAX_THRUST)/MAX_THRUST;
  trpy_command.roll   = std::min(MAX_ANGLE,std::max(-MAX_ANGLE,trpy(1)));
  trpy_command.pitch  = std::min(MAX_ANGLE,std::max(-MAX_ANGLE,-trpy(2)));
  trpy_command.yaw    = -trpy(3);
  trpy_command.cmd[0] = trpy_command.cmd[1] = trpy_command.cmd[2] = trpy_command.cmd[3] = true;
  trpy_command_pub_.publish(trpy_command);

}

void positionCmdCallback(const asctec_msgs::PositionCmd::ConstPtr &cmd)
{
	if(position_cmd_init_) {
		des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
		des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
		des_acc_ = Eigen::Vector3d(cmd->accel.x, cmd->accel.y, cmd->accel.z);
		des_yaw_ = cmd->yaw[0];

		position_cmd_updated_ = true;
		publishTRPYCommand();
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setYaw(current_yaw_);

	if(!position_cmd_init_) {
		des_pos_ = position;
		des_vel_ = Eigen::Vector3d(0, 0, 0);
		des_acc_ = Eigen::Vector3d(0, 0, 0);
		des_yaw_ = current_yaw_;

		position_cmd_init_ = true;
	}

  if(!position_cmd_updated_) {
    publishTRPYCommand();
	}
	position_cmd_updated_ = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lin_control");
  ros::NodeHandle n("~");

  double mass;
  n.param("mass", mass, 0.55);
  controller_.setMass(mass);
  controller_.setMaxIntegral(mass*1);

  std::vector<double> kx_list, kv_list, ki_list;
  n.getParam("gains/pos", kx_list);
  n.getParam("gains/vel", kv_list);
  n.getParam("gains/int", ki_list);
  n.getParam("gains/kiyaw", ki_yaw_);
  n.getParam("gains/kyaw", k_yaw_);

  kx_(0) = kx_list[0];  
  kx_(1) = kx_list[1];  
  kx_(2) = kx_list[2];  
  kv_(0) = kv_list[0];  
  kv_(1) = kv_list[1];  
  kv_(2) = kv_list[2];  
  ki_(0) = ki_list[0];  
  ki_(1) = ki_list[1];  
  ki_(2) = ki_list[2];  

  odom_sub_ = n.subscribe(ros::this_node::getNamespace()+"/odom", 10, odomCallback);
  position_cmd_sub_ = n.subscribe(ros::this_node::getNamespace()+"/position_cmd", 10, positionCmdCallback);
  trpy_command_pub_ = n.advertise<asctec_msgs::SICmd>(ros::this_node::getNamespace()+"/cmd_si", 10);
	
  ros::spin();
  return 0;
}
