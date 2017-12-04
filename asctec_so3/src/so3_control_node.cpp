#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/SICmd.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <SO3Control.h>
#include "SO3Control.cpp"
#include <tf/transform_datatypes.h>

#define MIN_THRUST 0.0

SO3Control controller;
ros::Publisher si_cmd_pub;
asctec_msgs::SICmd si_command;

bool position_cmd_updated = false, position_cmd_init = false;
Eigen::Vector3d des_pos, des_vel, des_acc, kx, kv;
double des_yaw = 0, des_yaw_dot = 0, ky = 5.0, ky_dot = 2.0;
double current_yaw = 0;
double max_thrust;
bool enable_motors = false;

void publishSO3Command(void)
{
  controller.calculateControl(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot,
                              kx, kv);

  const Eigen::Vector3d &force = controller.getComputedForce();
  const Eigen::Quaterniond &orientation = controller.getComputedOrientation();
  const Eigen::Vector3d &omg_ = controller.getDesiredAngularVelocity();

	if(!isnan(force(2))) {
		tf::Quaternion q(orientation.x(), orientation.y(), orientation.z(), orientation.w());
		double roll, pitch, yaw;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);

		si_command.cmd[0] = true;
		si_command.cmd[1] = true;
		si_command.cmd[2] = true;
		si_command.cmd[3] = true; 
		si_command.thrust = force.norm();
		if(si_command.thrust > max_thrust) {
		  si_command.thrust = 1.0;
		}else if(si_command.thrust < MIN_THRUST) {
		  si_command.thrust = 0.0;
		} else{
		  si_command.thrust = si_command.thrust/max_thrust;
		}
		si_command.roll = roll;
		si_command.pitch = -pitch;
		si_command.yaw = -controller.getComputedYaw(ky, ky_dot, des_yaw, des_yaw_dot);
	}
  si_cmd_pub.publish(si_command);
}

void position_cmd_callback(const asctec_msgs::PositionCmd::ConstPtr& cmd)
{
	if(position_cmd_init) {
		des_pos = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
		des_vel = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
		des_acc = Eigen::Vector3d(cmd->accel.x, cmd->accel.y, cmd->accel.z);
		des_yaw = cmd->yaw[0];
		des_yaw_dot = cmd->yaw[1];

		position_cmd_updated = true;
		publishSO3Command();
	}
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  controller.setYaw(tf::getYaw(odom->pose.pose.orientation));
	controller.setYawDot(odom->twist.twist.angular.z);

  controller.setPosition(position);
  controller.setVelocity(velocity);

	if(!position_cmd_init) {
		des_pos = position;
		des_vel = Eigen::Vector3d(0, 0, 0);
		des_acc = Eigen::Vector3d(0, 0, 0);
		des_yaw = 0.0;
		des_yaw_dot = 0.0;

		publishSO3Command();
		position_cmd_init = true;
	}

	if(!position_cmd_updated) {
		publishSO3Command();
	}
	position_cmd_updated = false;
}

void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "so3_control");
  ros::NodeHandle n("~");

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));

  double mass;
  n.param("mass", mass, 0.55);
	n.param("thrust", max_thrust, 13.184);
  controller.setMass(mass);

  std::vector<double> kx_list, kv_list;
  n.getParam("gains/pos", kx_list);
  n.getParam("gains/vel", kv_list);
	n.getParam("gains/yaw", ky);
	n.getParam("gains/yaw_dot", ky_dot);

  // high level controller gains
  kx(0) = kx_list[0];  
  kx(1) = kx_list[1];  
  kx(2) = kx_list[2];  
  kv(0) = kv_list[0];  
  kv(1) = kv_list[1];  
  kv(2) = kv_list[2];   
//  ky(1) = ky_list[1];  

  ros::Subscriber odom_sub = n.subscribe(ros::this_node::getNamespace()+"/odom", 10, odom_callback);
  ros::Subscriber position_cmd_sub = n.subscribe(ros::this_node::getNamespace()+"/position_cmd", 10, position_cmd_callback);

  ros::Subscriber enable_motors_sub = n.subscribe(ros::this_node::getNamespace()+"/motors", 2, enable_motors_callback);
  si_cmd_pub = n.advertise<asctec_msgs::SICmd>(ros::this_node::getNamespace()+"/cmd_si", 10);
  ros::spin();

  return 0;
}
