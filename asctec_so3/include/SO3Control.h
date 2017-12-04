#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>

class SO3Control
{
 public:
  SO3Control();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d &position);
  void setVelocity(const Eigen::Vector3d &velocity);
	void setYaw(const double &yaw);
	void setYawDot(const double &yaw_dot);
  void calculateControl(const Eigen::Vector3d &des_pos,
                        const Eigen::Vector3d &des_vel,
                        const Eigen::Vector3d &des_acc,
                        const double des_yaw,
                        const double des_yaw_dot,
                        const Eigen::Vector3d &kx,
                        const Eigen::Vector3d &kv);

  const Eigen::Vector3d &getComputedForce(void);
  const Eigen::Quaterniond &getComputedOrientation(void);
  const Eigen::Vector3d &getDesiredAngularVelocity(void);
	const double &getComputedYaw(const double kyaw, 
															 const double kdyaw,
															 const double des_yaw,
															 const double des_yaw_dot);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Inputs for the controller
  double mass_;
  double g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
	double yaw_;
	double yaw_dot_;
  // Outputs of the controller
  Eigen::Vector3d force_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d omg_;
};

#endif
