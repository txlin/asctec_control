#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

#include <Eigen/Geometry>

class PIDControl
{
 public:
  PIDControl();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d &position);
  void setVelocity(const Eigen::Vector3d &velocity);
  void setYaw(const double current_yaw);
  void setMaxIntegral(const double max_integral);
  void resetIntegrals(void);

  void calculateControl(const Eigen::Vector3d &des_pos,
                        const Eigen::Vector3d &des_vel,
                        const Eigen::Vector3d &des_acc,
                        const double des_yaw,
                        const Eigen::Vector3d &kx,
                        const Eigen::Vector3d &kv,
                        const Eigen::Vector3d &ki,
                        const double ki_yaw,
												const double k_yaw);
                  

  const Eigen::Vector4d &getControls(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Inputs for the controller
  double mass_;
  double g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double current_yaw_;
  Eigen::Vector3d pos_int_;
  double yaw_int_;
  double max_pos_int_;

  // Outputs of the controller
  Eigen::Vector4d trpy_;
};

#endif

