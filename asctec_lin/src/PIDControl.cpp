#include "PIDControl.h"
#include <iostream>

PIDControl::PIDControl()
  : mass_(0.5),
    g_(9.81),
    yaw_int_(0.0),
    max_pos_int_(0.5)
{
}

void PIDControl::setMass(const double mass)
{
  mass_ = mass;
}

void PIDControl::setGravity(const double g)
{
  g_ = g;
}

void PIDControl::setPosition(const Eigen::Vector3d &position)
{
  pos_ = position;
}

void PIDControl::setVelocity(const Eigen::Vector3d &velocity)
{
  vel_ = velocity;
}

void PIDControl::setYaw(const double current_yaw)
{
  current_yaw_ = current_yaw;
}

void PIDControl::setMaxIntegral(const double max_integral)
{
  max_pos_int_ = max_integral;
}

void PIDControl::calculateControl(const Eigen::Vector3d &des_pos,
                                  const Eigen::Vector3d &des_vel,
                                  const Eigen::Vector3d &des_acc,
                                  const double des_yaw,
                                  const Eigen::Vector3d &kx,
                                  const Eigen::Vector3d &kv,
                                  const Eigen::Vector3d &ki,
                                  const double ki_yaw,
																	const double k_yaw_)
{
  Eigen::Vector3d e_pos = (des_pos - pos_);
  Eigen::Vector3d e_vel = (des_vel - vel_);
  for(int i = 0; i < 3; i++)
  {
    if(kx(i) != 0)
      pos_int_(i) += ki(i)*e_pos(i);

    // Limit integral term
    if(pos_int_(i) >= max_pos_int_)
      pos_int_(i) = max_pos_int_;
    else if(pos_int_(i) <= -max_pos_int_)
      pos_int_(i) = -max_pos_int_;
  }

  //std::cout << pos_int_.transpose() << std::endl;
  Eigen::Vector3d force_des = kx.asDiagonal()*e_pos + pos_int_ + kv.asDiagonal()*e_vel + mass_*des_acc;
  double roll_des = force_des(0)*sin(current_yaw_) - force_des(1)*cos(current_yaw_);
  double pitch_des = force_des(0)*cos(current_yaw_) + force_des(1)*sin(current_yaw_);

  double e_yaw = (des_yaw - current_yaw_);

  yaw_int_ += e_yaw;
  if(yaw_int_ > M_PI)
    yaw_int_ = M_PI;
  else if(yaw_int_ < -M_PI)
    yaw_int_ = -M_PI;

  trpy_(0) = force_des(2) + mass_*g_;
  trpy_(1) = roll_des;
  trpy_(2) = pitch_des;
  //trpy_(3) = k_yaw_*e_yaw + ki_yaw*yaw_int_;
	trpy_(3) = des_yaw;
}

const Eigen::Vector4d &PIDControl::getControls(void)
{
  return trpy_;
}

void PIDControl::resetIntegrals(void)
{
  pos_int_ = Eigen::Vector3d::Zero();
  yaw_int_ = 0;
}

