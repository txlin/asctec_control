#include <SO3Control.h>
#include <iostream>

SO3Control::SO3Control()
  : mass_(1.282),
    g_(9.81)
{
}

void SO3Control::setMass(const double mass)
{
  mass_ = mass;
}

void SO3Control::setGravity(const double g)
{
  g_ = g;
}

void SO3Control::setPosition(const Eigen::Vector3d &position)
{
  pos_ = position;
}

void SO3Control::setVelocity(const Eigen::Vector3d &velocity)
{
  vel_ = velocity;
}

void SO3Control::setYaw(const double &yaw)
{
  yaw_ = yaw;
}

void SO3Control::setYawDot(const double &yaw_dot)
{
  yaw_dot_ = yaw_dot;
}

void SO3Control::calculateControl(const Eigen::Vector3d &des_pos,
                                  const Eigen::Vector3d &des_vel,
                                  const Eigen::Vector3d &des_acc,
                                  const double des_yaw,
                                  const double des_yaw_dot,
                                  const Eigen::Vector3d &kx,
                                  const Eigen::Vector3d &kv)
{
  force_.noalias() = kx.asDiagonal() * (des_pos - pos_) +
                     kv.asDiagonal() * (des_vel - vel_) +
                     mass_ * g_ * Eigen::Vector3d(0, 0, 1) +
                     mass_ * des_acc;
  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

  if(force_.norm() > 1e-6)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  Eigen::Matrix3d R;
  R << b1c, b2c, b3c;

  orientation_ = Eigen::Quaterniond(R);
  omg_ << 0,0,des_yaw_dot;
}

const Eigen::Vector3d &SO3Control::getComputedForce(void)
{
  return force_;
}

const double &SO3Control::getComputedYaw(const double kyaw, 
																				 const double kdyaw,
																				 const double des_yaw,
																				 const double des_yaw_dot)
{
  double e = des_yaw-yaw_;
	while (e > M_PI) {
		e -= 2*M_PI;
	}
	while (e < -M_PI) {
		e += 2*M_PI;
	}
	return kyaw*e + kdyaw*(des_yaw_dot-yaw_dot_);
}

const Eigen::Quaterniond &SO3Control::getComputedOrientation(void)
{
  return orientation_;
}

const Eigen::Vector3d &SO3Control::getDesiredAngularVelocity(void)
{
  return omg_;
}
