#include <min_accel.h>

using Eigen::MatrixXf;

MinAccel::MinAccel()
{
 /* Trajectory class constructor
  * - Holds A, B, X, T matricies
  * - Can return next waypoint upon call
  */

	A = MatrixXf::Zero(6,6);
	A(0,5) = 1.0;
	A(2,4) = 1.0;
	A(4,3) = 2.0; 

	Bx = MatrixXf::Zero(6,1);
	By = MatrixXf::Zero(6,1);
  Bz = MatrixXf::Zero(6,1);
	Byaw = MatrixXf::Zero(6,1);
}

void MinAccel::addWaypoint(asctec_msgs::MinAccelCmd wp)
{
 /* Grab wp from MinAccelCommand msg
  * Calculate new X matrix and add to X vector
  * Set init based on if continuing or starting a trajectory
  */
	if(wp.time == 0){ ROS_INFO("Error: time = 0"); return;}

  if(T.size() == 0) {
  	Bx(0,0) = odom_.pose.pose.position.x;
    Bx(2,0) = odom_.twist.twist.linear.x;
    Bx(4,0) = 0.0;
   
  	By(0,0) = odom_.pose.pose.position.y;
    By(2,0) = odom_.twist.twist.linear.y;
    By(4,0) = 0.0;
      
  	Bz(0,0) = odom_.pose.pose.position.z;
    Bz(2,0) = odom_.twist.twist.linear.z;
    Bz(4,0) = 0.0;
      
  	Byaw(0,0) = odom_.pose.pose.orientation.z;
    Byaw(2,0) = odom_.twist.twist.angular.z;
    Byaw(4,0) = 0.0;
    t0 = ros::Time::now();

  }else {
  	Bx(0,0) = Bx(1,0);
    Bx(2,0) = Bx(3,0);
    Bx(4,0) = Bx(5,0);
      
  	By(0,0) = By(1,0);
    By(2,0) = By(3,0);
    By(4,0) = By(5,0);
      
  	Bz(0,0) = Bz(1,0);
    Bz(2,0) = Bz(3,0);
    Bz(4,0) = Bz(5,0);
      
  	Byaw(0,0) = Byaw(1,0);
    Byaw(2,0) = Byaw(3,0);
    Byaw(4,0) = Byaw(5,0);

  }
  Bx(1,0) = wp.position.x;
  Bx(3,0) = wp.velocity.x;
  Bx(5,0) = wp.accel.x;
    
  By(1,0) = wp.position.y;
  By(3,0) = wp.velocity.y;
  By(5,0) = wp.accel.y;
    
  Bz(1,0) = wp.position.z;
  Bz(3,0) = wp.velocity.z;
  Bz(5,0) = wp.accel.z;
    
  Byaw(1,0) = wp.yaw[0];
  Byaw(3,0) = wp.yaw[1];
  Byaw(5,0) = wp.yaw[2];

	for(int i=5; i>=0; i--) {
		A(1,5-i) = pow(wp.time,i);
		A(3,5-i) = i*pow(wp.time,i-1);
		A(5,5-i) = i*(i-1)*pow(wp.time,i-2);
	}

  MatrixXf * x = new MatrixXf;
	*x = A.colPivHouseholderQr().solve(Bx);
	Xx.push_back(x);

  MatrixXf * y = new MatrixXf;
	*y = A.colPivHouseholderQr().solve(By);
	Xy.push_back(y);

  MatrixXf * z = new MatrixXf;
	*z = A.colPivHouseholderQr().solve(Bz);
	Xz.push_back(z);

  MatrixXf * yaw = new MatrixXf;
	*yaw = A.colPivHouseholderQr().solve(Byaw);
	Xyaw.push_back(yaw);

  T.push_back(wp.time);
}

void MinAccel::setState(nav_msgs::Odometry odom) {
  odom_ = odom;
}

bool MinAccel::getStatus(void) {
	return T.size() == 0;
}

void MinAccel::resetWaypoints(void) {
	if(T.size() == 0){ ROS_INFO("Waypoint List Empty"); return;}
	delete Xx.front();
	delete Xy.front();
	delete Xz.front();
	delete Xyaw.front();

  T.erase(T.begin());
  Xx.erase(Xx.begin());
  Xy.erase(Xy.begin());
  Xz.erase(Xz.begin());
  Xyaw.erase(Xyaw.begin());
}

asctec_msgs::PositionCmd* MinAccel::getNextCommand(void) {  		
  if(T.size() != 0) {
  	double t = ros::Time::now().toSec() - t0.toSec();
    if(t >= T.front()) {
      t0 = ros::Time::now();
			delete Xx.front();
			delete Xy.front();
			delete Xz.front();
			delete Xyaw.front();

      T.erase(T.begin());
      Xx.erase(Xx.begin());
      Xy.erase(Xy.begin());
      Xz.erase(Xz.begin());
      Xyaw.erase(Xyaw.begin());

    }
    if(T.size() != 0) {
      asctec_msgs::PositionCmd cmd;

      for(int i=5; i>=0; i--) {
        cmd.position.x += Xx.front()->operator()(5-i,0)*pow(t,i);
        cmd.velocity.x += i*Xx.front()->operator()(5-i,0)*pow(t,i-1);
        cmd.accel.x += i*(i-1)*Xx.front()->operator()(5-i,0)*pow(t,i-2);

        cmd.position.y += Xy.front()->operator()(5-i,0)*pow(t,i);
        cmd.velocity.y += i*Xy.front()->operator()(5-i,0)*pow(t,i-1);
        cmd.accel.y += i*(i-1)*Xy.front()->operator()(5-i,0)*pow(t,i-2);
	
        cmd.position.z += Xz.front()->operator()(5-i,0)*pow(t,i);
        cmd.velocity.z += i*Xz.front()->operator()(5-i,0)*pow(t,i-1);
        cmd.accel.z += i*(i-1)*Xz.front()->operator()(5-i,0)*pow(t,i-2);

        cmd.yaw[0] += Xyaw.front()->operator()(5-i,0)*pow(t,i);
        cmd.yaw[1] += i*Xyaw.front()->operator()(5-i,0)*pow(t,i-1);
        cmd.yaw[2] += i*(i-1)*Xyaw.front()->operator()(5-i,0)*pow(t,i-2);

      }
      cmd_ = cmd;
    }
  }
  return &cmd_;
}
