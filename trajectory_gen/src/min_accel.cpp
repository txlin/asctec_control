#include <min_accel.h>

using Eigen::MatrixXf;

MinAccel::MinAccel()
{
 /* Trajectory class constructor
  * - Holds A, B, X, T matricies
  * - Can return next waypoint upon call
  */

 /*	Initialize A constants, calculate time components later
	*	0.0  0.0  0.0  1.0
	*	t^3  t^2  t 	 1.0
	*	0.0  0.0  1.0  0.0
	*	3t^2 2t 	2.0	 0.0
	* Initialize empty B matricies
	*/

	A = MatrixXf::Zero(4,4);
	A(0,3) = 1.0;
	A(2,2) = 1.0; 

	Bx = MatrixXf::Zero(4,1);
	By = MatrixXf::Zero(4,1);
  Bz = MatrixXf::Zero(4,1);
	Byaw = MatrixXf::Zero(4,1);

 /*	Initialize trajectory marker
	*	color differentiation between snap, jerk, accel
	*/

	path_.header.frame_id = odom_.header.frame_id;
	path_.type = visualization_msgs::Marker::CUBE_LIST;
	path_.action = visualization_msgs::Marker::ADD;
	path_.ns = "path_accel";
	path_.id = 0;
	path_.scale.x = 0.0125;
	path_.scale.y = 0.0125;
	path_.scale.z = 0.0125;
	path_.color.a = 0.65;
	path_.color.r = 0.8;
	path_.color.b = 0.8;
	continuous = false;
	init = false;
}

void MinAccel::setPubSub(ros::NodeHandle *n, float rate)
{
  timer = n->createTimer(ros::Duration(1/rate), &MinAccel::timerCallback, this);
	wpt_sub = n->subscribe(ros::this_node::getNamespace()+"/waypoints", 100, &MinAccel::waypointCallback, this);
	odom_sub = n->subscribe(ros::this_node::getNamespace()+"/odom", 10, &MinAccel::odomCallback, this);

  pos_goal = n->advertise<asctec_msgs::PositionCmd>(ros::this_node::getNamespace()+"/position_cmd", 10); 																	// Position goals to linear and nonlinear controllers
  status = n->advertise<std_msgs::Bool>(ros::this_node::getNamespace()+"/status", 10);							 																			// Trajectory completion status
  wp_viz = n->advertise<visualization_msgs::Marker>(ros::this_node::getNamespace()+"/asctec_viz", 10);
}

void MinAccel::setCont(bool cont_)
{
	continuous = cont_;
}

void MinAccel::addWaypoint(const asctec_msgs::WaypointCmd::ConstPtr& wp)
{
 /* Grab wp from WaypointCmd msg
  * Calculate new X matrix and add to X vector
  * Set init based on if continuing or starting a trajectory
  */

	if(!wp->time && (!wp->desV || !wp->desA)) { ROS_INFO("Ignored: Time = 0 and no decision parameters set"); return;}
	float time = wp->time;
	if(!time) time = setTime(wp, wp->desV, wp->desA);
	 /*	Calculate a time estimate based on theoretical desired velocity and acceleration
		*/

 /*	Initialize A constants, calculate time components later
	*	0.0  0.0  0.0  1.0
	*	t^3  t^2  t 	 1.0
	*	0.0  0.0  1.0  0.0
	*	3t^2 2t 	2.0	 0.0
	* Initialize empty B matricies
	*/

  if(T.size() == 0 && !wp->cont) {
	 /*	Set B matrix according to current state
		* reset t0 for spline position goal extraction
		*/

  	Bx(0,0) = odom_.pose.pose.position.x;
    Bx(2,0) = odom_.twist.twist.linear.x;
   
  	By(0,0) = odom_.pose.pose.position.y;
    By(2,0) = odom_.twist.twist.linear.y;
      
		if(wp->hold_z) {
			Bz(0,0) = odom_.pose.pose.position.z;
		  Bz(2,0) = odom_.twist.twist.linear.z;
		    
		}else {
			Bz(0,0) = Bz(1,0);
		  Bz(2,0) = Bz(3,0);
		}

  	Byaw(0,0) = tf::getYaw(odom_.pose.pose.orientation);
    Byaw(2,0) = odom_.twist.twist.angular.z;

    t0 = ros::Time::now();

  }else {
	 /*	Set B matrix according to last segment's final state
		*/

  	Bx(0,0) = Bx(1,0);
    Bx(2,0) = Bx(3,0);
      
  	By(0,0) = By(1,0);
    By(2,0) = By(3,0);
      
  	Bz(0,0) = Bz(1,0);
    Bz(2,0) = Bz(3,0);

  	Byaw(0,0) = Byaw(1,0);
    Byaw(2,0) = Byaw(3,0);

  }
 /*	Set B matrix final state
	*/
  Bx(1,0) = wp->position.x;																		
  Bx(3,0) = wp->velocity.x;
    
  By(1,0) = wp->position.y;
  By(3,0) = wp->velocity.y;
  
  Bz(1,0) = wp->position.z;
  Bz(3,0) = wp->velocity.z;
 
  Byaw(1,0) = wp->yaw[0];
  Byaw(3,0) = wp->yaw[1];

 /*	Set A matrix according to calculated time
 	*	-----------
	*	t^3 t^2 t 1
	*	-----------
	*	3t^2 2t 2 0
	*/

	for(int i=3; i>=0; i--) {																		
		A(1,3-i) = pow(time,i);
		A(3,3-i) = i*pow(time,i-1);
	}

 /*	Solve X matricies for x,y,z,yaw
	*	Store X result in vector 
	*	Store time result in time vector for evaluation
	*/

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

  T.push_back(time);
	lock.push_back(wp->lock_yaw);
}

void MinAccel::setState(const nav_msgs::Odometry::ConstPtr& odom) {
  odom_ = *odom;
}

bool MinAccel::getStatus(void) {
 /*	Check if trajectory is completed
	*/
	return T.size() == 0;
}

float MinAccel::setTime(const asctec_msgs::WaypointCmd::ConstPtr& cmd, float desV, float desA) {
 /*	Self-calculation of spline duration
	* based on desired velocity and acceleration
	* calculates time as a trapezoid, time to ramp up to desV based on desA
	*/

	float tA0 = std::sqrt(std::pow((odom_.twist.twist.linear.x - desV)/desA,2)+std::pow((odom_.twist.twist.linear.y - desV)/desA,2));
	float tAf = std::sqrt(std::pow((cmd->velocity.x - desV)/desA,2)+std::pow((cmd->velocity.y - desV)/desA,2));
	float tD = std::sqrt(std::pow(cmd->position.x-odom_.pose.pose.position.x,2)+std::pow(cmd->position.y-odom_.pose.pose.position.y,2))/desV;
	return tD+tA0+tAf;
}

visualization_msgs::Marker *MinAccel::deleteMarker(void) {
 /*	Empty and clear visualization marker
	*/

	path_.points.clear();
	path_.action = 3;
	return &path_;
}

visualization_msgs::Marker *MinAccel::getMarker(void) {
 /*	rebuild marker msg according to remaining points
	* points added at 10hz intervals
	* returns an empty marker if no trajectory is running
	*/

	if(T.size() == 0) return &path_;
	double ts = ros::Time::now().toSec() - t0.toSec();
	if(ts >= T.front()) ts = T.front();
	path_.points.clear();
	for(double t=ts; t<T.front(); t+=0.1) {
		geometry_msgs::Point p;
		for(int i=3; i>=0; i--) {
		  p.x += Xx.front()->operator()(3-i,0)*pow(t,i);
		  p.y += Xy.front()->operator()(3-i,0)*pow(t,i);
		  p.z += Xz.front()->operator()(3-i,0)*pow(t,i);
		}
		path_.points.push_back(p);
	}
	path_.action = visualization_msgs::Marker::ADD;
	path_.lifetime = ros::Duration(T.front());
	path_.header.frame_id = odom_.header.frame_id;
	path_.header.stamp = ros::Time::now();
	return &path_;
}

void MinAccel::resetWaypoints(void) {
 /*	deallocate memory and flush X matrix vector buffers
	* called when waypointcmd reset boolean is set
	*/

	if(T.size() == 0) return;
	ROS_INFO("Clearing %i waypoints...", int(T.size()));

	for (std::vector<MatrixXf *>::iterator i = Xx.begin() ; i != Xx.end(); ++i) delete *i;
	for (std::vector<MatrixXf *>::iterator i = Xy.begin() ; i != Xy.end(); ++i) delete *i;
	for (std::vector<MatrixXf *>::iterator i = Xz.begin() ; i != Xz.end(); ++i) delete *i;
	for (std::vector<MatrixXf *>::iterator i = Xyaw.begin() ; i != Xyaw.end(); ++i) delete *i;

  Xx.clear();
  Xy.clear();
  Xz.clear();
  Xyaw.clear();
  T.clear();
}

asctec_msgs::PositionCmd* MinAccel::getNextCommand(void) { 
 /*	calculates next position goal according to current time
	* when switching splines, deletes spline point and resets time
	* returns next position as asctec_msgs::PositionCmd
	* currently does not control yaw
	* TODO: Change spline deletion from online to after completion
	*/
 		
  if(T.size() != 0) {
  	double t = ros::Time::now().toSec() - t0.toSec();
    if(t >= T.front()) {
      t0 = ros::Time::now();
			t = 0.0;
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
			cmd.header.stamp = ros::Time::now();
      for(int i=3; i>=0; i--) {
        cmd.position.x += Xx.front()->operator()(3-i,0)*pow(t,i);
        cmd.velocity.x += i*Xx.front()->operator()(3-i,0)*pow(t,i-1);
        cmd.accel.x += i*(i-1)*Xx.front()->operator()(3-i,0)*pow(t,i-2);

        cmd.position.y += Xy.front()->operator()(3-i,0)*pow(t,i);
        cmd.velocity.y += i*Xy.front()->operator()(3-i,0)*pow(t,i-1);
        cmd.accel.y += i*(i-1)*Xy.front()->operator()(3-i,0)*pow(t,i-2);
	
        cmd.position.z += Xz.front()->operator()(3-i,0)*pow(t,i);
        cmd.velocity.z += i*Xz.front()->operator()(3-i,0)*pow(t,i-1);
        cmd.accel.z += i*(i-1)*Xz.front()->operator()(3-i,0)*pow(t,i-2);

				if(!lock.front()) {
		      cmd.yaw[0] += Xyaw.front()->operator()(3-i,0)*pow(t,i);
		      cmd.yaw[1] += i*Xyaw.front()->operator()(3-i,0)*pow(t,i-1);
		      cmd.yaw[2] += i*(i-1)*Xyaw.front()->operator()(3-i,0)*pow(t,i-2);
				}else {
					cmd.yaw[0] = Byaw(0,0);
				}
      }
			if (isnan(cmd.velocity.x)) cmd.velocity.x = cmd_.velocity.x;
			if (isnan(cmd.velocity.y)) cmd.velocity.y = cmd_.velocity.y;
			if (isnan(cmd.velocity.z)) cmd.velocity.z = cmd_.velocity.z;
			if (isnan(cmd.accel.x)) cmd.accel.x = cmd_.accel.x;
			if (isnan(cmd.accel.y)) cmd.accel.y = cmd_.accel.y;
			if (isnan(cmd.accel.z)) cmd.accel.z = cmd_.accel.z;
      cmd_ = cmd;
    }
  }
  return &cmd_;
}

void MinAccel::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  setState(msg);
}

void MinAccel::waypointCallback(const asctec_msgs::WaypointCmd::ConstPtr& msg)
{
	if(msg->reset) resetWaypoints();
	if(msg->reset) wp_viz.publish(*deleteMarker());
	addWaypoint(msg);
	wp_viz.publish(*getMarker());
}

void MinAccel::timerCallback(const ros::TimerEvent& event)
{
	std_msgs::Bool temp;
	temp.data = getStatus();
	status.publish(temp);
	if(!temp.data || continuous) {
		pos_goal.publish(*getNextCommand());
	}
}
