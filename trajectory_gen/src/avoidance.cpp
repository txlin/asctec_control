#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <asctec_msgs/WaypointCmd.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#define maxV 0.5
#define maxA 0.3

std::string topic, frame;
asctec_msgs::WaypointCmd goal;
nav_msgs::Odometry odom_;
ros::Publisher wpt_pub, obs_pub;

typedef struct obstacle
{
	float x,y,r,ar,sr;
}obs;

typedef struct node
{
	bool goal;
  float x,y;
  std::vector<std::pair<struct node*, float> > next;
	node()
	{
		goal = false;
	}
}n;

void printNode(struct node *n0) {
	std::cout << "Head: " << n0->x << ", " << n0->y << std::endl;
	for(std::vector<std::pair<struct node*, float> >::iterator j=n0->next.begin(); j != n0->next.end(); j++) {
		std::cout << "Next: " << j->first->x << " " << j->first->y << " Cost: " << j->second << std::endl;
	}
}

void printNodes(struct node *n0, std::vector<std::pair<struct node, struct node> > *nodes) {
	std::cout << "Head: " << n0->x << ", " << n0->y << std::endl;
	for(std::vector<std::pair<struct node*, float> >::iterator j=n0->next.begin(); j != n0->next.end(); j++) {
		std::cout << "Next: " << j->first->x << " " << j->first->y << " Cost: " << j->second << std::endl;
	}
	std::cout << std::endl;
	for(std::vector<std::pair<struct node, struct node> >::iterator i=nodes->begin(); i != nodes->end(); i++) {
		std::cout << "Pair:" << std::endl;
		std::cout << "Node: " << i->first.x << ", " << i->first.y << std::endl;
		for(std::vector<std::pair<struct node*, float> >::iterator j=i->first.next.begin(); j != i->first.next.end(); j++) {
			std::cout << "Next: " << j->first->x << " " << j->first->y << " Cost: " << j->second << std::endl;
		}
		std::cout << std::endl;
		std::cout << "Node: " << i->second.x << ", " << i->second.y << std::endl;
		for(std::vector<std::pair<struct node*, float> >::iterator j=i->second.next.begin(); j != i->second.next.end(); j++) {
			std::cout << "Next: " << j->first->x << " " << j->first->y << " Cost: " << j->second << std::endl;
		}
		std::cout << std::endl;
	}
}

void printObs(std::vector<struct obstacle> *obs) {
	for(std::vector<struct obstacle>::iterator i=obs->begin(); i != obs->end(); i++) {
		std::cout << "Obs:     " << i->x << ", " << i->y << std::endl;
		std::cout << "Radius:  " << i->r << std::endl;
		std::cout << "Avoid R: " << i->ar << std::endl;
		std::cout << std::endl;
	}
}

void printPath(std::pair<std::vector<std::pair<float,float> >,float> *path) {
	std::cout << "Path cost: " << path->second << std::endl;
	std::cout << "Path: ";
	for(std::vector<std::pair<float,float> >::iterator i=path->first.begin(); i != path->first.end(); i++) {
		std::cout << i->first << ", " << i->second << "; ";
	}
	std::cout << std::endl;
}

bool pathIsClear(struct node *n0, struct node *nf, std::vector<struct obstacle> *obs)
{
  float a = -(nf->y-n0->y)/(nf->x-n0->x);
  float b = 1;
  float c = -(n0->y)+(-a*n0->x);

  for(std::vector<obstacle>::iterator i = obs->begin(); i != obs->end(); i++) {
    float d = std::abs(a*i->x+b*i->y+c)/(std::sqrt(std::pow(a,2)+std::pow(b,2)));
    if(d<i->sr) return false;
  }
  return true;
}

struct node *getValidNodes(struct node *n0, struct node *ng, std::vector<struct obstacle> *obs, std::vector<std::pair<struct node, struct node> > *nodes)
{
	for(std::vector<std::pair<struct node, struct node> >::iterator i=nodes->begin(); i != nodes->end(); i++) {
		if(pathIsClear(n0, &((*i).first), obs)) {	
			std::pair<struct node*, float> *temp = new std::pair<struct node*, float>;
			temp->first = &((*i).first);
			temp->second = std::sqrt(std::pow(n0->x-(&((*i).first))->x,2)+std::pow(n0->y-(&((*i).first))->y,2));
			n0->next.push_back(*temp);
		}
		if(pathIsClear(n0, &((*i).second), obs)) {
			std::pair<struct node*, float> *temp = new std::pair<struct node*, float>;
			temp->first = &((*i).second);
			temp->second = std::sqrt(std::pow(n0->x-(&((*i).second))->x,2)+std::pow(n0->y-(&((*i).second))->y,2));
			n0->next.push_back(*temp);
		}
	}
	if(pathIsClear(n0, ng, obs)) {
			std::pair<struct node*, float> *temp = new std::pair<struct node*, float>;
			temp->first = ng;
			temp->second = std::sqrt(std::pow(n0->x-ng->x,2)+std::pow(n0->y-ng->y,2));
			n0->next.push_back(*temp);
	}
	return n0;
}

struct node *buildTree(struct node *n0, struct node *ng, std::vector<struct obstacle> *obs, std::vector<std::pair<struct node, struct node> > *nodes) 
{ 
	n0 = getValidNodes(n0, ng, obs, nodes);
  for(std::vector<std::pair<struct node, struct node> >::iterator i=nodes->begin(); i != nodes->end(); i++) {
		std::vector<std::pair<struct node, struct node> > *v1 = new std::vector<std::pair<struct node, struct node> >;
		v1->assign(i+1, nodes->end());
  	i->first = *getValidNodes(&(i->first), ng, obs, v1);
		i->second = *getValidNodes(&(i->second), ng, obs, v1);
  }
  return n0;
}

std::vector<std::pair<struct node, struct node> > *generateNodes(struct node *n0, struct node *ng, std::vector<std::pair<struct node, struct node> > *nodes, std::vector<struct obstacle> *obs)
{
  float alpha = atan2((*ng).y-(*n0).y, (*ng).x-(*n0).x);

  for(std::vector<obstacle>::iterator i = obs->begin(); i != obs->end(); i++) {
    struct node *n1 = new struct node; 
		struct node *n2 = new struct node; 

    std::pair<struct node, struct node> *node_pair = new std::pair<struct node, struct node>;
    n1->x = (*i).x + (*i).r * sin(alpha);
    n2->x = (*i).x - (*i).r * sin(alpha);
    n1->y = (*i).y - (*i).r * cos(alpha);
    n2->y = (*i).y + (*i).r * cos(alpha);
    node_pair->first = *n1;
    node_pair->second = *n2;
    nodes->push_back(*node_pair);
  }
  return nodes; 
}

std::vector<struct obstacle> *orderObstacles(std::vector<struct obstacle> *obs, struct node *ng) {
	bool sorted = false;
	while(!sorted) {
		sorted = true;
		float d1 = std::sqrt(std::pow(ng->x-((obs->front()).x),2)+std::pow(ng->y-((obs->front()).y),2));
		for(std::vector<obstacle>::iterator i = obs->begin()+1; i != obs->end(); i++) {
			float d2 = std::sqrt(std::pow(ng->x-i->x,2)+std::pow(ng->y-i->y,2));
			if(d2 > d1) {std::iter_swap(i,i-1); sorted = false;}
			d1 = d2;
		}
	}
	return obs;
}

std::pair<std::vector<std::pair<float,float> >,float> *searchTree(std::pair<std::vector<std::pair<float,float> >,float> *path, struct node *n0)
{
	if(n0->goal){path->first.push_back(std::make_pair(n0->x,n0->y)); return path;}
	if(n0->next.size() == 0){path->second += 1000; return path;}

	bool first = true;
	path->first.push_back(std::make_pair(n0->x,n0->y));
	std::pair<std::vector<std::pair<float,float> >,float> temp2;

	for(std::vector<std::pair<struct node*, float> >::iterator i=n0->next.begin(); i != n0->next.end(); i++) {
		std::pair<std::vector<std::pair<float,float> >,float> temp = *path;
		temp.second += i->second;		
		temp = *searchTree(&temp, i->first);
		if(first || temp.second < temp2.second) {temp2 = temp; first = false;}
	}
	*path = temp2;
	return path;
}

void publishRedirect(struct node *n0, struct node *ng, std::pair<std::vector<std::pair<float,float> >,float> *path)
{
	bool first = true;
	for(std::vector<std::pair<float,float> >::iterator i=path->first.begin(); i != path->first.end()-1; i++) {
		float angle = atan2((*ng).y-(*(i+1)).second, (*ng).x-(*(i+1)).first);
		asctec_msgs::WaypointCmd cmd;
		if(first) {cmd.reset = true; first = false;}
		cmd.position.x = (i+1)->first;
		cmd.position.y = (i+1)->second;
		cmd.position.z = odom_.pose.pose.position.z;
		cmd.time = std::sqrt(std::pow(i->first-(i+1)->first,2)+std::pow(i->second-(i+1)->second,2));
		cmd.time += maxV/maxA;
		if(i != path->first.end()-2) {
			cmd.velocity.x = maxV*cos(angle);
			cmd.velocity.y = maxV*sin(angle);
		}
		
		wpt_pub.publish(cmd);
	}
}

void redirectCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	struct node n0, ng;
	n0.x = odom_.pose.pose.position.x;
	n0.y = odom_.pose.pose.position.y;

	ng.x = msg->x;
	ng.y = msg->y;
	ng.goal = true;
	
	std::vector<struct obstacle> obs;
	for(int i=0; i<4; i++) {
		struct obstacle *o = new struct obstacle;
		o->x = -1 + 6*float(rand() % 100) / 100;
		o->y = -1 + 6*float(rand() % 100) / 100;
		o->r = 1.2;
		o->sr = 0.75;
		o->ar = 0.3;
		obs.push_back(*o);
	}
	obs = *orderObstacles(&obs, &ng);
	visualization_msgs::Marker obstacles;
	obstacles.action = 3;
	visualization_msgs::Marker empty;
	obstacles.points = empty.points;
	obs_pub.publish(obstacles);

	obstacles.header.frame_id = frame;
	obstacles.type = visualization_msgs::Marker::POINTS;
	obstacles.id = 0;
	obstacles.scale.x = 0.025;
	obstacles.scale.y = 0.025;
	obstacles.scale.z = 0.025;
	obstacles.color.a = 0.7;
	obstacles.color.r = 0.7;
	obstacles.action = visualization_msgs::Marker::ADD;

	for(std::vector<obstacle>::iterator i = obs.begin(); i != obs.end(); i++) {
		for(float j=-M_PI; j<M_PI; j+=0.1) {
			geometry_msgs::Point p;
			p.x = i->x+cos(j)*i->r;
			p.y = i->y+sin(j)*i->r;
			p.z = odom_.pose.pose.position.z;
			obstacles.points.push_back(p);
		}
		for(float j=-M_PI; j<M_PI; j+=0.1) {
			geometry_msgs::Point p;
			p.x = i->x+cos(j)*i->ar;
			p.y = i->y+sin(j)*i->ar;
			p.z = odom_.pose.pose.position.z;
			obstacles.points.push_back(p);
		}
		for(float j=-M_PI; j<M_PI; j+=0.1) {
			geometry_msgs::Point p;
			p.x = i->x+cos(j)*i->sr;
			p.y = i->y+sin(j)*i->sr;
			p.z = odom_.pose.pose.position.z;
			obstacles.points.push_back(p);
		}
	}
	obs_pub.publish(obstacles);

	std::vector<std::pair<struct node, struct node> > nodes;
	nodes = *generateNodes(&n0, &ng, &nodes, &obs);
	n0 = *buildTree(&n0, &ng, &obs, &nodes);

	std::pair<std::vector<std::pair<float,float> >,float> path;
	path = *searchTree(&path, &n0);
	if(path.second < 1000) {publishRedirect(&n0, &ng, &path);}
	else {ROS_INFO("No safe path to %.2f, %.2f", ng.x, ng.y);}
	obs.clear();
	
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_ = *msg;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "redirect_node");
	ros::NodeHandle nh;

  ros::param::get("~topic", topic);
  ros::param::get("~world_frame", frame);

	/* -------------------- Publishers, and Subscribers -------------------- */
  wpt_pub = nh.advertise<asctec_msgs::WaypointCmd>(topic + "/waypoints", 10); 																			// Position goals to linear and nonlinear controllers 
  obs_pub = nh.advertise<visualization_msgs::Marker>(topic + "/obstacles", 10); 																		// Obstacle positions
  ros::Subscriber redirect_sub = nh.subscribe(topic + "/redirect", 1, redirectCallback);														// Redirect data
  ros::Subscriber odom_sub = nh.subscribe(topic + "/odom", 1, odomCallback);																				// odom data

	ros::spin();
}
