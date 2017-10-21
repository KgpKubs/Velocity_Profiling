#include "ros/ros.h"
#include <bits/stdc++.h>
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
using namespace std;

const int MAX_VEL = 100;

typedef pair<double,double> Vector2D;
typedef double (*max_vel)(const vector<Vector2D>& pos, int ind);

double max_vel_naive(const vector<Vector2D>& pos, int ind){
	return MAX_VEL;
}

vector<Vector2D> velocity_profile_naive(const vector<Vector2D>& pos, const double vi, const double vf, max_vel max_vel_fn= max_vel_naive){
	int sz = pos.size();
	vector<Vector2D> vel(sz);
	for(int i=0;i<sz;i++){
		// calculate theta using five-point formula for finite difference derivative
		double tan_theta;
		double x1, x2, x3, x4, y1, y2, y3, y4;
		if(i <= 1){
			x1 = pos[0].first;
			y1 = pos[0].second;
			x2 = pos[0].first;
			y2 = pos[0].second;
		}
		else{
			x1 = pos[i-2].first;
			y1 = pos[i-2].second;
			x2 = pos[i-1].first;
			y2 = pos[i-1].second;
		}
		if(i >= sz-2){
			x3 = pos[sz-1].first;
			y3 = pos[sz-1].second;
			x4 = pos[sz-1].first;
			y4 = pos[sz-1].second;
		}
		else{
			x3 = pos[i+1].first;
			y3 = pos[i+1].second;
			x4 = pos[i+2].first;
			y4 = pos[i+2].second;
		}
		tan_theta = (y1 - 8*y2 + 8*y3 - y4)/(x1 - 8*x2 + 8*x3 - x4);
		// set velocity components
		double v = min(max_vel_fn(pos,i), vi + (vf - vi) * i / sz);
		double cos_theta = 1/sqrt(1+tan_theta*tan_theta);
		vel[i].first = v*cos_theta;
		vel[i].second = v*tan_theta*cos_theta;
	}
	return vel;
}

void Callback(const krssg_ssl_msgs::planner_path::ConstPtr& msg)
{
	vector<Vector2D> pos;
	for(int i=0;i<msg->point_array.size();i++)
	{
		Vector2D p;
		p.x = msg->point_array[i].x;
		p.y = msg->point_array[i].y;
		pos.push_back(p);
	}

	vector<Vector2D> vel = velocity_profile_naive(pos, 0, 0);

	cout<<"got velocity, size = "<<vel.size()<<endl;
}

int main()
{
	ros::Subscriber sub = n.subscribe("/path_planner_ompl", 1000, Callback);
	return 0;
}