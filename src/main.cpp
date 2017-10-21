#include "ros/ros.h"
#include <bits/stdc++.h>
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>
using namespace std;

const int MAX_VEL = 100;

typedef pair<double,double> Vector2D;
typedef double (*max_vel)(const vector<Vector2D>& pos, int ind);
ros::Publisher pub;
krssg_ssl_msgs::gr_Commands final_msgs;
krssg_ssl_msgs::gr_Robot_Command command_msgs;

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
		double v;
		try{
			tan_theta = (y1 - 8*y2 + 8*y3 - y4)/(x1 - 8*x2 + 8*x3 - x4);
		}
		catch(...){
			tan_theta = 3.1416/2.0;
		}	
		// set velocity components
		try{
		v = min(max_vel_fn(pos,i), vi + (vf - vi) * i / sz);
		}
		catch(...){
			v = 0;
		}

		// cout<<"tan_theta = "<<tan_theta<<"v = "<<v<<endl;
		double cos_theta = 1/sqrt(1+tan_theta*tan_theta);
		vel[i].first = v*cos_theta>0.1?v*cos_theta:0;
		vel[i].second = v*tan_theta*cos_theta>0.1?v*tan_theta*cos_theta:0.1;
	}
	return vel;
}

void Callback(const krssg_ssl_msgs::planner_path::ConstPtr& msg)
{
	vector<Vector2D> pos;
	for(int i=0;i<msg->point_array.size();i++)
	{
		Vector2D p;
		p.first = msg->point_array[i].x;
		p.second = msg->point_array[i].y;
		pos.push_back(p);
	}

	vector<Vector2D> vel = velocity_profile_naive(pos, 2, 0);

	cout<<"got velocity, size = "<<vel.size()<<endl;
	// for(int i=0;i<vel.size();i++)
	// {
	// 	cout<<i<<" "<<vel[i].first<<" "<<vel[i].second<<endl;
	// }
	
	command_msgs.id          = 0;
	command_msgs.wheelsspeed = 0;
	command_msgs.veltangent  = -vel[0].second;
	command_msgs.velnormal   = +vel[0].first;
	command_msgs.velangular  = 0;
	command_msgs.kickspeedx  = 0;
	command_msgs.kickspeedz  = 0;
	command_msgs.spinner     = false;

	final_msgs.timestamp      = ros::Time::now().toSec();
	final_msgs.isteamyellow   = false;
	final_msgs.robot_commands = command_msgs;
	cout<<" Sending velnormal = "<<command_msgs.velnormal<<" veltangent = "<<command_msgs.veltangent<<endl;
	pub.publish(final_msgs);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_profiling");

  	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/path_planner_ompl", 1000, Callback);
	pub = n.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000);
	ros::spin();
	return 0;
}