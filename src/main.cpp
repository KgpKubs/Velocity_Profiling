#include "ros/ros.h"
#include <bits/stdc++.h>
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/pid_message.h"
#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>
using namespace std;
#include <time.h>

// std::vector<Vector2D> oldPos;
float dt;
const int MAX_VEL = 100;
int BOT_ID = 0;
float lastT;
struct timeval t;


typedef pair<double,double> Vector2D;
typedef double (*max_vel)(const vector<Vector2D>& pos, int ind);
ros::Publisher pub;
krssg_ssl_msgs::gr_Commands final_msgs;
krssg_ssl_msgs::gr_Robot_Command command_msgs;
krssg_ssl_msgs::pid_message pidMessage;
float home_pos_theta[6] = {0,0,0,0,0,0};
std::vector<krssg_ssl_msgs::point_2d> homePos(6);
krssg_ssl_msgs::point_2d lastExpected;



double max_vel_naive(const vector<Vector2D>& pos, int ind){
	return MAX_VEL;
}
vector<Vector2D> velocity_profile_naive2(const vector<Vector2D>& pos, const double vi, const double vf, max_vel max_vel_fn= max_vel_naive){
	int sz = pos.size();
	double Dy;
	vector<Vector2D> vel(sz);
	for(int i=0;i<sz;i++){
		// calculate theta using five-point formula for finite difference derivative
		double tan_theta;
		Vector2D n1, n2, n;
		double v;

		if(i<sz-10)
		{
			n1 = pos[i+9];
			n2 = pos[i+10];

			n.first = (n1.first + n2.first)/2.0;
			n.second = (n1.second + n2.second)/2.0;
		}
		else
			n = pos[sz-1];
		try{
			tan_theta = (n.second - pos[i].second)/(n.first - pos[i].first);
		}
		catch(...){
			cout<<"in catch \n";
			Dy = (n.second - pos[i].second);
			if(Dy==0)
				tan_theta = 0;
			else if(Dy>0)
				tan_theta = 3.1416/2.0;
			else
				tan_theta = -3.1416/2.0;
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
		vel[i].second = v*tan_theta*cos_theta>0.1?v*tan_theta*cos_theta:0;

		if(i==0)
		{
			cout<<"pos[0] = "<<pos[0].first<<","<<pos[0].second<<" n = "<<n.first<<","<<n.second<<endl;
			cout<<"Dy = "<<Dy<<"tan_theta = "<<tan_theta<<endl;
		}
	}
	return vel;
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
			cout<<"in catch \n";
			double Dy = (y1 - 8*y2 + 8*y3 - y4);
			if(Dy==0)
				tan_theta = 0;
			else if(Dy>0)
				tan_theta = 3.1416/2.0;
			else
				tan_theta = -3.1416/2.0;
		}	
		// set velocity components
		try{
		v = min(max_vel_fn(pos,i), vi + (vf - vi) * i / sz);
		}
		catch(...){
			v = 0;

		}
		double cos_theta = 1/sqrt(1+tan_theta*tan_theta);
		vel[i].first = v*cos_theta>0.1?v*cos_theta:0;
		vel[i].second = v*tan_theta*cos_theta>0.1?v*tan_theta*cos_theta:0.1;
	}
	return vel;
}

void Callback_BS(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
	for(int i=0;i<6;i++){
		home_pos_theta[i] = msg->homePos[i].theta;
		homePos[i].x = msg->homePos[i].x;
		homePos[i].y = msg->homePos[i].y;
	}

}

void Callback(const krssg_ssl_msgs::planner_path::ConstPtr& msg)
{
	long long currTime;
	gettimeofday(&t,NULL);
  	currTime = (long long)(t.tv_sec)*1000 + (long long)(t.tv_usec)/1000;
	float currT = ros::Time::now().toSec();
	float diffT = currT - lastT;
	vector<Vector2D> pos;
	for(int i=0;i<msg->point_array.size();i++)
	{
		Vector2D p;
		p.first = msg->point_array[i].x;
		p.second = msg->point_array[i].y;
		pos.push_back(p);
	}
	int sz = pos.size();
	vector<Vector2D> vel = velocity_profile_naive(pos, 4*sz/400, 0);
	pidMessage.velX = vel[0].first;
	pidMessage.velY = vel[0].second;
	pidMessage.errorX = lastExpected.x - homePos[0].x;
	pidMessage.errorY = lastExpected.y - homePos[0].y;
	float motionAngle = atan2(vel[0].second, vel[0].first);
	cout<<"motionAngle = "<<motionAngle*180/3.1416<<endl;
	pub.publish(pidMessage);

// #############################################################
// #############################################################
// #############################################################

	// float theta =  (home_pos_theta[BOT_ID] - motionAngle)*(-1);
	// float speed = sqrt(vel[0].first*vel[0].first + vel[0].second*vel[0].second);
	// cout<<"#######################"<<endl;
	// for (int i = 0; i < 10; ++i)
	// {
	// 	cout<<pos[i].first<<" "<<pos[i].second<<endl;
	// }
	// cout<<"######################"<<endl;
	// cout<<"vel[0].first = "<<vel[0].first<<" vel[0].second = "<<vel[0].second<<endl;
	// // cout<<"got velocity, size = "<<vel.size()<<endl;
	// // cout<<"initial point = "<<pos[0].first<<","<<pos[0].second<<"\nEnd point = "<<pos[pos.size()-1].first<<","<<pos[pos.size()-1].second<<endl;
	
	// command_msgs.id          = 0;
	// command_msgs.wheelsspeed = 0;
	// command_msgs.veltangent  = speed*cos(theta);
	// command_msgs.velnormal   = speed*sin(theta);
	// command_msgs.velangular  = 0;
	// command_msgs.kickspeedx  = 0;
	// command_msgs.kickspeedz  = 0;
	// command_msgs.spinner     = false;

	// gettimeofday(&t,NULL);
 //  	currTime = (long long)(t.tv_sec)*1000 + (long long)(t.tv_usec)/1000;
	// cout<<"calculated expectedPos at  "<<currTime<<endl;
	// command_msgs.nextExpectedX = vel[BOT_ID].first*diffT*1000 + homePos[BOT_ID][0];
	// command_msgs.nextExpectedY = vel[BOT_ID].second*diffT*1000 + homePos[BOT_ID][1]; 
	
	// final_msgs.timestamp      = ros::Time::now().toSec();
	// cout<<std::setprecision(15)<<final_msgs.timestamp<<"################"<<endl;

	// // cout<<"\n\n##########TIME STAMP##################";
	// // cout<<diffT/CLOCKS_PER_SEC<<endl;
	// final_msgs.isteamyellow   = false;
	// final_msgs.robot_commands = command_msgs;
	// // cout<<"actual velocity = spped"<<speed<<", theta"<<motionAngle<<endl;
	// // cout<<" Sending velnormal = "<<command_msgs.velnormal<<" veltangent = "<<command_msgs.veltangent<<endl;
	// lastT = clock();
	// pub.publish(final_msgs);
	// oldPos = pos;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_profiling");
  	ros::NodeHandle n;
  	// ros::Time::init();
	lastT = ros::Time::now().toSec();
	ros::Subscriber sub = n.subscribe("/path_planner_ompl", 1000, Callback);
	// pub = n.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000);
	pub = n.advertise<krssg_ssl_msgs::pid_message>("/pid", 1000);
	ros::Subscriber sub1 = n.subscribe("/belief_state", 1000, Callback_BS);
	ros::spin();
	return 0;
}