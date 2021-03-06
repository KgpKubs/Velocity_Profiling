#include "ros/ros.h"
#include <bits/stdc++.h>
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>
using namespace std;

const int MAX_VEL = 50;
int BOT_ID = 0;

typedef pair<double,double> Vector2D;
typedef double (*max_vel)(const vector<Vector2D>& pos, int ind);
ros::Publisher pub;
krssg_ssl_msgs::gr_Commands final_msgs;
krssg_ssl_msgs::gr_Robot_Command command_msgs;
float home_pos_theta[6] = {0,0,0,0,0,0};
vector<Vector2D> pos, vel;
double t1=0, t2=0, t_expected=0, speed, theta, motionAngle; 
Vector2D p_expected;
int flag=0, so_far=0;
//on an average it goes 930 unit with v = 1, in 1m/s
#define DIST_UNIT 930
Vector2D home_pos[6];

double max_vel_naive(const vector<Vector2D>& pos, int ind){
	return MAX_VEL;
}


double dist_(Vector2D p1, Vector2D p2)
{
	double dx = p2.first - p1.first;
	double dy = p2.second - p1.second;

	return (sqrt(dx*dx+dy*dy));
}

double get_time(Vector2D p1, Vector2D p2)
{
	double d = dist_(p1, p2);
	cout<<"p1 = "<<p1.first<<","<<p1.second<<" P2 = "<<p2.first<<","<<p2.second<<endl;
	cout<<"dist = "<<d<<endl;

	if(speed==0)
		return 0;
	return (d/(DIST_UNIT*speed));
}

void send_stop()
{
	cout<<"sending stop\n";
	command_msgs.veltangent  = 0;
	command_msgs.velnormal   = 0;
	final_msgs.isteamyellow   = false;
	final_msgs.robot_commands = command_msgs;
	final_msgs.timestamp      = ros::Time::now().toSec();
	pub.publish(final_msgs);
	return;
}

vector<Vector2D> velocity_profile_naive2(const vector<Vector2D>& pos, const double vi, const double vf, max_vel max_vel_fn= max_vel_naive){
	int sz = pos.size();
	double Dy=0, Dx=0;
	vector<Vector2D> vel(sz);
	for(int i=0;i<sz;i++){
		double tan_theta;
		Vector2D n1, n2, n;
		double v;

		if(i<sz-40)
		{
			n1 = pos[i+39];
			n2 = pos[i+40];

			n.first = (n1.first + n2.first)/2.0;
			n.second = (n1.second + n2.second)/2.0;
		}
		else
		{
			// cout<<"in else\n";
			n = pos[sz-1];
		}
		
		Dy = (n.second - pos[i].second);
		Dx = (n.first - pos[i].first);
		tan_theta = atan2((n.second - pos[i].second),(n.first - pos[i].first));
		
		
		// set velocity components
		try{
		v = min(max_vel_fn(pos,i), vi + (vf - vi) * i / sz);
		}
		catch(...){
			v = 0;

		}

		double cos_theta = 1/sqrt(1+tan_theta*tan_theta);
		vel[i].first = v*cos_theta;//>0.1?v*cos_theta:0;
		vel[i].second = v*tan_theta*cos_theta;//>0.1?v*tan_theta*cos_theta:0;

		// if(i==0)
		// {
		// 	cout<<"pos[0] = "<<pos[0].first<<","<<pos[0].second<<" n = "<<n.first<<","<<n.second<<endl;
		// 	cout<<"Dy = "<<Dy<<"tan_theta = "<<tan_theta<<endl;
		// 	cout<<"Dx = "<<Dx<<endl;
		// 	cout<<"vel[0] = "<<vel[i].second<<","<<vel[i].first<<endl;
		// }
	}
	return vel;
}

void run()
{
	cout<<"in run\n";
	
	command_msgs.id          = 0;
	command_msgs.wheelsspeed = 0;
	command_msgs.veltangent  = speed*cos(theta);
	command_msgs.velnormal   = speed*sin(theta);
	command_msgs.velangular  = 0;
	command_msgs.kickspeedx  = 0;
	command_msgs.kickspeedz  = 0;
	command_msgs.spinner     = false;

	t2 = final_msgs.timestamp      = ros::Time::now().toSec();

	// cout<<"t2 = "<<t2<<endl;
	// cout<<"t1 = "<<t1<<endl;
	if(t_expected>0.5)
	{
		send_stop();
		return;
	}
	cout<<"msg entered, t2-t1 = "<<t2-t1<<" t_expected = "<<t_expected<<endl;
	cout<<"so_far = "<<so_far<<" pos.size() = "<<pos.size()<<endl;	
	if(so_far+5>pos.size())
	{
		cout<<"pos size achieved here. exiting\n";
		command_msgs.veltangent  = 0;
		command_msgs.velnormal   = 0;
		final_msgs.isteamyellow   = false;
		final_msgs.robot_commands = command_msgs;
		final_msgs.timestamp      = ros::Time::now().toSec();
		pub.publish(final_msgs);
		return;
	}
	if(t2-t1>=t_expected)
	{
		double d = dist_(p_expected, home_pos[0]);
		cout<<"Deviation  = "<<d<<endl;
		if(d>800)
		{
			send_stop();
			return;
		}
		t1 = t2;
		t1 = t2;
		p_expected = pos[so_far+5];
		p_expected.first = p_expected.first*160/13-4000;
		p_expected.second = p_expected.second*40/3-3000;
		t_expected = get_time(home_pos[0], p_expected);
		motionAngle = atan2(vel[so_far].second, vel[so_far].first);
		theta =  (home_pos_theta[BOT_ID] - motionAngle)*(-1);
		speed = sqrt(vel[so_far].first*vel[so_far].first + vel[so_far].second*vel[so_far].second);
		so_far += 5;
	}
	final_msgs.isteamyellow   = false;
	final_msgs.robot_commands = command_msgs;
	// cout<<"actual velocity = spped"<<speed<<", theta"<<motionAngle<<endl;
	// cout<<" Sending velnormal = "<<command_msgs.velnormal<<" veltangent = "<<command_msgs.veltangent<<endl;
	pub.publish(final_msgs);
}

void Callback_BS(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
	for(int i=0;i<6;i++)
	{
		home_pos_theta[i] = msg->homePos[i].theta;
		home_pos[i].first = msg->homePos[i].x;
		home_pos[i].second = msg->homePos[i].y;
	}

	run();
}

void Callback(const krssg_ssl_msgs::planner_path::ConstPtr& msg)
{
	if(flag==1)
	{
		return;
	}
	pos.clear();
	for(int i=0;i<msg->point_array.size();i++)
	{
		Vector2D p;
		p.first = msg->point_array[i].x;
		p.second = msg->point_array[i].y;
		pos.push_back(p);
	}
	int sz = pos.size();
	vel = velocity_profile_naive2(pos, 4*sz/400, 0);
	
	cout<<"updated velocity for entire path\n\n\n\n";
	
	
	p_expected = pos[so_far+5];
	p_expected.first = p_expected.first*160/13-4000;
	p_expected.second = p_expected.second*40/3-3000;
	t_expected = get_time(home_pos[0], p_expected);
	motionAngle = atan2(vel[so_far].second, vel[so_far].first);
	theta =  (home_pos_theta[BOT_ID] - motionAngle)*(-1);
	speed = sqrt(vel[so_far].first*vel[so_far].first + vel[so_far].second*vel[so_far].second);
	so_far+=5;
	flag=1;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_profiling");

  	ros::NodeHandle n;
  	t1 = t2 = ros::Time::now().toSec();
	ros::Subscriber sub = n.subscribe("/path_planner_ompl", 1000, Callback);
	pub = n.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000);
	ros::Subscriber sub1 = n.subscribe("/belief_state", 1000, Callback_BS);
	ros::spin();
	return 0;
}