#include <bits/stdc++.h>
#include "functions.h"
using namespace std;

const int MAX_VEL = 50, BOT_ID = 0;
ros::Publisher pub;
krssg_ssl_msgs::gr_Commands final_msgs;
krssg_ssl_msgs::gr_Robot_Command command_msgs;

void send_stop()
{
	cout<<"sending stop\n";
	command_msgs.veltangent  = 0;
	command_msgs.velnormal   = 0;
	final_msgs.isteamyellow   = false;
	final_msgs.robot_commands = command_msgs;
	final_msgs.timestamp      = ros::Time::now().toSec();
	pub.publish(final_msgs);
	return ;
}

void send_vel(double speed, double motion_angle)
{
	cout<<"motion_angle = "<<motion_angle<<endl;
	double bot_angle = home_pos_theta[BOT_ID];
	double vel_tangent = speed*cos(bot_angle - motion_angle);
	double vel_normal = speed*sin(bot_angle - motion_angle);

	command_msgs.veltangent  = vel_tangent;
	command_msgs.velnormal   = vel_normal;
	final_msgs.isteamyellow   = false;
	final_msgs.robot_commands = command_msgs;
	final_msgs.timestamp      = ros::Time::now().toSec();
	pub.publish(final_msgs);
	cout<<"Publishing vel "<<speed<<","<<motion_angle*180.0/PI<<"\n";
	cout<<"Path_point[0] = "<<path_points[0].x<<","<<path_points[0].y<<endl;
	cout<<"Path_point[10] = "<<path_points[10].x<<","<<path_points[10].y<<endl;
	cout<<"vel_angle[0] = "<<vel_angle[0]<<endl;
	return;
}

void Callback_BS(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
	if(PATH_RECEIVED==false)
	{
		cout<<"Path PATH_RECEIVED = False\n";
		return;
	}

	for(int i=0;i<6;i++)
	{
		home_pos_theta[i] = msg->homePos[i].theta;
		home_pos[i].x = msg->homePos[i].x;
		home_pos[i].y = msg->homePos[i].y;
	}
	curr_time = ros::Time::now().toSec();
	double t = curr_time - start_time;
	// cout<<"start_time = "<<start_time<<endl;
	// cout<<"Calling Trapezoid!! t = "<<t<<endl;
	if(trapezoid(t, distance_traversed, out_speed))
	{
		int index = GetExpectedPositionIndex(distance_traversed);
		if(index == -1)
		{
			cout<<"Deviated!!\n\n";
			send_stop();
			return;
		}
		send_vel(out_speed, PI/2.0);
		// cout<<"Error = "<<dist(path_points[index], home_pos[BOT_ID])<<endl;
		// cout<<"Travelled "<<distance_traversed/path_length*100<<"percent \n";
		// cout<<"Going with vel = "<<out_speed<<" at "<<vel_angle[index]*180/(PI)<<" degree\n";
	}
	else
	{
		cout<<"Motion Not Possible!! \n\n";
		return;
	}
}

int flag = 0;
void Callback(const krssg_ssl_msgs::planner_path::ConstPtr& msg)
{
	// if(flag==1) return;
	// flag=1;
	// cout<<"in path_planner Callback function. \n";
	PATH_RECEIVED = true;
	path_points.clear();
	vel_angle.clear();
	int size_ = msg->point_array.size();
	// cout<<"size_ of vertex = "<<size_<<endl;
	for(int i=0;i<size_;i++)
	{
		// cout<<"i  = "<<i<<endl;
		point p;
		p.x = msg->point_array[i].x;
		p.y = msg->point_array[i].y;
		path_points.push_back(p);

		if(i<size_-4 && i>1)
		{
			double dx = path_points[i+2].x - path_points[i-2].x;
			double dy = path_points[i+2].y - path_points[i-2].y;

			vel_angle.push_back(atan2(dy, dx));
		}
		else if(i==0)
		{
			double dx = path_points[i+10].x - path_points[i].x;
			double dy = path_points[i+10].y - path_points[i].y;
			vel_angle.push_back(atan2(dy, dx));
			cout<<"vel_angle[0] = "<<vel_angle[0]<<endl;
		}
		else
		{
			double dx = path_points[i].x - path_points[i-1].x;
			double dy = path_points[i].y - path_points[i-1].y;
			vel_angle.push_back(atan2(dy, dx));
		}
	}

	start_point.x = msg->point_array[0].x;
	start_point.y = msg->point_array[0].y;
	goal_point.x = msg->point_array[size_-1].x;
	goal_point.y = msg->point_array[size_-1].y;

	path_length = GetPathLength();

	ExpectedTraverseTime = getTime(0, path_length, MAX_SPEED, MAX_ACC, START_SPEED, FINAL_SPEED);

	start_time = ros::Time::now().toSec();
	curr_time =  ros::Time::now().toSec();
	// cout<<"setting start_time = "<<start_time<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_profiling");

  	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/path_planner_ompl", 1000, Callback);
	pub = n.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000);
	ros::Subscriber sub1 = n.subscribe("/belief_state", 1000, Callback_BS);
	ros::spin();
	return 0;
}