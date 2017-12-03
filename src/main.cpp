#include <bits/stdc++.h>
#include "functions.h"
using namespace std;

const int MAX_VEL = 50, BOT_ID = 0;
ros::Publisher pub, pub_to_gui;
krssg_ssl_msgs::planner_path path_points_pub;
krssg_ssl_msgs::point_2d point_;
krssg_ssl_msgs::gr_Commands final_msgs;
krssg_ssl_msgs::gr_Robot_Command command_msgs;

void send_stop()
{
	cout<<"sending stop\n";
	krssg_ssl_msgs::pid_message msg;
	msg.velX = 0.0;
	msg.velY = 0.0;
	msg.errorX = 0;
	msg.errorY = 0;
	msg.id = BOT_ID;
	pub.publish(msg);
	return ;
}

void send_vel(double speed, double motion_angle, int index)
{
	krssg_ssl_msgs::pid_message msg;
	speed/=1000 ;
	cout<<"motion_angle = "<<motion_angle<<endl;
	double bot_angle = home_pos_theta[BOT_ID];
	double vX = speed*cos(motion_angle);
	double vY = speed*sin(motion_angle);
	msg.velX = vX;
	msg.velY = vY;
	// msg.velX = 0.0;
	// msg.velY = 0.5;
	msg.errorX = path_points[index].x - home_pos[BOT_ID].x;
	msg.errorY = path_points[index].y - home_pos[BOT_ID].y;
	msg.id = BOT_ID;
	msg.botAngle = bot_angle;

	pub.publish(msg);
	// double vel_tangent = speed*cos(bot_angle - motion_angle);
	// double vel_normal = -speed*sin(bot_angle - motion_angle);
// 
	// command_msgs.veltangent  = vel_tangent;
	// command_msgs.velnormal   = vel_normal;
	// final_msgs.isteamyellow   = false;
	// final_msgs.robot_commands = command_msgs;
	// final_msgs.timestamp      = ros::Time::now().toSec();
	// pub.publish(final_msgs);
	return;
}

void Callback_BS(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
	if(PATH_RECEIVED==false)
	{
		cout<<"Path PATH_RECEIVED = False\n";
		return;
	}

	// path_points_pub.point_array.clear();
	// for(int i=0;i<path_points.size();i++)
	// {
	// 	  point_.x=path_points[i].x;
	//       point_.y=path_points[i].y;
	//       path_points_pub.point_array.push_back(point_);
	// }
 //    pub_to_gui.publish(path_points_pub);

	for(int i=0;i<6;i++)
	{
		home_pos_theta[i] = msg->homePos[i].theta;
		home_pos[i].x = msg->homePos[i].x;
		home_pos[i].y = msg->homePos[i].y;
	}
	curr_time = ros::Time::now().toSec();
	double t = curr_time - start_time;

	if(t>ExpectedTraverseTime)
	{
		cout<<"OUT OF TIME!! ExpectedTraverseTime = "<<ExpectedTraverseTime<<"\n";
		send_stop();
		return;
	}

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
		send_vel(out_speed, vel_angle[index],index);
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
	if(flag==1) return;
	flag=1;
	cout<<"in path_planner Callback function. \n";
	PATH_RECEIVED = true;
	path_points.clear();
	vel_angle.clear();
	int size_ = msg->point_array.size();

	// for (int i = 0; i < size_; ++i)
	// {
		// cout<<"("<<msg->point_array[i].x<<","<<msg->point_array[i].y<<"),"<<endl;
	// }
	cout<<endl;
	for(int i=0;i<size_;i++)
	{
		point p;
		p.x = msg->point_array[i].x;
		p.y = msg->point_array[i].y;
		path_points.push_back(p);
		// cout<<"("<<p.x<<","<<p.y<<"),";
		if(i<size_-1 && i>0)
		{
			double dx = msg->point_array[i+1].x - msg->point_array[i-1].x;
			double dy = msg->point_array[i+1].y - msg->point_array[i-1].y;
			vel_angle.push_back(atan2(dy, dx));
			// cout<<"1 "<<dy/dx;
		}
		else if(i==0)
		{
			double dx = msg->point_array[i+1].x - msg->point_array[i].x;
			double dy = msg->point_array[i+1].y - msg->point_array[i].y;
			vel_angle.push_back(atan2(dy, dx));
			// cout<<"2 "<<dy/dx;
			// cout<<"vel_angle[0] = "<<vel_angle[0]<<endl;
		}
		else
		{
			double dx = msg->point_array[i].x - msg->point_array[i-1].x;
			double dy = msg->point_array[i].y - msg->point_array[i-1].y;
			vel_angle.push_back(atan2(dy, dx));
			// cout<<"3 "<<dy/dx;
		}
		// cout<<" "<<vel_angle[vel_angle.size() - 1]<<endl;
	}

	// cout<<endl;
	start_point.x = msg->point_array[0].x ;
	start_point.y = msg->point_array[0].y;
	goal_point.x = msg->point_array[size_-1].x;
	goal_point.y = msg->point_array[size_-1].y;

	path_length = GetPathLength();
	cout<<"----------------"<<path_length<<endl;
	ExpectedTraverseTime = getTime(path_length, path_length, MAX_SPEED, MAX_ACC, START_SPEED, FINAL_SPEED);
	cout<<"ExpectedTraverseTime = "<<ExpectedTraverseTime<<"\n";
	
	cout<<endl;
	start_time = ros::Time::now().toSec();
	curr_time =  ros::Time::now().toSec();
	// cout<<"setting start_time = "<<start_time<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_profiling");

  	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/path_planner_ompl", 1000, Callback);
	pub = n.advertise<krssg_ssl_msgs::pid_message>("/pid", 1000);
	pub_to_gui = n.advertise<krssg_ssl_msgs::planner_path>("/vel_profile_to_gui", 1000);
	ros::Subscriber sub1 = n.subscribe("/belief_state", 1000, Callback_BS);
	ros::spin();
	return 0;
}