/*
	Requires Distance setpoint & Speed setpoint
*/

#include"ros/ros.h"
#include"std_msgs/Int8.h"
#include"std_msgs/Int32.h"
#include"std_msgs/Int64.h"
#include"std_msgs/Header.h"
#include"std_msgs/Float64.h"
#include"geometry_msgs/Twist.h"
#include"geometry_msgs/Vector3.h"
#include<cmath>
#include<fstream>

using namespace std;
ros::Timer t_1,t_2;

geometry_msgs::Vector3 pos;
geometry_msgs::Twist heading;
std_msgs::Float64 dist;

ros::Publisher heading_set_p;
ros::Publisher Distance;

const float LENC_PPR=500.0;
const float RENC_PPR=500.0;

const float MAX_DIST=50.0;
const float MIN_DIST=0.08;

const float WHEEL_D=0.20;	//Diameter in m
const float WHEEL_2_WHEEL=0.4168;	//Length in m
const float WHEEL_2_WHEEL_INV=2.399232246;	//Length in m
const float _PI_=3.14159265359;	//PI
const float DEG=180.0/_PI_;	//DEG
const float RAD=_PI_/180.0;	//RAD

int enc_l;
double tima_l,timb_l,dtim_l;
int al=0;
float current_s_l;
int start_encl,end_encl,L_d;
int lenc,plenc;
float l_RPM;

int enc_r;
double tima_r,timb_r,dtim_r;
int ar=0;
float current_s_r;
int start_encr,end_encr,R_d;
int renc,prenc;
float r_RPM;
int LAP,oe=1;

int stop=1,start1=1,start2=1,PAUSE=1;
float total_dist,right_dist,left_dist;
float TGT=0.05,LAST_DIST=0.0,INIT_POS=TGT;


float SET_DIST=0.00,LEFT_SET_H=0.0,RIGHT_SET_H=0.0;


float to_MPS(float rpm)
{
	return ((rpm/60.0)*(WHEEL_D*_PI_));
}

void setdCB(const std_msgs::Float64::ConstPtr& set)
{
	float setd;
	setd=set->data;

	if(setd>=MIN_DIST && setd<=MAX_DIST)
	{
		SET_DIST=setd;	// per sec
		stop=0;
		al=0;
		ar=0;
		start1=0;
		start2=0;
		ros::param::get("/LEFT_SET_H",LEFT_SET_H);
		ros::param::get("/RIGHT_SET_H",RIGHT_SET_H);
		heading.linear.x = LEFT_SET_H;
		heading.angular.z = RIGHT_SET_H;
		heading_set_p.publish(heading);
		if(pos.y<=0.5)
		{
			oe=1;
			LAST_DIST=pos.y;
		}
//		ROS_INFO("oe:%d, Y:%f",oe,pos.y);
	}
	else
	{
		SET_DIST=0.0;
		stop=1;
		al=1;
		ar=1;
		start1=0;
		start2=0;
	}
}

void t1CB(const ros::TimerEvent& )
{
	if(stop==0 && PAUSE==1)
	{
		if(oe==1)
		{
			total_dist=pos.y-LAST_DIST;
		}
		else if (oe==2)
		{
			total_dist=LAST_DIST-pos.y;
		}

		if((SET_DIST-total_dist)<=TGT)
		{
			heading.linear.x = 0.0;
			heading.angular.z = 0.0;
			heading_set_p.publish(heading);
			LAST_DIST=pos.y;
			if(oe==2)
			{
				oe=1;
			}
			else if(oe==1)
			{
				oe=2;
			}
			stop=1;
		}

		dist.data = total_dist;
		Distance.publish(dist);

		ROS_INFO("Y:%f, LAST_D:%f, total_dist:%f",pos.y,LAST_DIST,total_dist);
	}
}

void getenclCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_l = msg->data;
	if(al<1)
	{
		plenc=enc_l;
		al++;
	}
	if(start1<2 && (enc_l>0 || enc_l<0))
	{
		start_encl=enc_l;
		start1++;
	}
}

void getencrCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_r = msg->data;
	if(ar<1)
	{
		prenc=enc_r;
		ar++;
	}
	if(start2<2 && (enc_r>0 || enc_r<0))
	{
		start_encr=enc_r;
		start2++;
	}
}

void getpauseCB(const std_msgs::Int32::ConstPtr& d)
{
	PAUSE=d->data;	

	if(stop==0 && PAUSE==0)
	{
		heading.linear.x = 0.0;
		heading.angular.z = 0.0;
		heading_set_p.publish(heading);
	}

	if(stop==0 && PAUSE==1)
	{
		heading.linear.x = LEFT_SET_H;
		heading.angular.z = RIGHT_SET_H;
		heading_set_p.publish(heading);
	}
}

void posCB(const geometry_msgs::Vector3::ConstPtr& P)
{
	pos.x=P->x;
	pos.y=P->y;
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"Distance_control");
	ros::NodeHandle n;


	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",50,&getenclCB);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",50,&getencrCB);
	ros::Subscriber get_pos=n.subscribe<geometry_msgs::Vector3>("/pos",10,&posCB);

	ros::Subscriber get_dist=n.subscribe<std_msgs::Float64>("/SET_DIST",10,&setdCB);
	ros::Subscriber get_Pause=n.subscribe<std_msgs::Int32>("/PAUSE",10,&getpauseCB);

	heading_set_p=n.advertise<geometry_msgs::Twist>("/SET_HEADING",10); //Heading control setpoint

	Distance = n.advertise<std_msgs::Float64>("/DISTANCE", 10);

	t_1=n.createTimer(ros::Duration (0.01),t1CB);

	ros::Rate r(20);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	ros::spin();
	if(ros::isShuttingDown())
	{
		t_1.stop();
	}
	return 0;
}

