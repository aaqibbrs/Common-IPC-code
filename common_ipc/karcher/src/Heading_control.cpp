/*
	updated logic
	Encoder + IMU
	Requires RPM setpoint
	
	Using current orientation as 0 heading and accordingly calculating error and corresponding PID control output for that error
*/

#include"ros/ros.h"
#include"std_msgs/Int8.h"
#include"std_msgs/Int32.h"
#include"std_msgs/Int64.h"
#include"std_msgs/Header.h"
#include"std_msgs/Float64.h"
#include"geometry_msgs/Twist.h"
#include"geometry_msgs/Vector3.h"

using namespace std;
ros::Timer t_1,t_2;

geometry_msgs::Twist setpoint;
geometry_msgs::Vector3 G,pos;
std_msgs::Float64 H_SET,Heading,angle,angle1,angle2;

ros::Publisher HEADING,H_SET_S,Angle,Angle1,Angle2;

ros::Publisher set_p;

const float LENC_PPR=500.0;
const float RENC_PPR=500.0;

const float WHEEL_D=0.20;	//Diameter in m
const float WHEEL_2_WHEEL=0.4168;	//Length in m
const float WHEEL_2_WHEEL_INV=2.399232246;	//Length in m
const float _PI_=3.14159265359;	//PI
const float DEG=180.0/_PI_;	//DEG

const float MAX_SPEED=0.45;
const float MIN_SPEED=0.03;
const float ANG_SPEED=0.50;
const float E_TOL = 0.0;

const int samp=10;

int setf=2,set_setp=0,SET_S=1;
int stopH=1,Ap=0;
int AF=1,TT=0,stp=1,change_offset=1,set180=1;

float speed_l,def_speed_l;
float speed_r,def_speed_r;
float left_w,right_w;

float S,A,A_,B,H;
int i=0,j=0;
int start1=5,start2=5;

//const float P=0.0092,I=0.0051,D=0.0042,dt=0.05;
//const float P=0.016,I=0.004,D=0.0012,dt=0.05;
//const float P=0.055,I=0.025,D=0.08,dt=0.05;
//const float P=0.042,I=0.028,D=0.08,dt=0.05;
const float P=0.0252,I=0.008,D=0.001,dt=0.025;


float kp_H=P,ki_H=I,kd_H=D,dt_H=dt;
float Error_H,S_error_H,P_error_H,D_error_H,Output_H,current_H;

int enc_r,start_encr,end_encr,R_d,prenc,renc;
float left_dist,left_dist_,l_dist;

int enc_l,start_encl,end_encl,L_d,plenc,lenc;
float right_dist,right_dist_,r_dist;

float LAST_H;
float total_dist,theta;



void up_pidCB(const geometry_msgs::Vector3::ConstPtr& gain)
{
	kp_H=gain->x;
	kd_H=gain->y;
	ki_H=gain->z;
//	we=0;
}



void set_headingCB(const geometry_msgs::Twist::ConstPtr& set)
{
	float setl,setr;
	setl=set->linear.x;
	setr=set->angular.z;
	
	if(setl>=MIN_SPEED && setl<=MAX_SPEED)
	{
		if(set_setp==0)
		{
			std::cout<<"START:- G.x: "<<G.x<<" left_dist: "<<left_dist<<" right_dist: "<<right_dist<<"\n"<<"\n";
			set_setp=1;
			set180=0;
			left_dist=0.0;
			right_dist=0.0;
		}
		if(start1>0)
		{
			start1=0;
		}
		if(start2>0)
		{
			start2=0;
		}
		
		setf=0;
		Ap=1;
		speed_l=setl;
		def_speed_l=setl;
		H=0.0;
	}
	else
	{
		speed_l=0.0;
		def_speed_l=0.0;
	}
	
	if(setl<MIN_SPEED)
	{
		std::cout<<"STOP:- G.x: "<<G.x<<" left_dist: "<<left_dist<<" right_dist: "<<right_dist<<"\n";
		stopH=1;
		setf=2;
		SET_S=1;
		set_setp=0;
		change_offset=0;
		j=0;
		AF=1;
		LAST_H=G.x;

		start1=0;
		start2=0;

			setpoint.linear.x=0.0;
			setpoint.angular.z=0.0;
			set_p.publish(setpoint);
	}

	if(right_w>MIN_SPEED && left_w<=MIN_SPEED && change_offset==0  && set180==0)
	{
		if(G.x>LAST_H)
		{
			S=180.0+S;

			if(S>359.99)
			{
				S=S-359.99;
			}
		}
		else
		{
			if(S>180.0)
			{
				S=S-180.0;
			}
			else
			{
				S=180.0-S;
			}
		}
		
		left_dist_=(WHEEL_2_WHEEL*_PI_);
		
		int r_d=(enc_r-start_encr);
		if(r_d<0)
		{
			r_d=-r_d;
		}
		right_dist_=float(r_d)*((WHEEL_D*_PI_)/RENC_PPR);

		change_offset=1;
		set180=1;
		std::cout<<"G.x: "<<G.x<<" S: "<<S<<" left_dist_: "<<left_dist_<<" right_dist_: "<<right_dist_<<"\n";
	}
	else if(left_w>MIN_SPEED && right_w<=MIN_SPEED && change_offset==0 && set180==0)
	{
		if(G.x>LAST_H)
		{
			S=180.0+S;

			if(S>359.99)
			{
				S=S-359.99;
			}
		}
		else
		{
			if(S>180.0)
			{
				S=S-180.0;
			}
			else
			{
				S=180.0-S;
			}
		}
		
		right_dist_=(WHEEL_2_WHEEL*_PI_);

		int l_d=(enc_l-start_encl);
		if(l_d<0)
		{
			l_d=-l_d;
		}

		left_dist_=float(l_d)*((WHEEL_D*_PI_)/LENC_PPR);

		change_offset=1;
		set180=1;

		std::cout<<"G.x: "<<G.x<<" S: "<<S<<" left_dist_: "<<left_dist_<<" right_dist_: "<<right_dist_<<"\n";
	}
}

//void headingCB(const geometry_msgs::Vector3::ConstPtr& Q)
void headingCB(const ros::TimerEvent& )

{
	float q;
	
//	G.x=Q->x;
	if(setf==0)
	{
		if(i<samp)
		{
			S=G.x;
			i++;
		}
		else
		{
			i=samp;	
			angle.data=S;
//			Angle.publish(angle);

			if(j<samp)
			{
				j++;
			}
			if(j>=samp)
			{
				setf=1;
				A=0.0;
				A_=0.0;
				AF=0;
				SET_S=0;
				j=0;
			}
		}
	}
	else if(setf==1)
	{
		if(pos.x==0.0)
		{
//			S=G.x;
		}
		
		if(G.x>=S)
		{
			q = G.x-S;
		}
		else
		{
			q=G.x-S+360.0;
		}
		
		if(SET_S==0)
		{
			if(q>179.99)
			{
				A_=q-360.0;
			}
			else
			{
				A_=q;
			}
		}
	}

//---------------------------------------------------------------------------

	if(start1>=2 && start1<3 && start2>=2 && start2<3 && setf==1)
	{
		end_encl=enc_l;
		end_encr=enc_r;
		L_d=(end_encl-start_encl);
		R_d=(end_encr-start_encr);
		if(L_d<0)
		{
			L_d=-L_d;
		}
		if(R_d<0)
		{
			R_d=-R_d;
		}

		left_dist=float(L_d)*((WHEEL_D*_PI_)/LENC_PPR)+left_dist_;
		right_dist=float(R_d)*((WHEEL_D*_PI_)/RENC_PPR)+right_dist_;
	}

	float theta_=(left_dist-right_dist);

	theta_*=WHEEL_2_WHEEL_INV;
	theta_*=DEG;
	
	if(theta_<=-180.0)
	{
		theta=theta_+360.0;
	}
	else if(theta_>179.9)
	{
		theta=theta_-360.0;
	}
	else
	{
		theta=theta_;
	}
	
	angle1.data=A_;
//	Angle1.publish(angle1);
	angle2.data=theta;
//	Angle2.publish(angle2);

//	A = (theta + A_);
//	A *= 0.5;
	A=theta;
//--------------------------------------------------------------------------------	
	if(setf==1 && AF==0)
	{
		stopH=0;
		AF=1;
	}
}

void t1CB(const ros::TimerEvent& )
{
	if(stopH==1)
	{
		Error_H=0.0;
		S_error_H=0.0;
		P_error_H=0.0;
	}
	
	if(stopH==0)
	{
		current_H = A;
		Error_H = current_H-H;
		
		if((Error_H==0.0 && current_H==0.0) || (Error_H>-E_TOL || Error_H<E_TOL))
		{
			S_error_H=0.0;
		}

		if(Error_H<-E_TOL || Error_H>E_TOL)
		{
			S_error_H = S_error_H + Error_H;
			D_error_H = Error_H - P_error_H;
			P_error_H = Error_H;

			Output_H = (kp_H * Error_H) + (ki_H * S_error_H) + (kd_H * D_error_H);

			speed_r=Output_H;

			if(speed_r>ANG_SPEED)
			{
				speed_r=ANG_SPEED;
			}
			if(speed_r<-ANG_SPEED)
			{
				speed_r=-ANG_SPEED;
			}
		}
		else
		{
//			speed_r=0.0;
		}

		if((Error_H <= E_TOL && Error_H >= -E_TOL) && stopH==0)
		{
			setpoint.linear.x=def_speed_l;
			setpoint.angular.z=0.0;
			set_p.publish(setpoint);
//			std::cout<<"If l";
		}
		else if((Error_H > E_TOL || Error_H < -E_TOL) && stopH==0)
		{
			setpoint.linear.x=def_speed_l;
			setpoint.angular.z=speed_r;
			set_p.publish(setpoint);
//			std::cout<<"If r";
		}
		else
		{
//			setpoint.linear.x=0.0;
//			setpoint.angular.z=0.0;
//			set_p.publish(setpoint);
			stopH=1;
			std::cout<<"Else";
		}


		Heading.data = current_H;
		HEADING.publish(Heading);
		H_SET.data=0.0;
		H_SET_S.publish(H_SET);

//		ROS_INFO("P:%f, I:%f, D:%f",kp_H,ki_H,kd_H);
		ROS_INFO("X:%f, Y:%f, theta:%f, H:%f, L:%f, R: %f",pos.x,pos.y,theta,H,setpoint.linear.x,setpoint.angular.z);
//		ROS_INFO("G.x:%f, S:%f, A_:%f, theta:%f, A:%f, left_dist:%f, right_dist:%f",G.x,S,A_,theta,A,left_dist,right_dist);
	}
}

void turnsCB(const std_msgs::Int32::ConstPtr& t)
{
	TT+=1;
}

void turnupCB(const geometry_msgs::Twist::ConstPtr& t)
{
	left_w=t->linear.x;
	right_w=t->angular.z;
}

void posCB(const geometry_msgs::Vector3::ConstPtr& P)
{
	pos.x=P->x;
	pos.y=P->y;
}

void head_upCB(const std_msgs::Float64::ConstPtr& P)
{
	H=P->data;
}

void stopCB(const std_msgs::Int32::ConstPtr& t)
{
	stp=t->data;
}

void getenclCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_l = msg->data;
	if(start1<2 && enc_l!=0)
	{
		start_encl=enc_l;
		start1++;
	}
}

void getencrCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_r = msg->data;
	if(start2<2 && enc_r!=0)
	{
		start_encr=enc_r;
		start2++;
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"Heading_control");
	ros::NodeHandle n;

//	ros::Subscriber get_pid=n.subscribe<geometry_msgs::Vector3>("/pid1",10,&up_pidCB);

	ros::Subscriber get_pos=n.subscribe<geometry_msgs::Vector3>("/pos",10,&posCB);
	ros::Subscriber update_heading=n.subscribe<std_msgs::Float64>("/head_up",10,&head_upCB);
//	ros::Subscriber get_heading=n.subscribe<geometry_msgs::Vector3>("/brick_imu",50,&headingCB);
	ros::Subscriber heading_set_p=n.subscribe<geometry_msgs::Twist>("/SET_HEADING",10,&set_headingCB); //Heading control setpoint
	ros::Subscriber turns = n.subscribe<std_msgs::Int32>("/TURNS", 10,&turnsCB);
	ros::Subscriber STP = n.subscribe<std_msgs::Int32>("/STOP", 10,&stopCB);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",50,&getenclCB);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",50,&getencrCB);
	ros::Subscriber heading_set_t = n.subscribe<geometry_msgs::Twist>("/TURN_HEADING", 10,&turnupCB);

	set_p = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	HEADING = n.advertise<std_msgs::Float64>("/heading", 10);
	H_SET_S = n.advertise<std_msgs::Float64>("/H_SET_S", 10);
	Angle = n.advertise<std_msgs::Float64>("/A", 10);
	Angle1 = n.advertise<std_msgs::Float64>("/A1", 10);
	Angle2 = n.advertise<std_msgs::Float64>("/A2", 10);
	t_1=n.createTimer(ros::Duration (dt_H),t1CB);
	t_2=n.createTimer(ros::Duration (0.04),headingCB);

	ros::Rate r(20);
	while(ros::ok())
	{

		if(stp==0)
		{
			i=0;
			j=0;
			TT=0;
			stp=1;
			right_dist_=0.0;
			left_dist_=0.0;
		}
	

		ros::spinOnce();
		r.sleep();
	}
	ros::spin();
	if(ros::isShuttingDown())
	{
		t_1.stop();
		t_2.stop();
	}
	return 0;
}

