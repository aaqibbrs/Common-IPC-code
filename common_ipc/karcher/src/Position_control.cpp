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
std_msgs::Float64 H_SET,Heading,angle,angle1,angle2,H_UP;

ros::Publisher HEADING,H_SET_S,Angle,Angle1,Angle2,update_heading;

ros::Publisher set_p;

const float LENC_PPR=500.0;
const float RENC_PPR=500.0;

const float WHEEL_D=0.20;	//Diameter in m
const float WHEEL_2_WHEEL=0.4168;	//Length in m
const float WHEEL_2_WHEEL_INV=2.399232246;	//Length in m
const float _PI_=3.14159265359;	//PI
const float DEG=180.0/_PI_;	//DEG

const float MAX_SPEED=0.30;
const float MIN_SPEED=0.03;
const float ANG_SPEED=1.50;
const float E_TOL = 0.001;

const int samp=10;

int setf=2,set_setp=0,SET_S=1;
int stopH=1,Ap=0;
int AF=1,TT=0,stp=1,change_offset=1,set180=1;

float speed_l,def_speed_l;
float speed_r,def_speed_r;
float left_w,right_w;

float S,A,A_,B;
int i=0,j=0;
int start1=5,start2=5;

//const float P=1.8,I=0.28,D=0.002,dt=0.05;
//const float P=0.50,I=0.150,D=0.002,dt=0.05;
//const float P=0.12,I=0.038,D=0.001,dt=0.05;
//const float P=1.42,I=0.825,D=0.25,dt=0.05;
const float P=1.8,I=0.92,D=0.5,dt=0.0125;

float kp_H=P,ki_H=I,kd_H=D,dt_H=dt;
float Error_H,S_error_H,P_error_H,D_error_H,Output_H,current_H;

int enc_r,start_encr,end_encr,R_d,prenc,renc;
float left_dist,left_dist_,l_dist;

int enc_l,start_encl,end_encl,L_d,plenc,lenc;
float right_dist,right_dist_,r_dist;

float LAST_H;
float total_dist,theta,T_TYPE;
float p_,op=P,i_,oi=I,d_,od=D;
int LAP=0;


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
	ros::param::get("/TURN_TYPE",T_TYPE);

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
		if(T_TYPE==1)
		{
			if(pos.x<=S)
			{
				S-=WHEEL_2_WHEEL;
			}
			else
			{
				S+=WHEEL_2_WHEEL;
			}
			LAP=1;
		}
		else if(T_TYPE==2)
		{
			if(pos.x>=S)
			{
				S+=WHEEL_2_WHEEL;
			}
			else
			{
				S-=WHEEL_2_WHEEL;
			}
			LAP=0;
		}

		change_offset=1;
		set180=1;
		std::cout<<"G.x: "<<G.x<<" S: "<<S<<" left_dist_: "<<left_dist_<<" right_dist_: "<<right_dist_<<"\n";
	}
	else if(left_w>MIN_SPEED && right_w<=MIN_SPEED && change_offset==0 && set180==0)
	{
		if(T_TYPE==1)
		{
			if(pos.x<=S)
			{
				S-=WHEEL_2_WHEEL;
			}
			else
			{
				S+=WHEEL_2_WHEEL;
			}
			LAP=0;
		}
		else if(T_TYPE==2)
		{
			if(pos.x>=S)
			{
				S+=WHEEL_2_WHEEL;
			}
			else
			{
				S-=WHEEL_2_WHEEL;
			}
			LAP=1;
		}

		change_offset=1;
		set180=1;

		std::cout<<"G.x: "<<G.x<<" S: "<<S<<" left_dist_: "<<left_dist_<<" right_dist_: "<<right_dist_<<"\n";
	}
}

void pos_up(const ros::TimerEvent& )
{
	float q;

	if(setf==0)
	{
		if(i<samp)
		{
			S=pos.x;
			i++;
		}
		else
		{
			i=samp;
			angle.data=S;
			Angle.publish(angle);
		

			if(j<samp)
			{
				j++;
			}
			if(j>=samp)
			{
				setf=1;
				A=0.0;
				AF=0;
				SET_S=0;
				j=0;
			}
		}
	}
	if(setf==1)	//	**
	{
		if(LAP==0)
		{
			A=pos.x-S;
			angle1.data=A;
		}
		else if(LAP==1)
		{
			A=-pos.x+S;
			angle1.data=-A;
		}
//		A=pos.x;

		if(T_TYPE==2)
		{
			Angle1.publish(angle1);
		}
		else if(T_TYPE==1)
		{
			angle1.data=-A;
			Angle1.publish(angle1);
		}

	}

	if(setf==1 && AF==0)
	{
		stopH=0;
		AF=1;
	}
}

void t1CB(const ros::TimerEvent& )
{
	ros::param::get("/P",p_);
	ros::param::get("/I",i_);
	ros::param::get("/D",d_);
	if(p_!=op && p_!=0.0)
	{
		kp_H=p_;
		op=p_;
		std::cout<<"P: "<<p_<<"\n";
	}
	if(i_!=oi && i_!=0.0)
	{
		ki_H=i_;
		oi=i_;
		std::cout<<"I: "<<i_<<"\n";
	}
	if(d_!=od && d_!=0.0)
	{
		kd_H=d_;
		od=d_;
		std::cout<<"D: "<<d_<<"\n";
	}
	if(stopH==1)
	{
		Error_H=0.0;
		S_error_H=0.0;
		P_error_H=0.0;
	}

	if(stopH==0)
	{
		current_H = A;
//		Error_H = S-current_H;
		Error_H = 0.0-current_H;

		if(Error_H>-E_TOL && Error_H<E_TOL)
		{
			S_error_H=0.0;
			D_error_H = 0.0;
			P_error_H = 0.0;
//			Output_H = 0.0;
//			speed_r=0.0;
		}

		else if(Error_H<=-E_TOL || Error_H>=E_TOL)
		{
			S_error_H = S_error_H + Error_H;
			D_error_H = Error_H - P_error_H;
			P_error_H = Error_H;

			Output_H = (kp_H * Error_H) + (ki_H * S_error_H) + (kd_H * D_error_H);

			speed_r=Output_H;

			if(speed_r>ANG_SPEED)
			{
//				speed_r=ANG_SPEED;
			}
			if(speed_r<-ANG_SPEED)
			{
//				speed_r=-ANG_SPEED;
			}


		}
		else
		{
//			speed_r=0.0;
		}
		H_UP.data=speed_r;
		update_heading.publish(H_UP);


/*
		if((Error_H <= E_TOL && Error_H >= -E_TOL) && stopH==0)
		{
			setpoint.linear.x=def_speed_l;
			setpoint.angular.z=0.0;
			set_p.publish(setpoint);
//			std::cout<<"If l";
		}
		else if((Error_H > E_TOL || Error_H < -E_TOL) && stopH==0)
		{
			if(speed_r<0.0)
			{
//				setpoint.linear.x=(-1.0)*(speed_r*0.19);
			}
			else
			{
//				setpoint.linear.x=speed_r*0.19;
			}
			setpoint.linear.x=speed_r*0.19;
			setpoint.angular.z=speed_r;
			set_p.publish(setpoint);
//			std::cout<<"If r";
		}
//----------------------------------------------------------------------------------------		
		if((Error_H <= E_TOL && Error_H >= -E_TOL) && stopH==0)
		{
			setpoint.linear.x=def_speed_l;
			setpoint.angular.z=0.0;
			set_p.publish(setpoint);
//			std::cout<<"If l";
		}
		else if((Error_H > E_TOL || Error_H < -E_TOL) && stopH==0)
		{
			setpoint.linear.x=0.0;
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
		}
*/


		Heading.data = current_H;
		HEADING.publish(Heading);
		H_SET.data=0.0;
		H_SET_S.publish(H_SET);

//		ROS_INFO("P:%f, I:%f, D:%f",kp_H,ki_H,kd_H);
		ROS_INFO("Error:%f, S:%f, A:%f, X:%f, Y:%f, L:%f, R: %f",Error_H,S,A,pos.x,pos.y,def_speed_l,speed_r);
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
	pos.z=P->z;
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
	ros::init(argc,argv,"Position_control");
	ros::NodeHandle n;

//	ros::Subscriber get_pid=n.subscribe<geometry_msgs::Vector3>("/pid1",10,&up_pidCB);

	ros::Subscriber get_pos=n.subscribe<geometry_msgs::Vector3>("/pos",10,&posCB);
//	ros::Subscriber get_heading=n.subscribe<geometry_msgs::Vector3>("/brick_imu",50,&headingCB);
	ros::Subscriber heading_set_p=n.subscribe<geometry_msgs::Twist>("/SET_HEADING",10,&set_headingCB); //Heading control setpoint
	ros::Subscriber turns = n.subscribe<std_msgs::Int32>("/TURNS", 10,&turnsCB);
	ros::Subscriber STP = n.subscribe<std_msgs::Int32>("/STOP", 10,&stopCB);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",50,&getenclCB);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",50,&getencrCB);
	ros::Subscriber heading_set_t = n.subscribe<geometry_msgs::Twist>("/TURN_HEADING", 10,&turnupCB);

	set_p = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	update_heading = n.advertise<std_msgs::Float64>("/head_up", 10);
	HEADING = n.advertise<std_msgs::Float64>("/heading", 10);
	H_SET_S = n.advertise<std_msgs::Float64>("/H_SET_S", 10);
	Angle = n.advertise<std_msgs::Float64>("/A", 10);
	Angle1 = n.advertise<std_msgs::Float64>("/A1", 10);
	Angle2 = n.advertise<std_msgs::Float64>("/A2", 10);
	t_1=n.createTimer(ros::Duration (dt_H),t1CB);
	t_2=n.createTimer(ros::Duration (0.04),pos_up);

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
			LAP=0;	//	**
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

