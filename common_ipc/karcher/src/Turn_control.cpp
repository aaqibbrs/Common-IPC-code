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
ros::Timer t_1;

geometry_msgs::Twist setpoint;
geometry_msgs::Vector3 G;
std_msgs::Int32 turn;
std_msgs::Float64 H_SET,Heading;

ros::Publisher HEADING,H_SET_S;

ros::Publisher setp,turns;

const float LENC_PPR=400.0;
const float RENC_PPR=400.0;	

const float WHEEL_D=0.20;	//Diameter in m
const float WHEEL_2_WHEEL=0.45077;	//Length in m
//const float R=WHEEL_2_WHEEL/2.0;	//TR calc
const float R=0.19;	//TR actual
const float WHEEL_2_WHEEL_INV=2.218426248;	//Length in m
const float _PI_=3.14159265359;	//PI
const float DEG=180.0/_PI_;	//DEG

const float MAX_SPEED=0.30;
const float MIN_SPEED=0.03;

const float TGT=1.5;

float TURN;

int setf=1,setg1=1,setg2=1,set_lt=1,set_rt=1,set_turn=1;
int GO1=1,GO2=1;
int stopH=1;
int i=0;

int T=0,START=1;

// LEFT wheel RPM calculation variables	
float speed_l,def_speed_l;

// RIGHT wheel RPM calculation variables
float speed_r,def_speed_r;

float S=0.0,A,A_prod,GA;
float dt=0.005;

float Diff;
float angle1,angle2,angle;
float dif;
int temp,pS=0;

int start1=0,start2=0;
int enc_r,start_encr,end_encr,R_d;
int enc_l,start_encl,end_encl,L_d;
float right_dist,left_dist,theta;

int pn=5,reshuff=0;
float pA;

void set_headingCB(const geometry_msgs::Twist::ConstPtr& set)
{
	float setl,setr;
	setl=set->linear.x;
	setr=set->angular.z;
	
	if(setl>=MIN_SPEED && setl<=MAX_SPEED)
	{
		std::cout<<"START:- G.x: "<<G.x<<" left_dist: "<<left_dist<<" right_dist: "<<right_dist<<"\n"<<"\n";
		setf=0;
		setg1=0;
		GO1=0;
		speed_l=setl;
		
		speed_r=(-setl)/R;		
		
		set_lt=0;
		i=0;
		START=0;
		pn=0;
		reshuff=0;
		pA=0.0;
		if(start1>0 && start2>0)
		{
			start1=0;
			start2=0;
		}
	}
	else
	{
		setg1=1;
		GO1=1;
//		speed_l=0.0;
		set_lt=1;
	}
	if(setr>=MIN_SPEED && setr<=MAX_SPEED)
	{
		std::cout<<"START:- G.x: "<<G.x<<" left_dist: "<<left_dist<<" right_dist: "<<right_dist<<"\n"<<"\n";
		setf=0;
		setg2=0;
		GO2=0;
		speed_l=setr;
		
		speed_r=setr/R;		

		set_rt=0;
		i=0;
		START=0;
		pn=1;
		reshuff=0;
		pA=0.0;
		if(start1>0 && start2>0)
		{
			start1=0;
			start2=0;
		}
	}
	else
	{
		setg2=1;
		GO2=1;
//		speed_r=0.0;
		set_rt=1;
	}
	if(setl<=0.01 && setr<=0.01)
	{
		stopH=1;
		setf=1;
		START=1;
		start1=0;
		start2=0;
		set_turn=1;
		pn=5;
		reshuff=360;
		pA=0.0;
	}
}

void headingCB(const geometry_msgs::Vector3::ConstPtr& Q)
{
	float q=0.0;
	float theta_;
	
	G.x=Q->x;
	if(START==0)
	{
		if(setf==0)
		{
			if(i<10)
			{
				S=G.x;
				i++;
			}
			else
			{
				i=10;
				setf=1;
				A=0.0;
				pS=0;
			}
		}
		else
		{
			if(G.x>=angle)
			{
				q = G.x - angle;
			}
			else
			{
				q= G.x - angle + 360.0;
			}
			
			if(pn==1)
			{
				A=q-359.99;

				if(reshuff<50)
				{
					if(round(A)!=pA)
					{
						reshuff++;
						pA=round(A);
					}

					if(A<-179.99)
					{
						A=-A-359.99;
					}
				}

				A+=angle1;
			}
			else if(pn==0)
			{
				A=q;
				
				if(reshuff<50)
				{
					if(round(A)!=pA)
					{
						reshuff++;
						pA=round(A);
					}
					if(A>179.99)
					{
						A=-A+359.99;
					}
				}
				
				A-=angle1;
			}
		}

		if(setf==1 && temp==1 && pS==0)
		{
			TURN=180.0;
			set_turn=0;
			temp=0;
			pS=1;
		}
	
//-------------------------------------------------------------------------------------

		if(start1>1 && start2>1)
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
			left_dist=float(L_d)*((WHEEL_D*_PI_)/LENC_PPR);
			right_dist=float(R_d)*((WHEEL_D*_PI_)/RENC_PPR);

			theta_=(left_dist-right_dist);
			theta_*=WHEEL_2_WHEEL_INV;
			theta_*=DEG;
			
			if(pn==1)
			{
				if(theta_>0.0)
				{
					theta=-theta_;
				}
				else
				{
					theta=theta_;
				}

				theta+=angle2;
			}
			else if(pn==0)
			{
				if(theta_<0.0)
				{
					theta=-theta_;
				}
				else
				{
					theta=theta_;
				}

				theta-=angle2;
			}
		}

//--------------------------------------------------------------------------------		
		if(setf==1 && setg1==0 && set_lt==0 && set_turn==0)
		{

			GA=TURN;
			std::cout<<"angle: "<<angle<<" angle1: "<<angle1<<" angle2: "<<angle2<<" S: "<<S<<" dif: "<<dif<<" TURN: "<<TURN<<" GAl: "<<GA;
			setg1=1;
			stopH=0;
			std::cout<<" GA:"<<GA<<"\n";
		}

		if(setf==1 && setg2==0 && set_rt==0 && set_turn==0)
		{

			GA = TURN;
			std::cout<<"angle: "<<angle<<" angle1: "<<angle1<<" angle2: "<<angle2<<" S: "<<S<<" dif: "<<dif<<" TURN: "<<TURN<<" GAr: "<<GA;
			setg2=1;
			stopH=0;
			std::cout<<" GA: "<<GA<<"\n";
		}
	}
}

void t1CB(const ros::TimerEvent& )
{
	if(stopH==0)
	{
		A_prod = (A + theta);
		A_prod *= 0.5;
//		A_prod = A;
		
		if(set_lt==0 || set_rt==0)
		{
			if(A_prod>=0.0)
			{
					Diff=GA-A_prod;
			}
			else
			{
				Diff=GA+A_prod;
			}
		}

		if(stopH==0 && (GO1==0||GO2==0))
		{
			setpoint.linear.x=speed_l;
			setpoint.angular.z=speed_r;
			setp.publish(setpoint);
			GO1=1;
			GO2=1;
			std::cout<<"speed_l: "<<speed_l<<" speed_r: "<<speed_r<<"\n";
		}

		if(Diff<TGT)
		{
			std::cout<<"STOP:- G.x: "<<G.x<<" left_dist: "<<left_dist<<" right_dist: "<<right_dist<<"\n";
			setpoint.linear.x=0.0;
			setpoint.angular.z=0.0;
			setp.publish(setpoint);
			turn.data=T++;
			turns.publish(turn);

			stopH=1;
		}

		Heading.data = A;
		HEADING.publish(Heading);
		H_SET.data=GA;
		H_SET_S.publish(H_SET);

//		ROS_INFO("theta:%f, angle2:%f, A:%f, angle1:%f, Goal:%f, Diff:%f, right_dist:%f",theta,angle2,A,angle1,GA,Diff,right_dist);
		ROS_INFO("G.x:%0.2f, A:%0.2f, theta:%0.2f, Goal:%0.2f, Diff:%0.2f, LE:%d, RE:%d",G.x,A,theta,GA,Diff,enc_l,enc_r);

	}
	
}

void angleCB(const std_msgs::Float64::ConstPtr a)
{
	angle=a->data;
	temp=1;
}

void angle1CB(const std_msgs::Float64::ConstPtr a)
{
	angle1=a->data;
}
void angle2CB(const std_msgs::Float64::ConstPtr a)
{
	angle2=a->data;
}

void getenclCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_l = msg->data;
	if(start1<2)
	{
		start_encl=enc_l;
		start1++;
	}
	else
	{
//		std::cout<<"enc_l: "<<enc_l<<"\n";
	}
}

void getencrCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_r = msg->data;
	if(start2<2)
	{
		start_encr=enc_r;
		start2++;
	}
	else
	{
//		std::cout<<"enc_r: "<<enc_r<<"\n";
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"Turn_control");
	ros::NodeHandle n;
	
	ros::Subscriber get_heading=n.subscribe<geometry_msgs::Vector3>("/brick_imu",50,&headingCB);
	ros::Subscriber heading_set_p=n.subscribe<geometry_msgs::Twist>("/TURN_HEADING",10,&set_headingCB); //RPM control setpoint
	ros::Subscriber Angle=n.subscribe<std_msgs::Float64>("/A",10,&angleCB);
	ros::Subscriber Angle1=n.subscribe<std_msgs::Float64>("/A1",10,&angle1CB);
	ros::Subscriber Angle2=n.subscribe<std_msgs::Float64>("/A2",10,&angle2CB);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",10,&getenclCB);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",10,&getencrCB);
	
	setp = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

	turns = n.advertise<std_msgs::Int32>("/TURNS", 10);

	HEADING = n.advertise<std_msgs::Float64>("/heading", 10);
	H_SET_S = n.advertise<std_msgs::Float64>("/H_SET_S", 10);
	t_1=n.createTimer(ros::Duration (dt),t1CB);
	
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

