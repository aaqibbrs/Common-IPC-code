/*
	updated_logic
	Requires Speed setpoint
	Speed control => RPM control
*/

#include"ros/ros.h"
#include"std_msgs/Int8.h"
#include"std_msgs/Int32.h"
#include"std_msgs/Header.h"
#include"std_msgs/Float64.h"
#include"geometry_msgs/Twist.h"
#include"geometry_msgs/Vector3.h"

using namespace std;

ros::Timer t_1,t_2;

geometry_msgs::Twist cmd;
std_msgs::Int8 dir;
std_msgs::Float64 L_SPEED,R_SPEED,l_SET,r_SET;

ros::Publisher pub_vel;
ros::Publisher mov_cmd;
ros::Publisher dir_cmd;
ros::Publisher LEFT_SPEED;
ros::Publisher LEFT_SET_S;
ros::Publisher RIGHT_SPEED;
ros::Publisher RIGHT_SET_S;

std_msgs::Header tl,tr;


const float LENC_PPR=400.0;
const float RENC_PPR=400.0;	

const float MAX_SPEED=0.30;
const float MIN_SPEED=0.03;	

const float MAX_PWM=250.0;
const float MIN_PWM=-250.0;

const float WHEEL_D=0.20;	//Diameter in m
const float _PI_=3.14159265359;	//PI

// LEFT wheel RPM calculation variables
int enc_l;	//the time period of pulses of count stored in dl is used to calculate the RPM
double tima_l,timb_l,dtim_l;
int stopl=1;
float PWM_l;
int al=0;

// RIGHT wheel RPM calculation variables
int enc_r;	//the time period of pulses of count stored in dr is used to calculate the RPM
double tima_r,timb_r,dtim_r;
int stopr=1;
float PWM_r;
int ar=0;

const float P=2.6,I=2.35,D=2.85,dt=0.05;

// LEFT wheel PID variables
float kp_l=P,ki_l=I,kd_l=D,dt_l=dt;
float Error_l,S_error_l,P_error_l,D_error_l,Output_l,current_l;

// RIGHT wheel PID variables
float kp_r=P,ki_r=I,kd_r=D,dt_r=dt;
float Error_r,S_error_r,P_error_r,D_error_r,Output_r,current_r;

int lenc,plenc;
float l_RPM;

int renc,prenc;
float r_RPM;


float LEFT_SET=0.05,RIGHT_SET=0.05;	//	Left wheel & right wheel setpoint variables
float pLEFT_SET=0.0,pRIGHT_SET=0.0;	//	Left wheel & right wheel setpoint variables

float to_RPM(float mps)
{
	return ((mps/(WHEEL_D*_PI_))*60.0);
}

float to_MPS(float rpm)
{
	return ((rpm/60.0)*(WHEEL_D*_PI_));
}

void setpCB(const geometry_msgs::Twist::ConstPtr& set)
{
	float setl,setr;
	setl=set->linear.x;
	setr=set->angular.z;

	if(setl>=MIN_SPEED && setl<=MAX_SPEED)
	{
		LEFT_SET=to_RPM(setl);	// per sec
		if(pLEFT_SET==0.0)
		{
			stopl=0;
			al=0;
		}
	}
	else
	{
		LEFT_SET=0.0;	
		stopl=1;
		al=3;
		cmd.linear.x=0.0;
		dir.data=30;
		pLEFT_SET=0.0;
	}
	if(setr>=MIN_SPEED && setr<=MAX_SPEED)
	{
		RIGHT_SET=to_RPM(setr);
		if(pRIGHT_SET==0.0)
		{
			stopr=0;
			ar=0;
		}
	}
	else
	{
		RIGHT_SET=0.0;
		stopr=1;
		ar=3;
		cmd.angular.z=0.0;
		dir.data=30;
		pRIGHT_SET=0.0;
	}
	mov_cmd.publish(cmd);
	dir_cmd.publish(dir);
	
}

float lenc_up()
{
	int d,i;
	lenc=enc_l;
	d=lenc-plenc;
	tima_l=ros::Time::now().toSec();
	dtim_l=tima_l-timb_l;
	if(d<0)
	{
		d=-d;
	}
	if(dtim_l>0.005)
	{
		l_RPM=((float)d/LENC_PPR)*(1.0/dtim_l)*60.0;
		timb_l=tima_l;
		plenc=lenc;
	}
	else if (dtim_l<0.005)
	{
		l_RPM=0.0;
	}

	return l_RPM;
}

float renc_up()
{
	int d,i;
	renc=enc_r;
	d=renc-prenc;
	tima_r=ros::Time::now().toSec();
	dtim_r=tima_r-timb_r;
	if(d<0)
	{
		d=-d;
	}
	if(dtim_r>0.005)
	{
		r_RPM=((float)d/RENC_PPR)*(1.0/dtim_r)*60.0;
//		r_RPM=((float)d/RENC_PPR)/(dtim_r/60.0);
		timb_r=tima_r;
		prenc=renc;
	}
	else if(dtim_r<0.005)
	{
		r_RPM=0.0;
	}
	return r_RPM;
}

void t2CB(const ros::TimerEvent& )
{
	if(stopr==1)
	{
		Error_r=0.0;
		S_error_r=0.0;
		P_error_r=0.0;
	}
	if(stopr==0)
	{
		current_r = renc_up();
		Error_r = RIGHT_SET-current_r;
		S_error_r = S_error_r + Error_r;
		D_error_r = (Error_r - P_error_r);
		P_error_r = Error_r;
		pRIGHT_SET=RIGHT_SET;
		if(Error_r==0.0 && RIGHT_SET==0.0)
		{
			S_error_r=0.0;
		}

		Output_r = (kp_r * Error_r) + (ki_r * S_error_r) + (kd_r * D_error_r);

		if(Output_r>MAX_PWM)
		{
			Output_r=MAX_PWM;
		}
		if(Output_r<-MAX_PWM)
		{
			Output_r=-MAX_PWM;
		}

		PWM_r = Output_r;

		if(stopr==0)
		{
			cmd.angular.z=PWM_r;
			dir.data=30;
			mov_cmd.publish(cmd);
			dir_cmd.publish(dir);
		}

		else
		{
			stopr=1;
			cmd.angular.z=0.0;
			dir.data=30;
			mov_cmd.publish(cmd);
			dir_cmd.publish(dir);
			pRIGHT_SET=0.0;
		}

		R_SPEED.data = to_MPS(current_r);
		RIGHT_SPEED.publish(R_SPEED);
		r_SET.data=to_MPS(RIGHT_SET);
		RIGHT_SET_S.publish(r_SET);

		ROS_INFO("E_r:%f,\tSET_r:%f,\tcurrent_r:%f,\tPWM_r:%f",to_MPS(Error_r),to_MPS(RIGHT_SET),to_MPS(current_r),PWM_r);	// per sec

	}
}

void t1CB(const ros::TimerEvent& )
{
	if(stopl==1)
	{
		Error_l=0.0;
		S_error_l=0.0;
		P_error_l=0.0;
	}
	if(stopl==0)
	{
		current_l = lenc_up();
		Error_l = LEFT_SET-current_l;
		S_error_l = S_error_l + Error_l;
		D_error_l = (Error_l - P_error_l);
		P_error_l = Error_l;
		pLEFT_SET=LEFT_SET;

		if(Error_l==0.0 && LEFT_SET==0.0)
		{
			S_error_l=0.0;
		}

		Output_l = (kp_l * Error_l) + (ki_l * S_error_l) + (kd_l * D_error_l);

		if(Output_l>MAX_PWM)
		{
			Output_l=MAX_PWM;
		}
		if(Output_l<-MAX_PWM)
		{
			Output_l=-MAX_PWM;
		}

		PWM_l = Output_l;

		if(stopl==0)
		{
			cmd.linear.x=PWM_l;
			dir.data=30;
			mov_cmd.publish(cmd);
			dir_cmd.publish(dir);
		}
		else
		{
			stopl=1;
			cmd.linear.x=0.0;
			dir.data=30;
			mov_cmd.publish(cmd);
			dir_cmd.publish(dir);
			pLEFT_SET=0.0;
		}

		L_SPEED.data = to_MPS(current_l);
		LEFT_SPEED.publish(L_SPEED);
		l_SET.data=to_MPS(LEFT_SET);
		LEFT_SET_S.publish(l_SET);

		ROS_INFO("E_l:%f,\tSET_l:%f,\tcurrent_l:%f,\tPWM_l:%f",to_MPS(Error_l),to_MPS(LEFT_SET),to_MPS(current_l),PWM_l);	// per sec
	}
}

void getenclCB(const std_msgs::Int32::ConstPtr& msg)
{
	enc_l = msg->data;
	if(al<2)
	{
		plenc=enc_l;
		al++;
	}
}

void getencrCB(const std_msgs::Int32::ConstPtr& msg)
{
	enc_r = msg->data;
	if(ar<2)
	{
		prenc=enc_r;
		ar++;
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"Speed_control");
	ros::NodeHandle n;

	ros::Subscriber get_encl=n.subscribe<std_msgs::Int32>("/left_enc",10,&getenclCB);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int32>("/right_enc",10,&getencrCB);
	ros::Subscriber get_set_p=n.subscribe<geometry_msgs::Twist>("/SETPOINT",20,&setpCB); //RPM control setpoint
	mov_cmd = n.advertise<geometry_msgs::Twist>("/CMD", 20);
	dir_cmd = n.advertise<std_msgs::Int8>("/DIR", 20);

	LEFT_SPEED = n.advertise<std_msgs::Float64>("/LEFT_S", 10);
	LEFT_SET_S = n.advertise<std_msgs::Float64>("/SET_LEFT", 10);
	RIGHT_SPEED = n.advertise<std_msgs::Float64>("/RIGHT_S", 10);
	RIGHT_SET_S = n.advertise<std_msgs::Float64>("/SET_RIGHT", 10);

	t_1=n.createTimer(ros::Duration (dt_l),t1CB);
	t_2=n.createTimer(ros::Duration (dt_r),t2CB);

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
		t_2.stop();
	}
	return 0;
}

