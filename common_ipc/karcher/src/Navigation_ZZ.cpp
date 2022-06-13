/*
*/

#include"ros/ros.h"
#include"std_msgs/Int8.h"
#include"std_msgs/Int32.h"
#include"std_msgs/Int64.h"
#include"std_msgs/Header.h"
#include"std_msgs/Float64.h"
#include"geometry_msgs/Twist.h"
#include"geometry_msgs/Vector3.h"
#include<fstream>

std::ofstream myf;

using namespace std;
ros::Timer t_1,t_2;

geometry_msgs::Twist heading_turn;
geometry_msgs::Vector3 pos,G;
std_msgs::Float64 dist,SX,SY;

ros::Publisher heading_set_t,set_dist,S_X,S_Y,POS;
ros::Publisher Distance;

const float LENC_PPR=500.0;
const float RENC_PPR=500.0;
const float WHEEL_D=0.20;	//Diameter in m
const float WHEEL_2_WHEEL=0.4168;	//Length in m
const float WHEEL_2_WHEEL_INV=2.399232246;	//Length in m
const float _PI_=3.14159265359;	//PI
const float DEG=180.0/_PI_;	//DEG
const float RAD=_PI_/180.0;	//RAD

int i=0,oe;

int START=1,SETD=1,SETT=1;
int current_turn;
float current_dist;

int enc_l,pl;
int start_encl,end_encl,L_d;
int enc_r,pr;
int start_encr,end_encr,R_d;

int start1=10,start2=10,PAUSE=1;
float total_dist,right_dist,left_dist;
float theta,X_,Y_,X,Y,ptheta,Theta;
double t,dt,pdt;

float SET_DIST=0.00,LEFT_SET=0.0,RIGHT_SET=0.0,LEFT_SET_H=0.0,RIGHT_SET_H=0.0,LEFT_SET_T=0.0,RIGHT_SET_T=0.0;
float DIST,S,q,A_;
int TYPE,setf=2,SET_S=1;


void getstartCB(const std_msgs::Int32::ConstPtr& msg)
{
	int st;
	st = msg->data;
	ros::param::get("/SET_DISTANCE",DIST);
	ros::param::get("/LEFT_SET",LEFT_SET);
	ros::param::get("/RIGHT_SET",RIGHT_SET);
	ros::param::get("/TURN_TYPE",TYPE);
	if(st==0)
	{
		START=0;
		SETD=0;
		SET_DIST=DIST;	
		LEFT_SET_H = LEFT_SET;
		RIGHT_SET_H = RIGHT_SET;
		LEFT_SET_T = 0.0;
		RIGHT_SET_T = 0.0;
		start1=0;
		start2=0;
		setf=0;
		end_encl=enc_l;
		end_encr=enc_r;
	}
	else
	{
		SET_DIST=0.0;
		SETD=1;
		SETT=1;
		START=1;
		setf=10;
		start1=10;
		start2=10;
		end_encl=enc_l;
		end_encr=enc_r;
	}

	if(TYPE==1)
	{
		oe=1;
	}
	if(TYPE==2)
	{
		oe=2;
	}
}

void odomCB(const ros::TimerEvent& )
{
	float q;
	if(start1<2)
	{
		end_encl=enc_l;
		start1++;
	}
	if(start2<2)
	{
		end_encr=enc_r;
		start2++;
	}
	if(setf==0)
	{
		if(i<2)
		{
			S=G.x;
			i++;
		}
		else
		{
			i=2;
			setf=1;
			A_=0.0;
			SET_S=0;
		}
	}
	if(setf==1)	//	**
	{
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
				A_=(q-360.0)*_PI_*(1.0/180.0);
			}
			else
			{
				A_=q*_PI_*(1.0/180.0);
			}
		}
	}
	if(start1>=2&&start1<5 && start2>=2 && start2<5)
	{
		float sx,sy,theta_;
		dt=ros::Time::now().toSec()-pdt;
		L_d=(enc_l-end_encl);
		R_d=(enc_r-end_encr);
		end_encl=enc_l;
		end_encr=enc_r;
		pdt=t;

		if(enc_l!=pl||enc_r!=pr)
		{
			myf<<enc_l<<","<<enc_r<<","<<A_<<","<<theta<<","<<X<<","<<Y<<",\n";
			pl=enc_l;
			pr=enc_r;
		}

		left_dist=float(L_d)*((WHEEL_D*_PI_)/LENC_PPR);
		right_dist=float(R_d)*((WHEEL_D*_PI_)/RENC_PPR);
		total_dist=(left_dist+right_dist)*0.5;

		theta_=(left_dist-right_dist);
		theta_*=WHEEL_2_WHEEL_INV;

		X_=total_dist * sin((Theta));
		Y_=total_dist * cos((Theta));
		X+=X_;
		Y+=Y_;
		theta+=theta_;
		Theta=(theta*0.00)+(A_*1.00);

		if(X==0.0)
		{
//			S=G.x;
		}

		sx=X_/dt;
		sy=Y_/dt;

		if(theta_!=ptheta)
		{
	//	myf <<enc_l<<","<<enc_r<<","<<left_dist<<","<<right_dist<<","<<total_dist<<","
	//		<<theta_<<","<<X_<<","<<Y_<<","<<X<<","<<Y<<","<<theta<<",\n";
			ptheta=theta_;
		}

		if(theta>_PI_)
		{
			theta -= (2.0*_PI_);
		}
		else if(theta<= -_PI_)
		{
			theta += (2.0*_PI_);
		}

	//	SX.data=sx;
	//	S_X.publish(SX);
	//	SY.data=sy;
	//	S_Y.publish(SY);

		std::cout<<"theta1: "<<theta<<" theta2: "<<A_<<"\n";
		pos.x=X;
		pos.y=Y;
		pos.z=Theta;
		POS.publish(pos);
	}
	else
	{
		X=0.0;
		Y=0.0;
		theta=0.0;
	}
}

void t1CB(const ros::TimerEvent& )
{
	if(START==0 && PAUSE==1)
	{

		if(SETD==0)
		{
			current_dist=0.0;
			if(i>=10)
			{
				ros::param::set("/LEFT_SET_H",LEFT_SET_H);
				ros::param::set("/RIGHT_SET_H",RIGHT_SET_H);
				dist.data = SET_DIST;
				set_dist.publish(dist);
				
				SETD=1;
				SETT=0;
				i=0;
			}
			else
			{
				i++;
			}
			std::cout<<"i "<<i<<"\n";
		}

		if(DIST-current_dist<=0.05 && SETT==0)
		{
			if(i<1)
			{
				dist.data = 0.0;
				set_dist.publish(dist);
			}
			if(i>=10)
			{
				if(oe%2==0 && oe!=0)
				{
					ROS_INFO("Reached, TURNING Right:%f",LEFT_SET);
					heading_turn.linear.x = 0.0285;
					heading_turn.angular.z = 0.0;
					heading_set_t.publish(heading_turn);
				}
				else if(oe%2==1 && oe!=0)
				{
					ROS_INFO("Reached, TURNING Left:%f",RIGHT_SET);
					heading_turn.linear.x = 0.0;
					heading_turn.angular.z = 0.0285;
					heading_set_t.publish(heading_turn);
				}
				oe+=1;
				SETT=1;
				i=0;
				if(oe>=4)
				{
//					oe=1;
				}
			}
			else
			{
				i++;
			}
			std::cout<<"i "<<i<<"\n";
		}
	}
}

void getpauseCB(const std_msgs::Int32::ConstPtr& d)
{
	PAUSE=d->data;	

	if(START==0 && PAUSE==0)
	{
		heading_turn.linear.x = 0.0;
		heading_turn.angular.z = 0.0;
		heading_set_t.publish(heading_turn);		
	}
}

void headingCB(const geometry_msgs::Vector3::ConstPtr& a)
{
	G.x=a->x;
}

void getdistCB(const std_msgs::Float64::ConstPtr& d)
{
	current_dist=d->data;	
}
void getturnCB(const std_msgs::Int32::ConstPtr& t)
{
	current_turn=t->data;
	SETD=0;
}

void getenclCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_l = msg->data;
}

void getencrCB(const std_msgs::Int64::ConstPtr& msg)
{
	enc_r = msg->data;
}

int main(int argc,char **argv)
{
	myf.open("/home/bros/DATS.csv");
	myf<<"enc_l,enc_r,A_,theta,X,Y,\n";
	ros::init(argc,argv,"Navigation_ZZ");
	ros::NodeHandle n;

	ros::Subscriber get_Pause=n.subscribe<std_msgs::Int32>("/PAUSE",10,&getpauseCB);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",50,&getenclCB);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",50,&getencrCB);
	ros::Subscriber get_heading=n.subscribe<geometry_msgs::Vector3>("/brick_imu",50,&headingCB);

	ros::Subscriber get_Start=n.subscribe<std_msgs::Int32>("/START",10,&getstartCB);
	ros::Subscriber Distance = n.subscribe<std_msgs::Float64>("/DISTANCE", 10,&getdistCB);
	ros::Subscriber turns = n.subscribe<std_msgs::Int32>("/TURNS", 10,&getturnCB);

	heading_set_t = n.advertise<geometry_msgs::Twist>("/TURN_HEADING", 10);
	set_dist=n.advertise<std_msgs::Float64>("/SET_DIST",10);
	POS=n.advertise<geometry_msgs::Vector3>("/pos",10);
	S_X=n.advertise<std_msgs::Float64>("/SX",10);
	S_Y=n.advertise<std_msgs::Float64>("/SY",10);

	t_1=n.createTimer(ros::Duration (0.01),t1CB);
	t_2=n.createTimer(ros::Duration (0.01),odomCB);

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
		myf.close();
	}
	return 0;
}

