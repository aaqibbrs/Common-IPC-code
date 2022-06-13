#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Header.h>
#include<iostream>
#include <time.h>
#include <wiringPi.h>

std_msgs::Int32 msg1, msg2;

std_msgs::Header tim_l,tim_r;

ros::Publisher left_encoder,right_encoder;
ros::Publisher timel,timer;

using namespace std;

int il=0,ir=0;

const int left1 = 5;	//Phase A pin of the encoder 1
const int left2 = 4;	//Phase B pin of the encoder 1

const int right1 = 26;	//Phase A pin of the encoder 2
const int right2 = 27;	//Phase B pin of the encoder 2

bool state_l1,state_l2;
bool pstate_l1,pstate_l2;
int count_left=0;
void counter_left();

bool state_r1,state_r2;
bool pstate_r1,pstate_r2;
int count_right=0;
void counter_right();


void _CBfl(void)
{
	state_l2=digitalRead(left2);
	state_l1=digitalRead(left1);

	if(state_l2!=pstate_l1)
	{
		count_left++;
	}
	else
	{
		count_left--;
	}
	pstate_l1=state_l1;
	pstate_l2=state_l2;
//	il++;

//	if(il==2)
//	{
//		il=0;
		msg1.data=count_left/2;
		left_encoder.publish(msg1);
		ROS_INFO("Left_encoder:%d",count_left/2);
//	}
}

void _CBfr(void)
{
	state_r2=digitalRead(right2);
	state_r1=digitalRead(right1);
	if(state_r2!=pstate_r1)
	{
		count_right++;
	}
	else
	{
		count_right--;
	}
	pstate_r1=state_r1;
	pstate_r2=state_r2;
//	ir++;
//	if(ir==2)
//	{
		ir=0;
		msg2.data=count_right/2;
		right_encoder.publish(msg2);
		ROS_INFO("Right_encoder:%d",count_right/2);
//	}
}

int main(int argc, char** argv)
{
    wiringPiSetup();
    pinMode(left1, INPUT); //l
    pinMode(left2, INPUT); //l
    pinMode(right1, INPUT); //l
    pinMode(right2, INPUT); //l

	pullUpDnControl(left1, PUD_UP);
	pullUpDnControl(left2, PUD_UP);

	pullUpDnControl(right1, PUD_UP);
	pullUpDnControl(right2, PUD_UP);

	pstate_l1=digitalRead(left1);
	pstate_r1=digitalRead(right1);

	ros::init(argc, argv, "Encoder_data_Node_wiring");
	ros::NodeHandle n;
	left_encoder = n.advertise<std_msgs::Int32>("/left_enc", 1000);
	right_encoder = n.advertise<std_msgs::Int32>("/right_enc", 1000);
	wiringPiISR(left1,INT_EDGE_BOTH,&_CBfl);
	wiringPiISR(right1,INT_EDGE_BOTH,&_CBfr);
	ros::Rate r(1000);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	ros::spin();
	if(ros::isShuttingDown())
	{
	}
	return 0;
}
