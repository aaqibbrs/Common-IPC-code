#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <wiringPi.h>
#include<iostream>
#include <time.h>
#include <softPwm.h>
#include <std_msgs/Float32.h>

using namespace std;

float mov1, mov1_;
int dir, x, y;

const int dir1leftWheel = 11;
const int dir2leftWheel = 10;
const int vel_leftWheel = 6;

const int brush = 7;
const int vaccum = 8;

const int dir1rightWheel = 15;
const int dir2rightWheel = 16;
const int vel_rightWheel = 1;

void movecallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    mov1 = cmd_vel->linear.x;
    mov1_ = cmd_vel->angular.z;
    mov1 = (mov1*50.0)/255.0;
    mov1_ = (mov1_*50.0)/255.0;
//    ROS_INFO("LEFT:%f, RIGHT:%f",mov1,mov1_);
}

void dircallback(const std_msgs::Int8::ConstPtr& dir_data)
{
    dir = dir_data->data;
    if (dir == 10 || dir == 12)
    {
        digitalWrite(dir1leftWheel, LOW);
        digitalWrite(dir2leftWheel, HIGH);
        softPwmWrite(vel_leftWheel, int(mov1));
        digitalWrite(dir1rightWheel, HIGH);
        digitalWrite(dir2rightWheel, LOW);
        softPwmWrite(vel_rightWheel, int(mov1_));
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
	ROS_INFO("RIGHT");
    }
  //Left
    if (dir == 20 || dir == 22) 
    {    
        digitalWrite(dir1leftWheel, HIGH);
        digitalWrite(dir2leftWheel, LOW);
        softPwmWrite(vel_leftWheel, int(mov1));
        digitalWrite(dir1rightWheel, LOW);
        digitalWrite(dir2rightWheel, HIGH);
        softPwmWrite(vel_rightWheel, int(mov1_));
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
	ROS_INFO("LEFT");
    }

  //Backward
    if (dir == 40)
    {
        softPwmWrite(vel_leftWheel, int(mov1));
        digitalWrite(dir1leftWheel, HIGH);
        digitalWrite(dir2leftWheel, LOW);
        softPwmWrite(vel_rightWheel, int(mov1_));
        digitalWrite(dir1rightWheel, HIGH);
        digitalWrite(dir2rightWheel, LOW);
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
	ROS_INFO("BACKWARD: %d, %d",int(mov1),int(mov1_));
    }
  //Forward
    if (dir == 30)
    {
		if(mov1>0.0)
		{
		    digitalWrite(dir1leftWheel, LOW);
		    digitalWrite(dir2leftWheel, HIGH);
		}
		else if(mov1<0.0)
		{
		    digitalWrite(dir1leftWheel, HIGH);
		    digitalWrite(dir2leftWheel, LOW);
		}

	        softPwmWrite(vel_leftWheel, int(mov1));

		if(mov1_>0.0)
		{
		    digitalWrite(dir1rightWheel, LOW);
		    digitalWrite(dir2rightWheel, HIGH);
		}
		if(mov1_<0.0)
		{
		    digitalWrite(dir1rightWheel, HIGH);
		    digitalWrite(dir2rightWheel, LOW);
		}

        softPwmWrite(vel_rightWheel, int(mov1_));

        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);
	ROS_INFO("FORWARD: %d, %d",int(mov1),int(mov1_));
    }
  //Stop
    if (dir == 50)
    {
        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);

        for (x = int(mov1), y = int(mov1_); x >= 0;)
        {
            softPwmWrite(vel_leftWheel, x);
            digitalWrite(dir1leftWheel, LOW);
            digitalWrite(dir2leftWheel, HIGH);
            softPwmWrite(vel_rightWheel, y);
            digitalWrite(dir1rightWheel, LOW);
            digitalWrite(dir2rightWheel, HIGH);
            x -= 5;
            y -= 5;
        }
	ROS_INFO("STOP");

    }
}

void messageCb(const geometry_msgs::Twist::ConstPtr& cmd_) {
    dir = 0;
    float cmdx,cmdy;
    cmdx = (cmd_->linear.x*50.0)/255.0;
    cmdy = (cmd_->angular.z*50.0)/255.0;

    if ((cmdx == 0.0) && (cmdy == 0.0))
    {
        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);
    }
    else
    {
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
    }
    softPwmWrite(vel_leftWheel, int(cmdx));
    digitalWrite(dir1leftWheel, HIGH);
    digitalWrite(dir2leftWheel, LOW);
    softPwmWrite(vel_rightWheel, int(cmdy));
    digitalWrite(dir1rightWheel, HIGH);
    digitalWrite(dir2rightWheel, LOW);
}

int main(int argc, char **argv)
{
    wiringPiSetup();
    pinMode(dir1leftWheel, OUTPUT); //l
    pinMode(dir2leftWheel, OUTPUT); //l
    softPwmCreate(vel_leftWheel, 0, 50); // leftmotor

    pinMode(dir1rightWheel, OUTPUT);//r
    pinMode(dir2rightWheel, OUTPUT);//r
    softPwmCreate(vel_rightWheel, 0,50); //rightmotor

    pinMode(7, OUTPUT); //brush
    pinMode(8, OUTPUT); //vaccum

    ros::init(argc, argv, "Motion_wiring");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, messageCb);
    ros::Subscriber sub2 = n.subscribe<geometry_msgs::Twist>("/mov_cmd", 1000, movecallback);
    ros::Subscriber sub3 = n.subscribe<std_msgs::Int8>("/dir_cmd", 1000, dircallback);
	ROS_INFO("HEY");
    ros::Rate r(500);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	ros::spin();
	if(ros::isShuttingDown())
	{
		softPwmWrite(vel_leftWheel, 0);
        softPwmWrite(vel_rightWheel, 0);
	}

    ros::spin();

  return 0;
}
