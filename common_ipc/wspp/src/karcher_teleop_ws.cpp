#include<string.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>


float d = 0.0;
std_msgs::Float64 dist;
std_msgs::Int32 STRT,STP;

geometry_msgs::Twist cmd,setpoint,heading,heading_turn;
std_msgs::Int8 dir,res,starter,PAU;
ros::Publisher mov_cmd;
ros::Publisher dir_cmd;
ros::Publisher set_p,heading_set_p,heading_set_t,set_d,nav_zz,STOP,starting,PAUSE;

// thresh1 => LEFT MOTOR PWM
// thresh2 => RIGHT MOTOR PWM

//int thresh2=76,thresh1=70; //PiGPIO
//int thresh2=66, thresh1=60; //WirinPi
float thresh2=0.63, thresh1=0.12; //teleop_twist

char c;

float DIST = 0.0;
float LEFT_SET = 0.0, RIGHT_SET = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_H = 0.0, RIGHT_SET_H = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_T = 0.0, RIGHT_SET_T = 0.0;	// Left wheel & right wheel RPM setpoint 

std::string COMMAND;
static struct termios oldt, newt;
int getch()
{
//  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void teleopcb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}

void navzzcb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}
void stopcb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}
void pausecb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}
void resumecb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}
void startingcb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}
void startsfcb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}
void startwfcb(const std_msgs::String::ConstPtr& a)
{
	COMMAND=a->data.c_str();
	ROS_INFO("Command:%s",a->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "karcher_teleop_ws");
	ros::NodeHandle n;

//	ros::Subscriber test = n.subscribe<std_msgs::Int8>("/test", 10,testcb);

	ros::Subscriber mov_cmd_str = n.subscribe<std_msgs::String>("/teleop", 50,teleopcb);	//CMD_VEL
	ros::Subscriber nav_zz_str=n.subscribe<std_msgs::String>("/NAV_ZZ",10,navzzcb);	//START ZZ L TYPE

	ros::Subscriber STOP_str=n.subscribe<std_msgs::String>("/STOP_",10,stopcb);	//STOP
    ros::Subscriber PAUSE_str = n.subscribe<std_msgs::String>("/PAUSE_",10,pausecb);	//PAUSE / RESUME
    ros::Subscriber RESUME_str = n.subscribe<std_msgs::String>("/RESUME_",10,resumecb);	//RESUME

	ros::Subscriber starting_str = n.subscribe<std_msgs::String>("/starting_", 10,startingcb);
	ros::Publisher start_sf_str = n.advertise<std_msgs::String>("/start_sf",10,startsfcb);
	ros::Publisher start_wf_str = n.advertise<std_msgs::String>("/start_wf",10,startwfcb);

	mov_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
//	dir_cmd = n.advertise<std_msgs::Int8>("/DIR", 20);

//	set_p = n.advertise<geometry_msgs::Twist>("/SETPOINT", 1000);

	heading_set_p = n.advertise<geometry_msgs::Twist>("/SET_HEADING", 10);
	heading_set_t = n.advertise<geometry_msgs::Twist>("/TURN_HEADING", 10);
//	set_p = n.advertise<geometry_msgs::Twist>("/SETPOINT", 20);
	set_d=n.advertise<std_msgs::Float64>("/SET_DIST",10);
	nav_zz=n.advertise<std_msgs::Int32>("/START",10);
	STOP=n.advertise<std_msgs::Int32>("/STOP",10);
	starting = n.advertise<std_msgs::Int8>("/starting", 10);
    PAUSE = n.advertise<std_msgs::Int32>("/PAUSE",10);

    ros::Publisher reset = n.advertise<std_msgs::Int8>("/RESET",10);

	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		if(COMMAND=="RIGHT")
		{
			ROS_INFO("Right Key Pressed");
			cmd.linear.x=thresh1;
			cmd.angular.z=-thresh2;
			mov_cmd.publish(cmd);
			COMMAND="NULL";
		}

		else if(COMMAND=="LEFT")
		{
			ROS_INFO("Left key Pressed");
			cmd.linear.x=thresh1;
			cmd.angular.z=thresh2;
			mov_cmd.publish(cmd);
			COMMAND="NULL";
		}

		else if(COMMAND=="FORWARD")
		{
			ros::param::get("/PWM_L",thresh1);
			ros::param::get("/PWM_R",thresh2);
			ros::param::get("/LEFT_SET",LEFT_SET);
			ros::param::get("/RIGHT_SET",RIGHT_SET);
			ROS_INFO("Forward Key Pressed");
			cmd.linear.x=thresh1;
			cmd.angular.z=d;
			mov_cmd.publish(cmd);	//comment for RPM control in forward motion
			COMMAND="NULL";
		}

/*
		else if(c=='h')
		{
			ros::param::get("/LEFT_SET_H",LEFT_SET_H);
			ros::param::get("/RIGHT_SET_H",RIGHT_SET_H);
			ROS_INFO("Heading Control");
			STP.data = 0;
			STOP.publish(STP);
			heading.linear.x = LEFT_SET_H;	//comment for normal opreation (without RPM control)
			heading.angular.z = RIGHT_SET_H;	//comment for normal opreation (without RPM control)
			heading_set_p.publish(heading);	//comment for normal opreation (without RPM control)
		}

		else if(c=='t')
		{
			ros::param::get("/LEFT_SET_T",LEFT_SET_T);
			ros::param::get("/RIGHT_SET_T",RIGHT_SET_T);
			ROS_INFO("Turn Control");
			heading.linear.x = 0.0;	//comment for normal opreation (without RPM control)
			heading.angular.z = 0.0;	//comment for normal opreation (without RPM control)
			heading_set_p.publish(heading);	//comment for normal opreation (without RPM control)
			heading_turn.linear.x = LEFT_SET_T;	//comment for normal opreation (without RPM control)
			heading_turn.angular.z = RIGHT_SET_T;	//comment for normal opreation (without RPM control)
			heading_set_t.publish(heading_turn);	//comment for normal opreation (without RPM control)
		}

		else if(c=='y')
		{
			ros::param::get("/SET_DISTANCE",DIST);
			ROS_INFO("Distance Control: %f",DIST);
			dist.data = DIST;
			set_d.publish(dist);
		}
*/
		else if(COMMAND=="START_L")
		{
			ros::param::get("/LEFT_SET",LEFT_SET);
			ros::param::get("/RIGHT_SET",RIGHT_SET);
			ros::param::set("/TURN_TYPE",1);
			ROS_INFO("Navigate L_TYPE ZZ: %f, %f",LEFT_SET, RIGHT_SET);
			STRT.data = 0;
			nav_zz.publish(STRT);
			STP.data = 0;
			STOP.publish(STP);
			COMMAND="NULL";
		}
		else if(COMMAND=="START_R")
		{
			ros::param::get("/LEFT_SET",LEFT_SET);
			ros::param::get("/RIGHT_SET",RIGHT_SET);
			ros::param::set("/TURN_TYPE",2);
			ROS_INFO("Navigate R_TYPE ZZ: %f, %f",LEFT_SET, RIGHT_SET);
			STRT.data = 0;
			nav_zz.publish(STRT);
			STP.data = 0;
			STOP.publish(STP);
			COMMAND="NULL";
		}

		else if(COMMAND=="BACKWARD")
		{
			ROS_INFO("BackWard Key Pressed");
			cmd.linear.x=-thresh1;
			cmd.angular.z=d;
			mov_cmd.publish(cmd);
			COMMAND="NULL";
		}

		else if (COMMAND=="PAUSE")
		{
			ROS_INFO("Pause Navigation");
			PAU.data=0;
			PAUSE.publish(PAU);
			COMMAND="NULL";
		}
		else if (COMMAND=="RESUME")
		{
			ROS_INFO("Resume Navigation");
			PAU.data=1;
			PAUSE.publish(PAU);
			COMMAND="NULL";
		}

		else if (COMMAND=="STOP")
		{
			ROS_INFO("Stop key pressed");
			cmd.linear.x=d;
			cmd.angular.z=d;
			mov_cmd.publish(cmd);

			dist.data = 0.0;
			set_d.publish(dist);

			heading_turn.linear.x = 0.0;	//comment for normal opreation (without RPM control)
			heading_turn.angular.z = 0.0;	//comment for normal opreation (without RPM control)
			heading_set_t.publish(heading_turn);	//comment for normal opreation (without RPM control)

			heading.linear.x = 0.0;	//comment for normal opreation (without RPM control)
			heading.angular.z = 0.0;	//comment for normal opreation (without RPM control)
			heading_set_p.publish(heading);	//comment for normal opreation (without RPM control)

			starter.data = 0;
			starting.publish(starter);	//lsf stop

/*
			setpoint.linear.x=0.0;
			setpoint.angular.z=0.0;
			set_p.publish(setpoint);
*/
			STRT.data = 1;
			nav_zz.publish(STRT);
			STP.data = 1;
			STOP.publish(STP);
			COMMAND="NULL";
		}
		else if(COMMAND=="START_LWF")
		{
			ROS_INFO("Start left-wf key pressed");
			starter.data = 10;
			starting.publish(starter);
			COMMAND="NULL";
		}
		else if(COMMAND=="START_RWF")
		{
			ROS_INFO("Start right-wf key pressed");
			starter.data = 20;
			starting.publish(starter);
			COMMAND="NULL";
		}
		else if(COMMAND=="START_LSF")
		{
			ROS_INFO("Start left-sf key pressed");
			starter.data = 30;
			starting.publish(starter);
			COMMAND="NULL";
		}
		else if(COMMAND=="START_RSF")
		{
			ROS_INFO("Start right-sf key pressed");
			starter.data = 40;
			starting.publish(starter);
			COMMAND="NULL";
		}
		else if(c=='n')
		{
			starter.data = 110;
			starting.publish(starter);
		}
		else if(c=='m')
		{
			starter.data = 51;
			starting.publish(starter);
		}
		else if(c=='p')
		{
			starter.data = 31;
			starting.publish(starter);
		}
		else if(c=='l')
		{
			starter.data = 41;
			starting.publish(starter);
		}
//--------------------------------------------------------
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();   
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return 0;
}
