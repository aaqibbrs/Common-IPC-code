#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>
#include <stdlib.h>

float d = 0.0, stopper_front = 0.0, stopper_right = 0.0, stopper_left = 0.0, total_ss;
std_msgs::Float64 dist;
std_msgs::Int32 STRT,STP,PAU;

int front_ss = 0, rigor =0, left_ss = 0, right_ss = 0;

geometry_msgs::Twist cmd,setpoint,heading,heading_turn;
std_msgs::Int8 dir,res,starter, rig, pose_starter;
ros::Publisher mov_cmd;
ros::Publisher dir_cmd;
ros::Publisher set_p,heading_set_p,heading_set_t,set_d,nav_zz,STOP,starting,PAUSE;

ros::Publisher rigorous;

int wall_step = 0, auto_nav = 0;

float S=0.25;
//float thresh2=0.25, thresh1=0.0475; //teleop_twist
float thresh2=0.3, thresh1=0.057; //teleop_twist

char c;

float DIST = 0.0;
float LEFT_SET = 0.0, RIGHT_SET = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_H = 0.0, RIGHT_SET_H = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_T = 0.0, RIGHT_SET_T = 0.0;	// Left wheel & right wheel RPM setpoint 

void dataCB(const geometry_msgs::Vector3::ConstPtr& dat)
{
	stopper_front = dat->x;
	stopper_right = dat->y;
	stopper_left = dat->z;

	total_ss = stopper_front + stopper_right + stopper_left;

	ROS_DEBUG("stopper_front in callback is %f", stopper_front);
	ROS_DEBUG("stopper_left in callback is %f", stopper_left);
	ROS_DEBUG("stopper_right in callback is %f", stopper_right);
}
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

void right_turn()
{
	ROS_INFO("Teleop : Right");
	cmd.linear.x=thresh1;
	cmd.angular.z=-thresh2;

	rig.data = 20;
	rigorous.publish(rig);//imp
}
void left_turn()
{
	ROS_INFO("Teleop : Left");
	cmd.linear.x=thresh1;
	cmd.angular.z=thresh2;

	rig.data = 10;
	rigorous.publish(rig);//imp
}
void straight()
{
	ros::param::get("/SPEED_S",S);
	ros::param::get("/PWM_L",thresh1);
	ros::param::get("/PWM_R",thresh2);
	ros::param::get("/LEFT_SET",LEFT_SET);
	ros::param::get("/RIGHT_SET",RIGHT_SET);
	ROS_INFO("Teleop : Forward");
	cmd.linear.x=S;
	cmd.angular.z=d;

	rig.data = 30;
	rigorous.publish(rig);//imp
}
void backward()
{
	ROS_INFO("Teleop : BackWard");
	cmd.linear.x=-S;
	cmd.angular.z=d;

	rig.data = 100;
	rigorous.publish(rig);//imp
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "karcher_teleop_termios");
	ros::NodeHandle n;

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	mov_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

	heading_set_p = n.advertise<geometry_msgs::Twist>("/SET_HEADING", 10);
	heading_set_t = n.advertise<geometry_msgs::Twist>("/TURN_HEADING", 10);
	
	set_d=n.advertise<std_msgs::Float64>("/SET_DIST",10);
	nav_zz=n.advertise<std_msgs::Int32>("/START",10);
	STOP=n.advertise<std_msgs::Int32>("/STOP",10);
	starting = n.advertise<std_msgs::Int8>("/starting", 10);
    PAUSE = n.advertise<std_msgs::Int32>("/PAUSE",10);

    ros::Publisher reset = n.advertise<std_msgs::Int8>("/RESET",10);
	rigorous = n.advertise<std_msgs::Int8>("/rigor",10);
	ros::Publisher pose_starting = n.advertise<std_msgs::Int8>("/pose_starting", 10);

	ros::Subscriber sensing_data = n.subscribe<geometry_msgs::Vector3>("/sensing_stopping",10,&dataCB);

	ros::Rate loop_rate(50);
	pose_starter.data = 0;
	while (ros::ok())
	{
		c=getch();
		if(total_ss == 0.0 or rigor == 1)
		{
			if(c=='d' || c==67 || c=='6')
			{
				right_turn();
			}
			else if(c=='a' || c==68 || c=='4')
			{
				left_turn();
			}
			else if(c=='w' || c==65 || c=='5')
			{
				straight();
			}
			else if(c=='s' || c=='2' || c==9)
			{
				backward();
			}
			mov_cmd.publish(cmd);
		}
		else if(total_ss == 3.0)
		{
			if(c=='s' || c=='2' || c==9)
			{
				backward();
			}
			else if(c=='d' || c==67 || c=='6')
			{
				ROS_WARN("Teleop : Right pressed but obstacle is present");
			}
			else if(c=='a' || c==68 || c=='4')
			{
				ROS_WARN("Teleop : Left pressed but obstacle is present");
			}
			else if(c=='w' || c==65 || c=='5')
			{
				ROS_WARN("Teleop : Straight pressed but obstacle is present");
			}
			mov_cmd.publish(cmd);
		}
		else if(total_ss == 1.0)
		{
			if(stopper_front == 1.0)
			{
				if(c=='d' || c==67 || c=='6')
				{
					right_turn();
				}
				else if(c=='a' || c==68 || c=='4')
				{
					left_turn();
				}
				else if(c=='s' || c=='2' || c==9)
				{
					backward();
				}
				else if(c=='w' || c==65 || c=='5')
				{
					ROS_WARN("Teleop : Straight pressed but obstacle is present");
				}
			}
			else if(stopper_right == 1.0)
			{
				if(c=='a' || c==68 || c=='4')
				{
					left_turn();
				}
				else if(c=='w' || c==65 || c=='5')
				{
					straight();
				}
				else if(c=='s' || c=='2' || c==9)
				{
					backward();
				}
				else if(c=='d' || c==67 || c=='6')
				{
					ROS_WARN("Teleop : Right pressed but obstacle is present");
				}
			}
			else if(stopper_left == 1.0)
			{
				if(c=='d' || c==67 || c=='6')
				{
					right_turn();
				}
				else if(c=='w' || c==65 || c=='5')
				{
					straight();
				}
				else if(c=='s' || c=='2' || c==9)
				{
					backward();
				}
				else if(c=='a' || c==68 || c=='4')
				{
					ROS_WARN("Teleop : Left pressed but obstacle is present");
				}
			}
			mov_cmd.publish(cmd);
		}
		else if(total_ss == 2.0)
		{
			if(stopper_front == 1.0 and stopper_right == 1.0)
			{
				if(c=='a' || c==68 || c=='4')
				{
					left_turn();
				}
				else if(c=='s' || c=='2' || c==9)
				{
					backward();
				}
				else if(c=='d' || c==67 || c=='6')
				{
					ROS_WARN("Teleop : Right pressed but obstacle is present");
				}
				else if(c=='w' || c==65 || c=='5')
				{
					ROS_WARN("Teleop : Straight pressed but obstacle is present");
				}
			}
			else if(stopper_right == 1.0 and stopper_left == 1.0)
			{
				if(c=='w' || c==65 || c=='5')
				{
					straight();
				}
				else if(c=='s' || c=='2' || c==9)
				{
					backward();
				}
				else if(c=='d' || c==67 || c=='6')
				{
					ROS_WARN("Teleop : Right pressed but obstacle is present");
				}
				else if(c=='a' || c==68 || c=='4')
				{
					ROS_WARN("Teleop : Left pressed but obstacle is present");
				}
			}
			else if(stopper_left == 1.0 and stopper_front == 1.0)
			{
				if(c=='d' || c==67 || c=='6')
				{
					right_turn();
				}
				else if(c=='s' || c=='2' || c==9)
				{
					backward();
				}
				else if(c=='a' || c==68 || c=='4')
				{
					ROS_WARN("Teleop : Left pressed but obstacle is present");
				}
				else if(c=='w' || c==65 || c=='5')
				{
					ROS_WARN("Teleop : Straight pressed but obstacle is present");
				}
			}
			mov_cmd.publish(cmd);
		}
		if (c=='t')
		{
			if(rigor == 1)
			{
				rigor = 0;
				ros::param::set("/ss_enable", 0);
				ROS_INFO("Teleop : Sense stop enabled");
			}
			else
			{
				rigor = 1;
				ros::param::set("/ss_enable", 1);
				ROS_WARN("Teleop : Sense stop disabled");
			}
		}
		else if(c=='f')
		{
			ros::param::get("/LEFT_SET",LEFT_SET);
			ros::param::get("/RIGHT_SET",RIGHT_SET);
			ros::param::set("/TURN_TYPE",1);
			ROS_DEBUG("Navigate L_TYPE ZZ: %f, %f",LEFT_SET, RIGHT_SET);
			STRT.data = 0;
			nav_zz.publish(STRT);
			STP.data = 0;
			STOP.publish(STP);
		}
		else if(c=='g')
		{
			ros::param::get("/LEFT_SET",LEFT_SET);
			ros::param::get("/RIGHT_SET",RIGHT_SET);
			ros::param::set("/TURN_TYPE",2);
			ROS_DEBUG("Navigate R_TYPE ZZ: %f, %f",LEFT_SET, RIGHT_SET);
			STRT.data = 0;
			nav_zz.publish(STRT);
			STP.data = 0;
			STOP.publish(STP);

			rig.data = 77;
			rigorous.publish(rig);//imp

			auto_nav = 1;
		}
		else if (c=='v')
		{
			ROS_DEBUG("Pause Navigation");
			PAU.data=0;
			PAUSE.publish(PAU);
		}
		else if (c=='b')
		{
			ROS_DEBUG("Resume Navigation");
			PAU.data=1;
			PAUSE.publish(PAU);
		}

		else if (c=='q'|| c=='+' || c==66)
		{
			ROS_INFO("Teleop : Stop key pressed");
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
			wall_step = 0;

			STRT.data = 1;
			nav_zz.publish(STRT);
			STP.data = 1;
			STOP.publish(STP);

			rig.data = 110;
			rigorous.publish(rig);//imp

			auto_nav = 0;

			pose_starter.data = 0;//remove later
			pose_starting.publish(pose_starter);

			ros::param::set("/sensing_straight", 0);
		}
		else if(c=='u')
		{
			ROS_INFO("Teleop : Start LWF first lane");
			starter.data = 10;
			starting.publish(starter);
			ros::param::set("/ws_setpoint",0.55); //first lane
			ros::param::set("/new_command",1);
			wall_step = 1;
		}
		else if(c=='i')
		{
			ROS_INFO("Teleop : Start LWF second lane");
			starter.data = 10;
			starting.publish(starter);
			ros::param::set("/ws_setpoint",0.80); //second lane
			ros::param::set("/new_command",1);
			wall_step = 1;
		}
		else if(c=='o')
		{
			ROS_INFO("Teleop : Start RWF first lane");
			starter.data = 20;
			starting.publish(starter);
			// ros::param::set("/ws_setpoint",0.55); //first lane
			// ros::param::set("/new_command",1);
			wall_step = 2;
		}
		else if(c=='p')
		{
			ROS_INFO("Teleop : Start RWF second lane");
			starter.data = 20;
			starting.publish(starter);
			// ros::param::set("/ws_setpoint",0.80); //first lane
			// ros::param::set("/new_command",1);
			wall_step = 2;
		}
		else if(c=='j')
		{
			ROS_INFO("Teleop : Start LSF first lane");
			starter.data = 30;
			starting.publish(starter);
			// ros::param::set("/ws_setpoint",0.55); //first lane
			// ros::param::set("/new_command",1);
			wall_step = 3;
		}
		else if(c=='n')
		{
			ROS_INFO(" Teleop : Start LSF second lane");
			starter.data = 30;
			starting.publish(starter);
			// ros::param::set("/ws_setpoint",0.80); //second lane
			// ros::param::set("/new_command",1);
			wall_step = 3;
		}
		else if(c=='k')
		{
			ROS_INFO("Teleop : Start RSF first lane");
			starter.data = 40;
			starting.publish(starter);
			ros::param::set("/ws_setpoint",0.600); //first lane
			ros::param::set("/new_command",1);
			wall_step = 4;
		}
		else if(c=='m')
		{
			ROS_INFO("Teleop : Start RSF second lane");
			starter.data = 40;
			starting.publish(starter);
			ros::param::set("/ws_setpoint",0.80); //second lane
			ros::param::set("/new_command",1);
			wall_step = 4;
		}
		else if(c=='l')
		{
			if(pose_starter.data == 0)
			{
				pose_starter.data = 1;
				ROS_INFO("Teleop : Pose node started");
			}
			else
			{
				pose_starter.data = 0;
				ROS_INFO("Teleop : Pose node stopped");
			}
			pose_starting.publish(pose_starter);
		}
		else if(c=='z')
		{
			starter.data = 1;
			starting.publish(starter);	//lsf stop

			ros::param::set("/sensing_straight", 0);

			rig.data = 77;
			rigorous.publish(rig);//imp

			pose_starter.data = 1;
			pose_starting.publish(pose_starter);

			ROS_INFO("Z pressed");
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

	return 0;
}
