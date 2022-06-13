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
#include <iostream>


float d = 0.0, stopper_front = 0.0, stopper_right = 0.0, stopper_left = 0.0, total_ss;
std_msgs::Float64 dist;
std_msgs::Int32 STRT,STP,PAU;

int front_ss = 0, rigor =0, enabler =0, left_ss = 0, right_ss = 0;

geometry_msgs::Twist cmd,setpoint,heading,heading_turn;
std_msgs::Int8 dir,res,starter, rig, pose_starter, pose_starter_str, alarm_msg;
ros::Publisher mov_cmd;
ros::Publisher dir_cmd;
ros::Publisher set_p,heading_set_p,heading_set_t,set_d,nav_zz,STOP,starting,PAUSE;

ros::Publisher rigorous, alarm_pub;

int wall_step = 0, auto_nav = 0, tim = 0;

float S=0.3;
float thresh2=0.3, thresh1=0.057; //teleop_twist
//float thresh2=0.15, thresh1=0.0285; //teleop_twist

char prev_c;

float DIST = 0.0;
float LEFT_SET = 0.0, RIGHT_SET = 0.0;  // Left wheel & right wheel RPM setpoint 
float LEFT_SET_H = 0.0, RIGHT_SET_H = 0.0;      // Left wheel & right wheel RPM setpoint 
float LEFT_SET_T = 0.0, RIGHT_SET_T = 0.0;      // Left wheel & right wheel RPM setpoint 

void dataCB(const geometry_msgs::Vector3::ConstPtr& dat)
{
        stopper_left = dat->x;
        stopper_front = dat->y;
        stopper_right = dat->z;

        total_ss = stopper_front + stopper_right + stopper_left;

        ROS_DEBUG("stopper_front in callback is %f", stopper_front);
        ROS_DEBUG("stopper_left in callback is %f", stopper_left);
        ROS_DEBUG("stopper_right in callback is %f", stopper_right);
}
void restoring_now(struct termios *sinom)
{
        tcsetattr(0, TCSANOW, sinom);
}

void setting_now(struct termios *sinom)
{
    struct termios ajar;
    tcgetattr(0,sinom);

    ajar = *sinom;

        ajar.c_lflag &= ~ICANON;
    ajar.c_lflag &= ~ECHO;
    ajar.c_lflag &= ~ISIG;
    ajar.c_cc[VMIN] = 0;
    ajar.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &ajar);
}

void right_turn()
{
	ROS_INFO("Teleop : Right");
	cmd.linear.x=thresh1;
	cmd.angular.z=-thresh2;

	rig.data = 20;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
}
void left_turn()
{
	ROS_INFO("Teleop : Left");
	cmd.linear.x=thresh1;
	cmd.angular.z=thresh2;

	rig.data = 10;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
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

    mov_cmd.publish(cmd);
}
void backward()
{
	ROS_INFO("Teleop : BackWard");
	cmd.linear.x=-S;
	cmd.angular.z=d;

	rig.data = 100;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "brs_teleop");
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
        ros::Publisher pose_starting_str = n.advertise<std_msgs::Int8>("/pose_starting_str", 10);
        ros::Subscriber sensing_data = n.subscribe<geometry_msgs::Vector3>("/sensing_stopping",10,&dataCB);

        alarm_pub = n.advertise<std_msgs::Int8>("/alarm", 10);

        ros::Rate loop_rate(50);
        pose_starter.data = 0;
        pose_starter_str.data = 0;

        
	while (ros::ok())
	{
		struct termios current_settings;
		char c = 0;

		setting_now(&current_settings);

		c = getchar();
               int a = int(c);
        
	if(c>0 and c!= -1)
        {
            if(total_ss == 0.0 or rigor == 1)
            {
                if(c=='d' || c=='6' || a == 67)
                {
                    right_turn();
                }
                else if(c=='a' || c=='4' || a==68)
                {
                    left_turn();
                }
                else if(c=='w' || c=='5' || a==65)
                {
                    straight();
                    
                    if (enabler == 1)
                    {
                       ros::param::set("/ss_enable", 0);
                       ROS_WARN("Teleop : Sense stop enabled");
                       enabler = 0;
                    }

                }
                else if(c=='s' || c=='2')
                {
                    backward();
                }
            }
            
            else if(c=='s' || c=='2')
            {
                backward();
            }
            
            else if(c=='w' || c=='5' || a==65)
            {
                if(stopper_front == 1.0)
                {
                    ROS_WARN("Teleop : Straight pressed but obstacle is present");
                    cmd.linear.x=0.0;
                    cmd.angular.z=0.0;
                }
                else
                {
                    straight();
                }
            }
            else if(c=='d' || c=='6' || a == 67)
            {
                if(stopper_right == 1.0)
                {
                    ROS_WARN("Teleop : Right pressed but obstacle is present");
                    cmd.linear.x=0.0;
                    cmd.angular.z=0.0;
                    mov_cmd.publish(cmd);
                }
                else
                {
                    right_turn();
                }
            }
            else if(c=='a' || c=='4' || a==68)
            {
                if(stopper_left == 1.0)
                {
                    ROS_WARN("Teleop : Left pressed but obstacle is present");
                    cmd.linear.x=0.0;
                    cmd.angular.z=0.0;
                    mov_cmd.publish(cmd);
                }
                else
                {
                    left_turn();
                }
            }
            if (c=='r')
            {
                enabler = 0;
                rigor = 0;
                ros::param::set("/ss_enable", 0);
                ROS_INFO("Teleop : Sense stop enabled");
            }
            
            else if(c=='e')
            {
                enabler = 1;
                rigor = 1;
                ros::param::set("/ss_enable", 1);
                ROS_WARN("Teleop : Sense stop disabled");
            }
            
            else if (c=='b') //Alarm sound ON
            {
                ROS_INFO("Teleop : Alarm ON");
                alarm_msg.data = 1;
                alarm_pub.publish(alarm_msg);
            }
            else if(c=='h') //Alarm sound OFF
            {
                ROS_INFO("Teleop : Alarm Off");
                alarm_msg.data = 0;
                alarm_pub.publish(alarm_msg);
            }
            else if (c=='q'|| c=='+' || a==66)
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

                STRT.data = 1;
                nav_zz.publish(STRT);
                STP.data = 1;
                STOP.publish(STP);

                rig.data = 110;
                rigorous.publish(rig);//imp

                auto_nav = 0;

                pose_starter_str.data = 0;//remove later
                pose_starting_str.publish(pose_starter_str);

                ros::param::set("/sensing_straight", 0);
            }
            else if(c=='g')
            {
                wall_step = 1;
                ROS_INFO(" Mode switched to Wall following");
            }
            else if(c=='G')
            {
                wall_step = 2;
                ROS_INFO(" Mode switched to Step following");
            }

            if(wall_step == 1)
            {
                if(c=='u')
                {
                    ROS_INFO("Teleop : Start LWF first lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",0.55); //first lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='i')
                {
                    ROS_INFO("Teleop : Start LWF second lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.0); //second lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='o')
                {
                    ROS_INFO("Teleop : Start LWF third lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='p')
                {
                    ROS_INFO("Teleop : Start LWF fourth lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",2.0); //second lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c==',')
                {
                    ROS_INFO("Teleop : Start LWF fifth lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",2.5); //first lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='.')
                {
                    ROS_INFO("Teleop : Start LWF sixth lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",3.0); //second lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='-')
                {
                    ROS_INFO("Teleop : Start LWF seventh lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",3.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='f')
                {
                    ROS_INFO("Teleop : Start LWF eighth lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",4.0); //second lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='n')
                {
                    ROS_INFO("Teleop : Start LWF ninth lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",4.5); //first lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c==']')
                {
                    ROS_INFO("Teleop : Start LWF tenth lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",5.0); //second lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c==';')
                {
                    ROS_INFO("Teleop : Start LWF eleventh lane");
                    starter.data = 10;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",5.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='U')
                {
                    ROS_INFO("Teleop : Start RWF first lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",0.55); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='I')
                {
                    ROS_INFO("Teleop : Start RWF second lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.0); //second lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='O')
                {
                    ROS_INFO("Teleop : Start RWF third lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='P')
                {
                    ROS_INFO("Teleop : Start RWF fourth lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",2.0); //second lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='<')
                {
                    ROS_INFO("Teleop : Start RWF fifth lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",2.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='>')
                {
                    ROS_INFO("Teleop : Start RWF sixth lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",3.0); //second lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='_')
                {
                    ROS_INFO("Teleop : Start RWF seventh lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",3.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='F')
                {
                    ROS_INFO("Teleop : Start RWF eighth lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",4.0); //second lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='H')
                {
                    ROS_INFO("Teleop : Start RWF ninth lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",4.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='}')
                {
                    ROS_INFO("Teleop : Start RWF tenth lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",5.0); //second lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c==':')
                {
                    ROS_INFO("Teleop : Start RWF eleventh lane");
                    starter.data = 20;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",5.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='t')
                {
                    ROS_INFO("Short wall for Wall following mode enabled");
                    ros::param::set("/allow_shortwall", 1);
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='T')
                {
                    ROS_INFO("Short wall for Wall following mode Disabled");
                    ros::param::set("/allow_shortwall", 0);
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
            }
            else if(wall_step == 2)
            {
                if(c=='u')
                {
                    ROS_INFO("Teleop : Start LSF first lane");
                    starter.data = 30;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",0.45); //first lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='i')
                {
                    ROS_INFO("Teleop : Start LSF second lane");
                    starter.data = 30;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.0); //second lane
                    ros::param::set("/new_comman",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='o')
                {
                    ROS_INFO("Teleop : Start LSF third lane");
                    starter.data = 30;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.50); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='p')
                {
                    ROS_INFO("Teleop : Start LSF fourth lane");
                    starter.data = 30;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",2.0); //second lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='U')
                {
                    ROS_INFO("Teleop : Start RSF first lane");
                    starter.data = 40;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",0.45); //first lane
                    ros::param::set("/new_command",1);

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='I')
                {
                    ROS_INFO("Teleop : Start RSF second lane");
                    starter.data = 40;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.0); //second lane
                    ros::param::set("/new_command",1);  

                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='O')
                {
                    ROS_INFO("Teleop : Start RSF third lane");
                    starter.data = 40;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",1.5); //first lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='P')
                {
                    ROS_INFO("Teleop : Start RSF fourth lane");
                    starter.data = 40;
                    starting.publish(starter);
                    ros::param::set("/ws_setpoint",2.0); //second lane
                    ros::param::set("/new_command",1);
                    
                    rig.data = 109;
                    rigorous.publish(rig);//imp
                }
                else if(c=='v')
                {
                    ros::param::set("/new_command",1);
                    ros::param::set("/obstacle_avoidance", 1); //this disables obstacle avoidance
                    ROS_INFO("Obstacle avoidance disabled");
                }
                else if(c=='V')
                {
                    ros::param::set("/new_command",1);
                    ros::param::set("/obstacle_avoidance", 0); //this enables obstacle avoidance
                    ROS_INFO("Obstacle avoidance Enabled");
                }
            }
            if(c=='j')
            {
                ROS_INFO("Teleop : P to P set at 0 ");
                ros::param::set("/p_to_p",0.0); //second lane
                ros::param::set("/new_command",1);
                
                rig.data = 109;
                rigorous.publish(rig);//imp
            }
            else if(c=='J')
            {
                ROS_INFO("Teleop : P to P set at 11.0 feet ");
                ros::param::set("/p_to_p",11.0); //second lane
                ros::param::set("/new_command",1);
                
                rig.data = 109;
                rigorous.publish(rig);//imp
            }
            else if(c=='l')
            {
                
                pose_starter.data = 1;
                ROS_INFO("Teleop : Pose node started");
                pose_starting.publish(pose_starter);
            }
            else if(c=='L')
            {
                pose_starter.data = 0;
                ROS_INFO("Teleop : Pose node stopped");
                pose_starting.publish(pose_starter);
            }
            else if(c=='z')
            {
                starter.data = 1;
                starting.publish(starter);	//lsf stop

                ros::param::set("/sensing_straight", 0);

                rig.data = 77;
                rigorous.publish(rig);//imp

                pose_starter_str.data = 1;
                pose_starting_str.publish(pose_starter_str);

                ROS_INFO("Z pressed");
            }
            prev_c=c;
            c = -1;
        }
		
        restoring_now(&current_settings);

		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();

	return 0;
}

