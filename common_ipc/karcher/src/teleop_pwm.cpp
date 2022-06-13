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

float stopper_front = 0.0, stopper_right = 0.0, stopper_left = 0.0, total_ss;

int front_ss = 0, rigor =0, enabler =0, left_ss = 0, right_ss = 0;

ros::Publisher mov_cmd;
ros::Publisher dir_pub;

ros::Publisher rigorous, starting;

geometry_msgs::Twist cmd;
std_msgs::Int8 v, rig, starter;

int wss_command = 50;

int d = 0, wall_step = 0;

float thresh2=155, thresh1=168; //teleop_twist

char prev_c;

float DIST = 0.0;
float LEFT_SET = 0.0, RIGHT_SET = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_H = 0.0, RIGHT_SET_H = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_T = 0.0, RIGHT_SET_T = 0.0;	// Left wheel & right wheel RPM setpoint 

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
	cmd.angular.z=d;

    //cmd.linear.x=0.06280;
	//cmd.angular.z=-0.358873;

    v.data = 10;
    dir_pub.publish(v);

	rig.data = 20;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
}
void left_turn()
{
	ROS_INFO("Teleop : Left");
	cmd.linear.x=d;
	cmd.angular.z=thresh2;

    v.data = 30;
    dir_pub.publish(v);

	rig.data = 10;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
}
void slight_left_turn()
{
	ROS_INFO("Teleop : Slight Left");
	cmd.linear.x=60;
	cmd.angular.z=125;

    v.data = 30;
    dir_pub.publish(v);

	rig.data = 10;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
}
void slight_right_turn()
{
	ROS_INFO("Teleop : Slight Right");
	cmd.linear.x=125;
	cmd.angular.z=60;

    v.data = 30;
    dir_pub.publish(v);

	rig.data = 20;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
}
void straight()
{
	ros::param::get("/PWM_L",thresh1);
	ros::param::get("/PWM_R",thresh2);
	ros::param::get("/LEFT_SET",LEFT_SET);
	ros::param::get("/RIGHT_SET",RIGHT_SET);
	ROS_INFO("Teleop : Forward");
	cmd.linear.x=thresh1;
	cmd.angular.z=thresh2;

    v.data = 30;
    dir_pub.publish(v);

	rig.data = 30;
	rigorous.publish(rig);//imp//

    mov_cmd.publish(cmd);
}
void backward()
{
	ROS_INFO("Teleop : BackWard");
	cmd.linear.x=-thresh1;
	cmd.angular.z=-thresh2;

    v.data = 40;
    dir_pub.publish(v);

	rig.data = 100;
	rigorous.publish(rig);//imp

    mov_cmd.publish(cmd);
}
void teleop_wss(const std_msgs::Int8::ConstPtr& dat)
{
	wss_command = dat->data;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "brs_teleop");
	ros::NodeHandle n;

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	rigorous = n.advertise<std_msgs::Int8>("/rigor",10);
	ros::Subscriber sensing_data = n.subscribe<geometry_msgs::Vector3>("/sensing_stopping",10,&dataCB);
    ros::Subscriber teleop_wss_ = n.subscribe<std_msgs::Int8>("/wss_teleop",10,&teleop_wss);

    starting = n.advertise<std_msgs::Int8>("/starting", 10);

    mov_cmd = n.advertise<geometry_msgs::Twist>("/mov_cmd", 10);
    dir_pub = n.advertise<std_msgs::Int8>("/dir_cmd", 10);

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		struct termios current_settings;
		char c = 0;

		setting_now(&current_settings);

		c = getchar();
        int a = int(c);
        if(wss_command == 50)
        {
            c = 'q';
            wss_command = 0;
        }
        else if(wss_command == 40)
        {
            c = 'a';
            wss_command = 0;
        }
        else if(wss_command == 60)
        {
            c = 'd';
            wss_command = 0;
        }
        else if(wss_command == 80)
        {
            c = 'w';
            wss_command = 0;
        }
        else if(wss_command == 20)
        {
            c = 's';
            wss_command = 0;
        }
        else if(wss_command == 21)
        {
            wall_step = 1;
            c = 'u';
            wss_command = 0;
        }
        else if(wss_command == 22)
        {
            wall_step = 1;
            c = 'i';
            wss_command = 0;
        }
        else if(wss_command == 23)
        {
            wall_step = 1;
            c = 'o';
            wss_command = 0;
        }
        else if(wss_command == 24)
        {
            wall_step = 1;
            c = 'p';
            wss_command = 0;
        }
        else if(wss_command == 25)
        {
            wall_step = 1;
            c = ',';
            wss_command = 0;
        }
        else if(wss_command == 26)
        {
            wall_step = 1;
            c = '.';
            wss_command = 0;
        }
        else if(wss_command == 27)
        {
            wall_step = 1;
            c = '-';
            wss_command = 0;
        }
        else if(wss_command == 28)
        {
            wall_step = 1;
            c = 'f';
            wss_command = 0;
        }
        else if(wss_command == 29)
        {
            wall_step = 1;
            c = 'n';
            wss_command = 0;
        }
        else if(wss_command == 30)
        {
            wall_step = 1;
            c = ']';
            wss_command = 0;
        }
        else if(wss_command == 31)
        {
            wall_step = 1;
            c = ';';
            wss_command = 0;
        }
        else if(wss_command == 61)
        {
            wall_step = 1;
            c = 'U';
            wss_command = 0;
        }
        else if(wss_command == 62)
        {
            wall_step = 1;
            c = 'I';
            wss_command = 0;
        }
        else if(wss_command == 63)
        {
            wall_step = 1;
            c = 'O';
            wss_command = 0;
        }
        else if(wss_command == 64)
        {
            wall_step = 1;
            c = 'P';
            wss_command = 0;
        }
        else if(wss_command == 65)
        {
            wall_step = 1;
            c = '<';
            wss_command = 0;
        }
        else if(wss_command == 66)
        {
            wall_step = 1;
            c = '>';
            wss_command = 0;
        }
        else if(wss_command == 67)
        {
            wall_step = 1;
            c = '_';
            wss_command = 0;
        }
        else if(wss_command == 68)
        {
            wall_step = 1;
            c = 'F';
            wss_command = 0;
        }
        else if(wss_command == 69)
        {
            wall_step = 1;
            c = 'N';
            wss_command = 0;
        }
        else if(wss_command == 70)
        {
            wall_step = 1;
            c = '}';
            wss_command = 0;
        }
        else if(wss_command == 71)
        {
            wall_step = 1;
            c = ':';
            wss_command = 0;
        }
        else if(wss_command == 41)
        {
            wall_step = 2;
            c = 'u';
            wss_command = 0;
        }
        else if(wss_command == 42)
        {
            wall_step = 2;
            c = 'i';
            wss_command = 0;
        }
        else if(wss_command == 43)
        {
            wall_step = 2;
            c = 'o';
            wss_command = 0;
        }
        else if(wss_command == 44)
        {
            wall_step = 2;
            c = 'p';
            wss_command = 0;
        }
        else if(wss_command == 46)
        {
            wall_step = 2;
            c = 'U';
            wss_command = 0;
        }
        else if(wss_command == 47)
        {
            wall_step = 2;
            c = 'I';
            wss_command = 0;
        }
        else if(wss_command == 48)
        {
            wall_step = 2;
            c = 'O';
            wss_command = 0;
        }
        else if(wss_command == 49)
        {
            wall_step = 2;
            c = 'P';
            wss_command = 0;
        }
        else if(wss_command == 11)
        {
            c = 'V';
            wss_command = 0;
        }
        else if(wss_command == 12)
        {
            c = 'v';
            wss_command = 0;
        }
        else if(wss_command == 13)
        {
            c = 't';
            wss_command = 0;
        }
        else if(wss_command == 14)
        {
            c = 'T';
            wss_command = 0;
        }
        else if(wss_command == 15)
        {
            c = 'r';
            wss_command = 0;
        }
        else if(wss_command == 16)
        {
            c = 'e';
            wss_command = 0;
        }
        else if(wss_command == 98)
        {
            c = 'j';
            wss_command = 0;
        }
        else if(wss_command == 99)
        {
            c = 'J';
            wss_command = 0;
        }
		if(c>0 and c!= -1)
        {
            if(total_ss == 0.0 or rigor == 1)
            {
                if(c=='c')
                {
                    slight_right_turn();
                }
                else if(c=='z')
                {
                    slight_left_turn();
                }
                else if(c=='d' || c=='6' || a == 67)
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
                    mov_cmd.publish(cmd);
                    v.data = 40;
                }
                else
                {
                    straight();
                }
            }
            else if(c=='d' || c=='6' || a == 67 || c=='c')
            {
                if(stopper_right == 1.0)
                {
                    ROS_WARN("Teleop : Right pressed but obstacle is present");
                    cmd.linear.x=0.0;
                    cmd.angular.z=0.0;
                    mov_cmd.publish(cmd);

                    v.data = 50;
                    dir_pub.publish(v);
                }
                else
                {
                    right_turn();
                }
            }
            else if(c=='a' || c=='4' || a==68 || c=='z')
            {
                if(stopper_left == 1.0)
                {
                    ROS_WARN("Teleop : Left pressed but obstacle is present");
                    cmd.linear.x=0.0;
                    cmd.angular.z=0.0;
                    mov_cmd.publish(cmd);

                    v.data = 50;
                    dir_pub.publish(v);
                }
                else
                {
                    left_turn();
                }
            }
            if (c=='r')
            {
                rigor = 0;
                ros::param::set("/ss_enable", 0);
                ROS_INFO("Teleop : Sense stop enabled");
            }
            else if(c=='e')
            {
                rigor = 1;
                ros::param::set("/ss_enable", 1);
                ROS_WARN("Teleop : Sense stop disabled");
            }
            else if (c=='q'|| c=='+' || a==66)
            {
                ROS_INFO("Teleop : Stop key pressed");
                cmd.linear.x=d;
                cmd.angular.z=d;
                mov_cmd.publish(cmd);

                v.data = 50;
                dir_pub.publish(v);

                rig.data = 110;
                rigorous.publish(rig);//imp

                starter.data = 0;
                starting.publish(starter);	//lsf stop

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
                else if(c=='N')
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
                    ros::param::set("/new_command",1);

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
                ROS_INFO("Teleop : P to P set at 3.0 feet ");
                ros::param::set("/p_to_p",11.0); //second lane
                ros::param::set("/new_command",1);
                
                rig.data = 109;
                rigorous.publish(rig);//imp
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
