#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <fstream>
#include <iostream>
#include <ctime>
#include <ros/console.h>
#include <stdlib.h>
#include "timing_cal.h"

using namespace std;
std::ofstream myfile("/home/bros/motor_current.csv",std::ios::out | std::ios::app); //change this accordingly

double ref_end, ref_start;

float right_c = 0.0, left_c = 0.0;
int left_c_ref = 0, ref_safe = 0;

void left_current(const std_msgs::Float64::ConstPtr& msg)
{
    left_c = msg->data;
    left_c_ref = 1;
}
void right_current(const std_msgs::Float64::ConstPtr& msg2)
{
    right_c = msg2->data;
}
int main(int argc,char **argv)
{
	if(!myfile)
	{
		std::cout<<"open file failure"<<std::endl;
	}

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    myfile<<std::endl;
    myfile<<"New file start \n";

    time_t now = time(0);
    char* dt = ctime(&now);

    string t = "";
    for(int z=11; z<19; z++)
    {
        t = t + dt[z];
    }
    string m = converter(t);
    int u=0;
    for(int k=11; k<19; k++)
    {
        dt[k]=m[u];
        u++;
    }
    myfile<<"Motor Current file start : \t"<<dt;
    myfile<<std::endl;
    
	myfile<<" Time \t Left Motor \t Right Motor \n";

    ROS_INFO("motor_current \t Time \t Left Motor \t Right Motor \n");
	ros::init(argc,argv,"motor_current");
	ros::NodeHandle n;

    ros::Subscriber get_cull=n.subscribe<std_msgs::Float64>("/L_AMP",10,&left_current);
	ros::Subscriber get_curr=n.subscribe<std_msgs::Float64>("/R_AMP",10,&right_current);

    ros::Rate loop_rate(20);

	while (ros::ok()) 
	{
        ros::spinOnce();
        if(ref_safe == 0)
        {
            ref_start = ros::Time::now().toSec();
            ref_safe = 1;
        }
        if(ref_safe == 1)
        {
            ref_end = ros::Time::now().toSec();
            if((ref_end - ref_start >= 0.5) and left_c_ref == 1)
            {
                time_t now = time(0);
				char* dt = ctime(&now);
                string t = "";
				for(int z=11; z<19; z++)
				{
					t = t + dt[z];
				}

                string m = converter(t);
                
                myfile<<m<<"\t"<<left_c<<"\t"<<right_c;
                myfile<<std::endl;

                ROS_INFO("motor_current \t %s \t %f \t %f", m.c_str(), left_c, right_c);

                left_c_ref = 0;
                ref_safe = 0;
            }
        }
		loop_rate.sleep();
	}
    if(ros::isShuttingDown())
	{
		time_t now = time(0);
		char* dt = ctime(&now);

        string t = "";
        for(int z=11; z<19; z++)
        {
            t = t + dt[z];
        }

        string m = converter(t);
        int u=0;
        for(int k=11; k<19; k++)
        {
            dt[k]=m[u];
            u++;
        }

		myfile<<std::endl;
		myfile<<"Current monitor end Time : \t"<<dt;
		myfile<<std::endl;
	}
	ros::spin();
	return 0;
}