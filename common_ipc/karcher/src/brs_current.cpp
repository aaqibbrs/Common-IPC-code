#include"ros/ros.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <iostream>
#include <ctime>
#include <ros/console.h>
#include <stdlib.h>
#include "timing_cal.h"

using namespace std;

std::ofstream myfile("/home/bros/current_data.csv",std::ios::out | std::ios::app); //change this accordingly

ros::Time now_time;

float current_l, current_r;

void l_current_callback(const std_msgs::Float64::ConstPtr& p_start)
{
	current_l = p_start->data;
}
void r_current_callback(const std_msgs::Float64::ConstPtr& p_start)
{
	current_r = p_start->data;
}
int main(int argc,char **argv)
{
	if(!myfile)
	{
		std::cout<<"open file failure"<<std::endl;
	}
	myfile<<std::endl;

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

    myfile<<"New current file start : \t"<<dt;
    myfile<<std::endl;
    
	myfile<<"Time \t Left Motor Current \t Right Motor current \n";

	ros::init(argc,argv,"brs_current_data");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	ros::NodeHandle n;

	ros::Subscriber get_current_l=n.subscribe<std_msgs::Float64>("/L_AMP",50,&l_current_callback);
    ros::Subscriber get_current_r=n.subscribe<std_msgs::Float64>("/R_AMP",50,&r_current_callback);

	ros::Rate loop_rate(20);

    ROS_INFO("current_data \t Time \t Left motor current \t Right motor current");

	while (ros::ok())
	{
        time_t now = time(0);
        char* dt = ctime(&now);

        string t = "";
        for(int z=11; z<19; z++)
        {
            t = t + dt[z];
        }
        string m = converter(t);
        
        if(current_l >= 20.0)
        {
            ROS_FATAL("Current : Left Current data exceeded 20 AMPS");
        }
        if(current_r >= 20.0)
        {
            ROS_FATAL("Current : Right Current data exceeded 20 AMPS");
        }
        myfile<<m<<"\t"<<current_l<<"\t"<<current_r;
        myfile<<std::endl;

        ROS_INFO("current_data \t %s \t %f \t %f", m.c_str(), current_l, current_r);

		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}