#include"ros/ros.h"
#include"geometry_msgs/Vector3.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <stdlib.h>
#include "timing_cal.h"
#include "debug_sensor.h"

using namespace std;

int starter = 0, allowed = 0;

std::ofstream myfile("/home/bros/node_check.csv",std::ios::out | std::ios::app); //change this accordingly

void start_callback(const std_msgs::Int8::ConstPtr& strt)
{
	starter = strt->data;
}
int main(int argc,char **argv)
{
	if(!myfile)
	{
		std::cout<<"open file failure"<<std::endl;
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

    myfile<<"node check start : \t"<<dt;
    myfile<<std::endl;
    
	myfile<<"Left wall \t Right wall \t Left step \t Right step \n";


    ROS_INFO("Node checking \t Left wall \t Right wall \t Left step \t Right step");

	ros::init(argc,argv,"brs_node_check");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	ros::NodeHandle n;

	N::debugging_next obj_node;

	ros::Subscriber get_pose=n.subscribe<std_msgs::Int8>("/starter",10,&start_callback);

	ros::Rate loop_rate(20);

	while (ros::ok()) 
	{
		if(starter == 10)
        {
            allowed = obj_node.node_status(1.0, 1, 0, 0, 0);
        }
        else if(starter == 20)
        {
            allowed = obj_node.node_status(1.0, 0, 1, 0, 0);
        }
        else if(starter == 30)
        {
            allowed = obj_node.node_status(1.0, 0, 0, 1, 0);
        }
        else if(starter == 40)
        {
            allowed = obj_node.node_status(1.0, 0, 0, 0, 1);
        }
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}