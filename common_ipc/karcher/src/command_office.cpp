#include"ros/ros.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <iostream>
#include<cmath>
#include<math.h>

int  flag_first = 0;

float pose_x, pose_y, changed_direction, actual_direction, setpoint;

double sec_start, sec_end;

void dataCB(const geometry_msgs::Twist::ConstPtr& data)
{
    pose_x = data->linear.x;
    pose_y = data->linear.y;
    changed_direction = data->linear.z;
	actual_direction = data->angular.z;
}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"lino_command_office_leftstep_rightwall");
	ros::NodeHandle n;

	ros::Subscriber pose_data = n.subscribe<geometry_msgs::Twist>("/pose_dir",1,&dataCB);
    ros::Publisher starter = n.advertise<std_msgs::Int8>("/starting", 10);
    ros::Rate loop_rate(50);
    std_msgs::Int8 msg1;
	while (ros::ok()) 
	{
        if(flag_first == 0)
        {
            ros::param::set("/ws_setpoint",0.600); //first lane
            flag_first = 1;
            ros::param::set("/new_command",1);
            ros::param::set("/lidar_height", 0.6200); //change according to height of lidar


            ros::param::set("/safe", 2.0); //step following : should be around 2

            ros::param::set("/turn_dist", 750); //step following : straight before turn encoder count

            ros::param::set("/presence_step", 8); //variable ocunt for step present

            ros::param::set("/presence_wall", 20); //variable ocunt for wall present
            ROS_INFO("This is the first command for setpoint 0.600");
        }
        // if(pose_y >= 60.0 and pose_x>= -10.0 and pose_x <= 10.0 and flag_first == 2) //second lane starting
        // {
        //     flag_first = 3;

        //     ros::param::set("/ws_setpoint",0.7000);

        //     ros::param::set("/new_command",1);

        //     ROS_INFO("This is the second command for setpoint 0.60000 for right wall following");

        //     msg1.data = 0;
        //     starter.publish(msg1);
        //     sec_start = ros::Time::now().toSec();
        // }
        // if(flag_first == 3)
        // {
        //     sec_end =ros::Time::now().toSec();
        //     ROS_INFO("waiting for 3 seconds");
        //     msg1.data = 0;
        //     starter.publish(msg1);
        //     if(sec_end - sec_start >= 3.0000)
        //     {
        //         flag_first = 4;
        //     }
        // }
        // if(flag_first == 4)
        // {
        //     msg1.data = 30;
        //     starter.publish(msg1);

        //     ROS_INFO("final command for starting the right wall following node");

        //     flag_first = 6;
        // }
        // if(pose_y >= 220.0 and pose_x>= -50.0 and flag_first == 6) //lift area coordinates for start
        // {
        //     flag_first = 7;

        //     ROS_INFO("This is the last command");

        //     msg1.data = 0;
        //     starter.publish(msg1);
        // }
        ros::spinOnce();
        loop_rate.sleep();
    }
	ros::spin();
	return 0;
}