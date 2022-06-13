#include"ros/ros.h"
#include"geometry_msgs/Vector3.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <stdlib.h>
#include "timing_cal.h"
#include <ctime>

using namespace std;

void pose_callback(const geometry_msgs::Twist::ConstPtr& p_start)
{
	float p_data = p_start->angular.y;
    time_t now = time(0);
    char* dt = ctime(&now);

    string t = "";
    for(int z=11; z<19; z++)
    {
        t = t + dt[z];
    }
    string m = converter(t);

    ROS_INFO("kangan \t \t \t \t \t \t Sub \t %s \t %d", m.c_str(), int(p_data));
}

void pose_callback_without_ekf(const geometry_msgs::Twist::ConstPtr& p_start_without_ekf)
{
    float x = p_start_without_ekf->linear.x;
    float y = p_start_without_ekf->linear.y;

    ROS_INFO("odom_ \t without_ekf \t %f \t %f",x,y);
}

void pose_callback_ekf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p_start_ekf)
{
    float x = p_start_ekf->pose.pose.position.x;
    float y = p_start_ekf->pose.pose.position.y;

    ROS_INFO("odom_ \t \t \t \t with_ekf \t %f \t %f",x,y);
}    
int main(int argc,char **argv)
{
	ros::init(argc,argv,"brs_checking");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
	ros::NodeHandle n;
	ros::Subscriber cmd_pub = n.subscribe<geometry_msgs::Twist>("/pose_dir", 1, &pose_callback);

    ros::Subscriber pose_without_ekf = n.subscribe<geometry_msgs::Twist>("/pose_dir_without_ekf", 1, &pose_callback_without_ekf);
    ros::Subscriber pose_ekf = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_ekf/odom_combined", 1, &pose_callback_ekf);
	ros::Rate loop_rate(40);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}