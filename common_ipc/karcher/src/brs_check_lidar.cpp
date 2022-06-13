#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <ros/console.h>
#include <stdlib.h>

#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std;

ros::Publisher start;

int i = 0, z=0;
int flat = 0, slant = 0;
int done = 0;

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	int count = scan->scan_time / scan->time_increment;

    if(i>400)
    {
        for(int i = 0; i < count; i++)
        {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            if(degree >= -0.2 and degree <= 0.2 and scan->intensities[i]!=0)
            {
                if(scan->ranges[i] < 1.5)
                {
                    flat = 1;
                }
            }
        }
    }
}
void laser_scan_callback2(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	int count = scan->scan_time / scan->time_increment;

    if(i>400)
    {
        for(int i = 0; i < count; i++)
        {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            if(degree >= -0.2 and degree <= 0.2 and scan->intensities[i]!=0)
            {
                if(scan->ranges[i] < 1.5)
                {
                    slant = 1;
                }
            }
        }
    }
}

int main(int argc, char **argv) 
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
	ros::init(argc, argv, "lidar_check");
	ros::NodeHandle n;

	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 2, laser_scan_callback);
    ros::Subscriber laser_sub2 = n.subscribe<sensor_msgs::LaserScan>("/new_scan_far", 2, laser_scan_callback2);
	
    start = n.advertise<std_msgs::Int8>("/lidar_check", 2);
    ros::Rate rate(40);
    
    std_msgs::Int8 st;
	while (ros::ok()) 
	{
        ros::spinOnce();
        i++;
        z++;

        if(i>800 and done == 0)
        {
            if(flat == 1 and slant == 1)
            {
                ROS_ERROR("Remove front obstacle for lidar detection node");
                i = 0;
                z = 0;
            }
            else if(flat == 1 and slant == 0)
            {
                ROS_INFO("Changing Lidar port, wait for 10 seconds");
                st.data = 1;
                start.publish(st);
                i = 0;
            }
            else if(flat == 0 and slant == 1)
            {
                ROS_INFO("Slant Lidar correctly placed");
				ROS_DEBUG("You may START operation now");
                z = 2400;
            
                st.data = 2;
                start.publish(st);
            }
            else if(flat == 0 and slant == 0)
            {
                ROS_ERROR("Unable to detect floor from either of the lidars");
				ROS_DEBUG("You may START operation now");
                z = 2400;
                st.data = 2;
                start.publish(st);
            }
			flat = 0;
			slant = 0;
        }
        if(z>=2400)
        {
			ROS_INFO(" Shutting down lidar check cpp node");
			ros::shutdown();
        }
		rate.sleep();
	}

	ros::spin();
	return 0;
}