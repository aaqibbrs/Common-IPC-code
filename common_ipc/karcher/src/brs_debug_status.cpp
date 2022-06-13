#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <iostream>
#include <ctime>
#include <ros/console.h>
#include <stdlib.h>
#include "timing_cal.h"

using namespace std;
std::ofstream myfile("/home/bros/debug_sensors.csv",std::ios::out | std::ios::app); //change this accordingly

int starter = 0, rigor = 0;
int wall_ref = 0, rigor_ref = 0, laser_flat_ref = 0;
int laser_slant_ref = 0, left_enc_ref = 0, right_enc_ref = 0, imu_ref =0;

int enc = 0, las_flat = 0, las_slant = 0, im = 0, prev_a=5500;

int ref_safe = 0, i =0, s=0, n_s = 0, s_n_s = 0, en = 0;
double ref_end, ref_start;

void start_callback(const std_msgs::Int8::ConstPtr& strt)
{
	starter = strt->data;
    wall_ref = 1;
}
void rigor_callback(const std_msgs::Int8::ConstPtr& rig)
{
	rigor = rig->data;
    rigor_ref = 1;
}
void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
    laser_flat_ref = 1;
}
void dataCB(const geometry_msgs::Vector3::ConstPtr& dat)
{
    imu_ref = 1;
}
void laser_scan_callback2(const sensor_msgs::LaserScan::ConstPtr& scan2) 
{
    laser_slant_ref = 1;
}
void leftcallback(const std_msgs::Int64::ConstPtr& msg)
{
    left_enc_ref = 1;
}
void rightcallback(const std_msgs::Int64::ConstPtr& msg2)
{
    right_enc_ref = 1;
}
int main(int argc,char **argv)
{
	if(!myfile)
	{
		std::cout<<"open file failure"<<std::endl;
	}

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
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
    myfile<<"Debug start : \t"<<dt;
    myfile<<std::endl;
    
	myfile<<" Time \t Encoders \t Scan Lidar \t New Scan Lidar \t IMU \n";

    ROS_INFO("debugging info \t Time \t Encoders \t Scan Lidar \t New Scan Lidar \t IMU");
	ros::init(argc,argv,"brs_debug_status");
	ros::NodeHandle n;

    ros::Subscriber laser_sub_slant = n.subscribe<sensor_msgs::LaserScan>("/new_scan_far", 50, laser_scan_callback2);
    ros::Subscriber laser_sub_flat = n.subscribe<sensor_msgs::LaserScan>("/scan", 50, laser_scan_callback);
	ros::Subscriber gyro_data = n.subscribe<geometry_msgs::Vector3>("/brick_imu",50,&dataCB);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",50,&leftcallback);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",50,&rightcallback);
    ros::Subscriber start_sub = n.subscribe<std_msgs::Int8>("/starting", 10, &start_callback);
    ros::Subscriber rigor_sub = n.subscribe<std_msgs::Int8>("/rigor", 10, &rigor_callback);

    ros::Rate loop_rate(20);

	geometry_msgs::Twist msg1;
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
            if(ref_end - ref_start >= 1.000)
            {
                if(left_enc_ref == 0 or right_enc_ref == 0)
                {
                    if(enc==1)
                    {
                        ROS_ERROR("Encoder Stopped");
                        enc = 0;
                    }
                    else if(en==0 and enc==0)
                    {
                        ROS_ERROR("Waiting for Encoder");
                        im = 0;
                    }
                    else if(en%40 == 0 and enc==0)
                    {
                        ROS_ERROR("Encoders have not yet started");
                        enc = 0;
                        en=11;
                    }
                    en++;
                }
                else
                {
                    if(enc == 0)
                    {
                        ROS_DEBUG("Encoder Started");
                        enc = 1;
                    }
                }
                if(laser_flat_ref == 0 and laser_slant_ref == 0)
                {
                    if(s_n_s == 0)
                    {
                        ROS_ERROR("Waiting for Both Lidar");
                        las_flat = 0;
                        las_slant = 0;
                    }
                    else if(s_n_s%40 == 0)
                    {
                        ROS_ERROR("Both Lidar have still NOT started");
                        las_flat = 0;
                        las_slant = 0;
                        s_n_s = 11;
                    }
                    s_n_s++;
                }
                else
                {
                    if(laser_flat_ref == 0)
                    {
                        if(las_flat == 1)
                        {
                            ROS_WARN("/scan Lidar Stopped");
                            las_flat = 0;
                        }
                        else if(s==0 and las_flat == 0)
                        {
                            ROS_WARN("Waiting for /scan Lidar");
                            las_flat = 0;
                        }
                        else if(s%40 ==0 and las_flat==0)
                        {
                            ROS_WARN("/scan Lidar is still not working");
                            las_flat = 0;
                            s= 11;
                        }
                        s++;
                    }
                    else
                    {
                        if(las_flat == 0)
                        {
                            ROS_DEBUG("/scan Lidar Started");
                            las_flat = 1;
                        }
                    }
                    if(laser_slant_ref == 0)
                    {
                        if(las_slant == 1)
                        {
                            ROS_WARN("/new_scan_far Lidar Stopped");  
                            las_slant = 0;
                        }
                        else if(n_s == 0 and las_slant == 0)
                        {
                            ROS_WARN("Waiting for /new_scan_far");  
                            las_slant = 0;
                        }
                        else if(n_s%40 == 0 and las_slant == 0)
                        {
                            ROS_WARN("/new_scan_far Lidar is still not working");  
                            las_slant = 0;
                            n_s = 11;
                        }
                        n_s++;
                    }
                    else
                    {
                        if(las_slant == 0)
                        {
                            ROS_DEBUG("/new_scan_far Lidar Started");
                            las_slant = 1;
                        }
                    }
                }

                if(imu_ref == 0)
                {
                    if(im==1)
                    {
                        ROS_ERROR("IMU Stopped");
                        im = 0;
                    }
                    else if(i==0 and im==0)
                    {
                        ROS_ERROR("Waiting for IMU");
                        im = 0;
                    }
                    else if(i%40 == 0 and im==0)
                    {
                        ROS_ERROR("IMU has not yet started");
                        im = 0;
                        i=11;
                    }
                    i++;
                }
                else
                {
                    if(im == 0)
                    {
                        ROS_DEBUG("IMU Started");
                        im = 1;
                    }
                }

                time_t now = time(0);
				char* dt = ctime(&now);
                string t = "";
				for(int z=11; z<19; z++)
				{
					t = t + dt[z];
				}

                string m = converter(t);

                stringstream ss;
                ss<<enc<<las_flat<<las_slant<<im;
                int a;
                ss>>a;
                if(prev_a != a)
                {
                    myfile<<m<<"\t"<<enc<<"\t"<<las_flat<<"\t"<<las_slant<<"\t"<<im;
                    myfile<<std::endl;

                    ROS_INFO("debugging info \t %s \t %d \t %d \t %d \t %d", m.c_str(), enc, las_flat, las_slant, im);
                }

                left_enc_ref = 0;
                right_enc_ref = 0;
                laser_slant_ref = 0;
                laser_flat_ref = 0;
                imu_ref = 0;

                ref_safe = 0;
                prev_a = a;
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
		myfile<<"Debug End Time : \t"<<dt;
		myfile<<std::endl;
	}
	ros::spin();
	return 0;
}