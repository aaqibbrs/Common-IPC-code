#include "std_msgs/Header.h"
#include <fstream>
#include <iostream>
#include<cmath>
#include<math.h>
#include"ros/ros.h"
#include"sensor_msgs/LaserScan.h"
#include"std_msgs/String.h"
#include"std_msgs/Int8.h"
#include "debug_sensor.h"
#include <ros/console.h>

#include <vector> 
#include <string> 
#include <algorithm> 
#include <sstream> 
#include <iterator> 

#define PI 3.14159265
#define RAD2DEG(x) ((x)*180./M_PI)

int once = 0;

int first = 0, mov = 0, mov_slant = 0;
float length_to_ground[200],theta[200];
float lidar_height = 0.72, final_map = 0.0;

int lid = 0;

std::vector<float> above_, slant_;

float rounding(float variab)//rounding to 3 decimal places
{
    float val = (int)(variab * 1000 + .5);

    if(int(val)%10 == 0)
    {
        val = val + 1;
    }
    return (float)val / 1000;
}

template<typename T>

inline int map_data(T x, T in_min, T in_max, T out_min, T out_max)
{
  return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void lidarCallback(const std_msgs::Int8::ConstPtr& lidar)
{
    lid = lidar->data;
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;

    if(mov == 0 and lid == 2)
    {
        for(int i = 0; i < count; i++)
        {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            if(degree >= -89.0 and degree <= 89.0)
            {
                int h = degree + 89;
                if(scan->ranges[i]>0.0 and scan->ranges[i]<=5.0 and scan->intensities[i]!=0)
                {
                    float x_up = scan->ranges[i]*sin(degree*PI/180);
                    float y_up = scan->ranges[i]*cos(degree*PI/180);

                    double dis = sqrtf(x_up*x_up + y_up*y_up);
                    if(dis<2.5)//red to yellow
                    {
                        float loc_x;
                        if(x_up < 0.00)
                        {
                            loc_x = -rounding(x_up/10.0 - 255.0);
                        }
                        else
                        {
                            loc_x = -rounding(x_up/10.0 + 255.0);
                        }

                        float loc_y = rounding(y_up/10.0 + map_data(dis, 0.10, 2.5, 0.0, 255.0));
                        above_.push_back(loc_x);
                        above_.push_back(loc_y);
                    }
                    else//yellow to green
                    {
                        float loc_x;
                        if(x_up < 0.00)
                        {
                            loc_x = -rounding(x_up/10.0 - map_data(dis, 2.5, 5.0, 255.0, 0.0));
                        }
                        else
                        {
                            loc_x = -rounding(x_up/10.0 + map_data(dis, 2.5, 5.0, 255.0, 0.0));
                        }

                        float loc_y = rounding(y_up/10.0 + 255.0);//green constant
                        above_.push_back(loc_x);
                        above_.push_back(loc_y);
                    }
                }
            }
            if(degree>=90.0)
            {
                mov=1;
                break;
            }
        }
    }
}
void newscanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;

    if(mov_slant == 0 and lid == 2)
    {
        for(int i = 0; i < count; i++)
        {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

            float centre;
            int k_new;

            if(first == 0)
            {
                if(degree >= -0.01 and degree <= 0.01)
                {
                    centre = scan->ranges[i] + 0.035;	
                    first = 1;
                }
                if(first == 1) 
                {
                    for(int k=0; k<=89;k++)
                    {
                        float height_;
                        height_ = lidar_height + (k * 0.00024); //4.4 cm total height so, 0 to 90 is 2.2 cm, so for 1 degree = 2.2/90 cm
                        length_to_ground[k] = centre/cos((k) * PI/180) - 0.035; //which is equal to 0.00024m
                        theta[k] = acos((height_/length_to_ground[k]));
                        ROS_INFO("length = %f and theta = %f",length_to_ground[k], theta[k]);
                    }
                    first = 2;
                    ROS_INFO(" Centre value is %f", centre);
                }
            }
            if(first == 2)
            {
                if(degree >= -89.0 and degree <= 89.0)
                {
                    int degree_k;
                    if(degree< 0.0)
                    {
                        degree_k = -int(degree);
                    }
                    else
                    {
                        degree_k = int(degree);
                    }

                    float relation = 0.85 * length_to_ground[degree_k];

                    if(scan->ranges[i]>0.0 and scan->ranges[i]<=5.0 and scan->intensities[i]!=0 and scan->ranges[i] <= relation)
                    {
                        final_map = scan->ranges[i] * sin(theta[degree_k]);

                        float x_up = final_map*sin(degree*PI/180);
                        float y_up = final_map*cos(degree*PI/180);

                        ROS_DEBUG("x_up is : %f and y_up is : %f", x_up, y_up);

                        double dis = sqrtf(x_up*x_up + y_up*y_up);
                        if(dis<=2.5)
                        {
                            float loc_x;
                            if(x_up < 0.00)
                            {
                                loc_x = -rounding(x_up/10.0 - 255.0);
                            }
                            else
                            {
                                loc_x = -rounding(x_up/10.0 + 255.0);
                            }
                            
                            float loc_y = rounding(y_up/10.0 + map_data(dis, 0.10, 2.5, 0.0, 255.0));
                            slant_.push_back(loc_x);
                            slant_.push_back(loc_y);
                        }
                        else
                        {
                            float loc_x;
                            if(x_up < 0.00)
                            {
                                loc_x = -rounding(x_up/10.0 - map_data(dis, 2.5, 5.0, 255.0, 0.0));
                            }
                            else
                            {
                                loc_x = -rounding(x_up/10.0 + map_data(dis, 2.5, 5.0, 255.0, 0.0));
                            }

                            float loc_y = rounding(y_up/10.0 + 255.0);
                            slant_.push_back(loc_x);
                            slant_.push_back(loc_y);
                        }
                    }
                }
            }
            if(degree >= 90.0)
            {
                mov_slant = 1;
                break;
            }
        }
    }
}

int main(int argc,char **argv)
{
	ros::init(argc,	argv, "lidar_vis_wss");
	ros::NodeHandle	nh;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    N::debugging_next lidar_top, lidar_slan;

    ros::Subscriber laser_flat = nh.subscribe<sensor_msgs::LaserScan>("/scan", 2, &scanCallback);
    ros::Subscriber laser_slant = nh.subscribe<sensor_msgs::LaserScan>("/new_scan_far", 2, &newscanCallback);
    ros::Subscriber lidar_check = nh.subscribe<std_msgs::Int8>("/lidar_check", 2, &lidarCallback);

    ros::Publisher pub_lidar = nh.advertise<std_msgs::String>("/both_lidar", 2);

    std_msgs::String msg;

	ros::Rate loop_rate(20);
	while (ros::ok())
	{
        ros::spinOnce();
        int allowed_top = lidar_top.sensor_status(1.0, 0, 0, 1, 0, 0);
        int allowed_slant = lidar_slan.sensor_status(1.0, 0, 0, 0, 1, 0);

        if(once==0)
        {
            ros::param::get("/lidar_height",lidar_height);
            once = 1;
        }
        
        if(allowed_top == 1 and allowed_slant == 1)
        {
            if(mov == 1 and mov_slant == 1)
            {
                std::ostringstream vts;

                if(!above_.empty())
                {
                    std::copy(above_.begin(), above_.end(),
                        std::ostream_iterator<float>(vts, ", "));
                }
                if(!slant_.empty()) 
                {
                    std::copy(slant_.begin(), slant_.end()-1,
                        std::ostream_iterator<float>(vts, ", "));	
                
                
                    vts << slant_.back();
                }
                msg.data = vts.str();
                
                pub_lidar.publish(msg);
                above_.clear();
                slant_.clear();

                mov = 0;
                mov_slant = 0;
            }
        }
        else if(allowed_top == 1 and allowed_slant == 0)
        {
            if(mov == 1)
            {
                std::ostringstream vts;

                if(!above_.empty())
                {
                    std::copy(above_.begin(), above_.end()-1,
                        std::ostream_iterator<float>(vts, ", "));
                
                    vts << above_.back();
                }
                msg.data = vts.str();
                
                pub_lidar.publish(msg);
                above_.clear();
                slant_.clear();

                mov = 0;
                mov_slant = 0;
            }
        }
        else if(allowed_top == 0 and allowed_slant == 1)
        {
            if(mov_slant == 1)
            {
                std::ostringstream vts;

                if (!slant_.empty()) 
                {
                    std::copy(slant_.begin(), slant_.end()-1,
                        std::ostream_iterator<float>(vts, ", "));
                
                vts << slant_.back();

                }
                msg.data = vts.str();
                
                pub_lidar.publish(msg);
                above_.clear();
                slant_.clear();

                mov = 0;
                mov_slant = 0;
            }
        }

		loop_rate.sleep();
	}
    ros::spin();
	return 0;
}