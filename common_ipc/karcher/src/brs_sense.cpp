/*
Change stopping Distance ->
rosparam set /new_stop_flat 1 && rosparam set /stop_dist_flat 0.80
rosparam set /new_stop_slant 1 && rosparam set /stop_dist_slant 0.50
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <sstream>
#include "std_msgs/String.h"
#include <ros/console.h>
#include <stdlib.h>
#include "debug_sensor.h"

#define RAD2DEG(x) ((x)*180./M_PI)

int stopped_1 = 0,stopped_3 = 0, stopped_4 = 0, starter = 0, rigor = 0, stopped = 0, motion_halt=0;
int  stopped_22 = 0, stopped_33 = 0, stopped_44 = 0; 
float range_1 = 0, range_2 = 0;

int z = 0, front_ss = 0, left_ss = 0, right_ss = 0;

int new_stop_slant = 0, par_slant = 0, new_stop_flat = 0, par_flat = 0;
float stopping_dist_flat_strt = 1.0, stopping_dist_slant_strt = 0.80,stopping_dist_flat_left=0.6,stopping_dist_flat_right=0.6,stopping_dist_slant_left=0.8,stopping_dist_slant_right=0.8;

int ss_en = 0;
float linear=0.25;
//float linear_turn=0.0285, angular = 0.15;
float linear_turn = 0.0471, angular = 0.2691;

float ws_degree, prev_ws_degree;

float LEFT_SET = 0.0, RIGHT_SET = 0.0;

int lo = 0, ko = 0, mo = 0, once_zz = 0, k=0, allowed_flat = 0, allowed_slant = 0;

using namespace std;
ros::Publisher lidar_pub;
void start_callback(const std_msgs::Int8::ConstPtr& strt)
{
	starter = strt->data;
}

void rigor_callback(const std_msgs::Int8::ConstPtr& rig)
{
	rigor = rig->data;
}

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)  //Upper lidar
{
	int count = scan->scan_time / scan->time_increment;

    int max_degree = int(RAD2DEG(scan->angle_min + (count * scan->angle_increment)));
    int min_degree = RAD2DEG(scan->angle_min);

    stopped_1 = 0;stopped_3=0;stopped_4=0;

    ros::param::get("/new_stop_flat", new_stop_flat);
	
	
    if(new_stop_flat == 1)
    {
        ros::param::get("/stop_dist_flat_strt", stopping_dist_flat_strt);
		ros::param::get("/stop_dist_flat_left", stopping_dist_flat_left);
	    ros::param::get("/stop_dist_flat_right", stopping_dist_flat_right);
        par_flat++;
        if(par_flat>= 20)
        {
            ros::param::set("/new_stop_flat", 0);
            par_flat = 0;
            new_stop_flat = 0;
        }
    }
    stringstream aa;
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

   /* for(int i = (count-1); i >= 0; i--)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(degree>=-180.0 and degree <= 180.0)
        {
            if(degree < 0.0)
            {
                ws_degree = -int(degree);
            }
            else
            {
                ws_degree = int(360 - degree);
            }
            if(lo == 0)
            {
                lo = 1;
                //aa << "robodata|lidar|";
            
                // int a = 180 + min_degree;
                // for(int b = 0; b < a; b++)
                // {
                //     //aa<< "0.00, ";
                //     aa<< "0, ";
                //     ko++;
                // }
                int a = 180 - max_degree;
                for(int b = 0; b<a; b++)
                {
                    //aa<< "0.00, ";
                    aa<< "0, ";
                    ko++;
                }
            }
            //if(int(degree) <= (max_degree - 3))
            if(int(degree) >= (min_degree + 3))
            {
                if(ws_degree != prev_ws_degree)
                {
                    if(scan->intensities[i]!=0)
                    {
                        //aa << setprecision(2) << scan->ranges[i]<< ", ";
                        aa << int(scan->ranges[i] * 10)<< ", ";
                        ko++;
                    }
                    else
                    {
                        //aa << "6.0, ";//change this to 0.0 if required for infinite value of that particular degree
                        aa << "60, ";
                        ko++;
                    }
                }
            }
            else
            {
                if(mo == 0)
                {
                    // int a = 180 - max_degree;
                    // for(int b = 0.0; b<=a; b++)
                    // {
                    //     if(b<a)
                    //     {
                    //         //aa<< "0.00, ";
                    //         aa<< "0, ";
                    //         ko++;
                    //     }
                    //     else
                    //     {
                    //         //aa<< "0.00";
                    //         aa<< "0";
                    //         ko++;
                    //     }   
                    // }
                    int a = 180 + min_degree;
                    for(int b = 0; b <= a; b++)
                    {
                        //aa<< "0.00, ";
                        if(b<a)
                        {
                            aa<< "0, ";
                            ko++;
                        }
                        else
                        {
                            aa<< "0";
                            ko++;
                        }    
                    }
                    mo = 1;
                }
            }
            prev_ws_degree = ws_degree;
            // "robdata|lidar|dist1, dist2, dist3 , ....... , dist360"
        }*/

    //--------------------------------------------------------------------------------------------------
      
         if(degree >= -30.0 and degree <= 30.0 and scan->ranges[i]>0.1 and scan->intensities[i]!=0)
         {
             range_1 = scan->ranges[i];
             if(range_1 <= stopping_dist_flat_strt)
             {
                 ROS_WARN("flat sensing stopping : Stopped Straight");                 
                 stopped_1++;
             }
         }
         else if(degree >= -90.0 and degree < -30.0 and scan->ranges[i]>0.1 and scan->intensities[i]!=0)
         {
             range_1 = scan->ranges[i];
             if(range_1 <= stopping_dist_flat_right)
             {
                 ROS_WARN("flat sensing stopping : Stopped right turn ");
                 stopped_3++;
             } 
         }
         else if(degree > 30.0 and degree <= 90.0 and scan->ranges[i]>0.1 and scan->intensities[i]!=0)
         {
             range_1 = scan->ranges[i];
             if(range_1 <= stopping_dist_flat_left)
             {
                 ROS_WARN("flat sensing stopping : Stopped left turn ");
                 stopped_4++;
             }
         }
    }
   /* lo = 0;
    ko = 0;
    mo = 0;
    std_msgs::String msg;
    msg.data = aa.str();

    lidar_pub.publish(msg);*/

}
void laser_scan_callback2(const sensor_msgs::LaserScan::ConstPtr& scan2)  //lower lidar
{
	int count2 = scan2->scan_time / scan2->time_increment;

    stopped_22 =0;stopped_33=0;stopped_44=0;

    ros::param::get("/new_stop_slant", new_stop_slant);
    if(new_stop_slant == 1)
    {
        ros::param::get("/stop_dist_slant_strt", stopping_dist_slant_strt);
	   	ros::param::get("/stop_dist_slant_left", stopping_dist_slant_left);
	    ros::param::get("/stop_dist_slant_right", stopping_dist_slant_right);
        par_slant++;
        if(par_slant>= 20)
        {
            ros::param::set("/new_stop_slant", 0);
            par_slant = 0;
            new_stop_slant = 0;
        }
    }

    for(int i = 0; i < count2; i++)
    {
        float degree2 = RAD2DEG(scan2->angle_min + scan2->angle_increment * i);
        if(degree2 >= -35.0 and degree2 <= 35.0 and scan2->ranges[i]>0.1 and scan2->intensities[i]!=0)
        {
            range_2 = scan2->ranges[i];
            if(range_2 <= stopping_dist_slant_strt)
            {
                ROS_WARN("slant sensing stopping : Stopped Straight turn ");
                stopped_22++;
            }
        }
        else if(degree2 >= -90.0 and degree2 < -35.0 and scan2->ranges[i]>0.1 and scan2->intensities[i]!=0)
        {
            range_2 = scan2->ranges[i];
            if(range_2 <= stopping_dist_slant_right)
            {
                ROS_WARN("slant sensing stopping : Stopped right turn ");
                stopped_33++;
            }
        }
        else if(degree2 > 35.0 and degree2 <= 90.0 and scan2->ranges[i]>0.1 and scan2->intensities[i]!=0)
        {
            range_2 = scan2->ranges[i];
            if(range_2 <= stopping_dist_slant_left)
            {
                ROS_WARN("slant sensing stopping : Stopped left turn ");
                stopped_44++;
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
	ros::init(argc, argv, "brs_sense");
	ros::NodeHandle n;

    N::debugging_next obj_sense_flat;
    N::debugging_next obj_sense_slant;

	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 2, laser_scan_callback);
    ros::Subscriber laser_sub2 = n.subscribe<sensor_msgs::LaserScan>("/new_scan_far", 2, laser_scan_callback2);
    ros::Publisher stopping = n.advertise<geometry_msgs::Vector3>("/sensing_stopping", 50);
	ros::Publisher wall_stopping=n.advertise<geometry_msgs::Vector3>("/wall_sensing",10);

//for zigzag---------------------------------------------------------------
    ros::Publisher PAUSE = n.advertise<std_msgs::Int32>("/PAUSE",10);
    ros::Publisher heading_set_p = n.advertise<geometry_msgs::Twist>("/SET_HEADING", 10);
	ros::Publisher heading_set_t = n.advertise<geometry_msgs::Twist>("/TURN_HEADING", 10);
	
	ros::Publisher set_d=n.advertise<std_msgs::Float64>("/SET_DIST",10);
	ros::Publisher nav_zz=n.advertise<std_msgs::Int32>("/START",10);
	ros::Publisher STOP=n.advertise<std_msgs::Int32>("/STOP",10);
//end for zigzag---------------------------------------------------------------

    ros::Subscriber start_sub = n.subscribe<std_msgs::Int8>("/starting", 10, start_callback);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

    ros::Subscriber rigorous = n.subscribe<std_msgs::Int8>("/rigor",10, rigor_callback);

    lidar_pub = n.advertise<std_msgs::String>("/lidar_stream", 1);

	ros::Rate rate(40);

	geometry_msgs::Vector3 msg, msg_wall;
    geometry_msgs::Twist cmd;

    std_msgs::Int32 PAU, STRT, STP;
    std_msgs::Float64 dist;
    geometry_msgs::Twist heading, heading_turn;
	while (ros::ok()) 
	{
        ros::spinOnce();
        if((stopped_1 >= 1) or (stopped_22 >= 1))
        {
            msg.y = 1.0;
            z++;
            front_ss++;
            msg_wall.y = 1.0;
        }
        else
        {
            msg.y = 0.0;
            msg_wall.y = 0.0;
        }
        if( (stopped_3>=1) or (stopped_33 >= 1))
        {
            msg.z = 1.0;
            z++;
            right_ss++;
            msg_wall.z = 1.0;
        }
        else
        {
            msg.z = 0.0;
            msg_wall.z = 0.0;
        }
        
        if((stopped_4 >= 1) or (stopped_44 >= 1))
        {
            msg.x = 1.0;
            z++;
            left_ss++;
            msg_wall.x = 1.0;
        }
        else
        {
            msg.x = 0.0;
            msg_wall.x = 0.0;
        }

        stopping.publish(msg);

        ros::param::get("/ss_enable", ss_en);

        if(ss_en == 1)
        {
            msg_wall.x = 0.0;
            msg_wall.z = 0.0;
        }
        wall_stopping.publish(msg_wall);

	if(ss_en==0)
	{
		if((left_ss>=1||right_ss>=1||front_ss>=1) && motion_halt<3 && rigor!=40)
		{
			cmd.linear.x=0.0;
			cmd.angular.z=0.0;
			cmd_pub.publish(cmd);
			motion_halt +=1;
			ROS_WARN("Object detected!! (L:%d, F:%d, R:%d)",left_ss,front_ss,right_ss);
		}
		if (rigor==10||rigor==20||rigor==30||rigor==40||rigor==110)
		{
			motion_halt=0;
		}
	}
        if(z>=1 and starter <= 5 and ss_en == 0)
        {
            if(left_ss>=1 and rigor == 10)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub.publish(cmd);
                if(k%40 == 0)
                {
                    ROS_WARN("sensing stopping : Stopped left turn ");
                    k=1;
                }
                k++;
                stopped = 1;
            }
            else if(right_ss>=1 and rigor == 20)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub.publish(cmd);
                if(k%40 == 0)
                {
                    ROS_WARN("sensing stopping : Stopped right turn ");
                    k=1;
                }
                k++;
                stopped = 1;
            }
            else if(front_ss>=1 and rigor == 30)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub.publish(cmd);
                if(k%40 == 0)
                {
                    ROS_WARN("sensing stopping : Stopped STRAIGHT ");
                    k=1;
                }
                k++;
                stopped = 1;
            }
            else if(rigor == 77 and once_zz == 0)
            {
                if(k%40 == 0)
                {
                    ROS_WARN("sensing stopping : Stopped ZZ ");
                    k=1;
                }
                k++;
                PAU.data=0;
		        PAUSE.publish(PAU);

                dist.data = 0.0;
                set_d.publish(dist);

                heading_turn.linear.x = 0.0;	//comment for normal opreation (without RPM control)
                heading_turn.angular.z = 0.0;	//comment for normal opreation (without RPM control)
                heading_set_t.publish(heading_turn);	//comment for normal opreation (without RPM control)

                heading.linear.x = 0.0;	//comment for normal opreation (without RPM control)
                heading.angular.z = 0.0;	//comment for normal opreation (without RPM control)
                heading_set_p.publish(heading);	//comment for normal opreation (without RPM control)

                STRT.data = 1;
                nav_zz.publish(STRT);
                STP.data = 1;
                STOP.publish(STP);

                once_zz = 1;
                stopped = 1;
                ros::param::set("/sensing_straight", 1);
            }
        }
        if(stopped == 1 and starter <= 5)
        {
            if(rigor == 10 and left_ss == 0)
            {
                cmd.linear.x = linear_turn;
                cmd.angular.z = angular;
                cmd_pub.publish(cmd);
                ROS_DEBUG("sensing stopping : RESUMED left turn ");
                stopped = 0;
                k=0;
		motion_halt=0;
            }
            else if(rigor == 20 and right_ss == 0)
            {
                cmd.linear.x = linear_turn;
                cmd.angular.z = -angular;
                cmd_pub.publish(cmd);
                ROS_DEBUG("sensing stopping : RESUMED right turn ");
                stopped = 0;
                k=0;
		motion_halt=0;
            }
            else if(rigor == 30 and front_ss == 0)
            {
                cmd.linear.x = linear;
                cmd.angular.z = 0.0;
                cmd_pub.publish(cmd);
                ROS_DEBUG("sensing stopping : RESUMED straight ");
                stopped = 0;
                k=0;
		motion_halt=0;
            }
            else if(rigor == 100)
            {
                cmd.linear.x = -linear;
                cmd.angular.z = 0.0;
                cmd_pub.publish(cmd);
                ROS_DEBUG("sensing stopping : Going REVERSE after stopping");
                stopped = 0;
                k=0;
		motion_halt=0;
            }
            else if(rigor == 110)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub.publish(cmd);
                ROS_DEBUG("sensing stopping : STOPPED with key press");
                stopped = 0;
                k=0;
		motion_halt=0;
            }
            else if(rigor == 77 and once_zz == 1 and z==0)
            {
                ros::param::set("/sensing_straight", 0);
                ROS_DEBUG("sensing stopping : RESUMED ZZ");
                PAU.data=1;
		        PAUSE.publish(PAU);

                ros::param::get("/LEFT_SET",LEFT_SET);
                ros::param::get("/RIGHT_SET",RIGHT_SET);
                ros::param::set("/TURN_TYPE",2);

                STRT.data = 0;
                nav_zz.publish(STRT);
                STP.data = 0;
                STOP.publish(STP);
                once_zz = 0;
                stopped = 0;
                k=0;
            }
        }

        z = 0; front_ss = 0; left_ss = 0; right_ss = 0;
        
        allowed_flat = obj_sense_flat.sensor_status(1.0, 0, 0, 1, 0, 0);
        allowed_slant = obj_sense_slant.sensor_status(1.0, 0, 0, 0, 1, 0);

        if(allowed_flat == 0)
        {
            stopped_1 = 0;stopped_3=0;stopped_4=0;
        }
        if(allowed_slant == 0)
        {
            stopped_22 =0;stopped_33=0;stopped_44=0;
        }
		rate.sleep();
	}

	ros::spin();
	return 0;
}
