/*
FOR TUNING PID ->
rosrosparam set /tuning 1 && rosparam set /KP 0.2 && rosparam set /KI 0.0 && rosparam set /KD 0.1

FOR NEW SETPOINT ->
rosparam set /new_command 1 && rosparam set /ws_setpoint 0.55 && rosparam set /safe 1.5 && rosparam set /turn_dist 750 && rosparam set /presence_step 8

FOR Height ->
rosparam set /lidar_height 0.65
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <stdlib.h>
#include "debug_sensor.h"
#include "tuning_pid.h"

#define PI 3.14159265
#define RAD2DEG(x) ((x)*180./M_PI)
#define CIRCU 0.62831853

float left_enc_ppr = 500.0, right_enc_ppr = 500.0, short_wall = 1.0, turning_radius = 0.0;

float dir_rad; int new_dir = 0;

static float safety_var = 2.0;

float KP = 0.2;
float KI = 0.0;
float KD = 0.1;

int right_now = 0, first_time = 0, right_side = 0, only_twenty = 0;
float time_set, timer_diff, ws_setpoint, lidar_end;

int enc_count = 750, presence = 8, obstacle_avoidance;
double integral_end = 0.0, sw_param;

float lidar_height = 0.7248; //0.6248
float centre = 0.0, setpoint_array[100], stopped, stopped_front;
float lidar_size = 0.035, newdegree=0.0;

int flag1=0, wall_ref = 0, k,i, first = 0, z = 0, hold_twenty = 0; 

float setpoint_new = 0.0, thres = 0.25, degree, setpoint, err=0, p_to_p;

int starter = 0, flag_turn = 0, stepmanual = 0, one_time = 10, dist_pp, done_pp =0, enc_pp;

int lidar_change = 0, new_flag = 0, allowed = 0, dist_next = 0, dist = 0;

int stringent = 0, new_command, second_time = 0, param_var = 0, ppr_d_2 = 0, ratio_enc_count = 0;

long int first_next_left = 0, left_wheel = 0, right_wheel = 0, first_left = 0, first_pp = 0;

ros::Publisher cmd_pub;
geometry_msgs::Twist msg1;

T::tuner rs_tuner;

double sec_start;
double sec_end;

ros::Time timer_start;

using namespace std;
void start_callback(const std_msgs::Int8::ConstPtr& strt)
{
	starter = strt->data;
}
void leftcallback(const std_msgs::Int64::ConstPtr& msg)
{      
	left_wheel = msg->data;
}
void rightcallback(const std_msgs::Int64::ConstPtr& msg2)
{      
	right_wheel = msg2->data;
}
void stopcallback(const geometry_msgs::Vector3::ConstPtr& msg3)
{
	stopped = msg3->x;
	stopped_front = msg3->y;
	if(setpoint>0.70)
	{
		stopped_front = stopped_front;
	}
	else
	{
		stopped_front = 0.0;
	}
}
void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
    if(allowed == 1)
    {
        int count = scan->scan_time / scan->time_increment;

        if(starter == 40)
        {
            int h=0, second_degree, new_val = 0;
            int j=0;
            int error_cal = 0;

            int flag_sf = 0;

            right_side = 0;

            one_time = 0;

            float arr[100] = {0.0}, degree_array[100] = {0.0};

            float second_error, final_error = -50.0;

            ros::param::get("/new_command",new_command);

            if(new_command == 1)
            {
                ros::param::get("/ws_setpoint",ws_setpoint);

                ros::param::set("/new_command", 0);

                ros::param::get("/lidar_height",lidar_height);

                ros::param::get("/safe", safety_var);

                ros::param::get("/turn_dist", enc_count);

                ros::param::get("/presence_step", presence);

                ros::param::get("/left_enc_ppr", left_enc_ppr);

                ros::param::get("/right_enc_ppr", right_enc_ppr);
                
                ros::param::get("/p_to_p", p_to_p);

                ros::param::get("/obstacle_avoidance", obstacle_avoidance);

                ros::param::get("/sw_param", sw_param);

                ros::param::get("/lidar_right_end", lidar_end);

                //setpoint = ws_setpoint + lidar_end;
                setpoint = ws_setpoint;

				enc_pp = int(((p_to_p/3.281)*right_enc_ppr)/CIRCU);
                
                turning_radius = setpoint*0.15*1.2;
				lidar_height = lidar_height + 0.0048;
				new_command = 0;

                ppr_d_2 = int(left_enc_ppr/2.0);
                ratio_enc_count = int(((4/10)*enc_count));

                time_set = 5.0;

                right_now = 0;
                first_time = 0;
                right_side = 0;

                wall_ref = 0;
                lidar_change = 0;
                stringent = 0;
                done_pp = 0;
                new_flag = 0;
                new_dir = 0;
                only_twenty = 0;

                if(second_time == 20)
                {
                    first = 1;
                    ROS_DEBUG("Inside second time = 20");
                }
                else if(second_time == 0)
                {
                    first = 0;
                    ROS_DEBUG("Inside second time = 0");
                }
            }
            for(int i = 0; i < count; i++)
            {
                float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
                if(degree<=-90.0)
                {
                    continue;
                }
                if(degree>=30.0)
                {
                    break;
                }
                if(first == 0)
                {
                    ROS_DEBUG("here it is");
                    if(degree >= -0.001 and degree <= 0.001 and scan->intensities[i]!=0.0)
                    {
                        centre = scan->ranges[i] + lidar_size;	
                        first = 1;
                        second_time = 20;	
                    }
                }
                if(first == 1)
                {
                    float length_to_ground, height, cos_alpha, setpoint_parallel;
                    for(k=10; k<=70;k++)
                    {
                        height = lidar_height + ((k-20) * 0.00024); //4.4 cm total height so, 0 to 90 is 2.2 cm, so for 1 degree = 2.2/90 cm
                        length_to_ground = centre/cos((k) * PI/180) - lidar_size; //which is equal to 0.00024m
                        setpoint_parallel = setpoint/sin((k) * PI/180);
                        height = height * height;
                        length_to_ground = length_to_ground * length_to_ground;
                        cos_alpha = sqrt(1 - (height/length_to_ground));
                        setpoint_array[k] = setpoint_parallel/cos_alpha;
                        ROS_DEBUG(" Degree is %d and Setpoint is %f and height is %f", k, setpoint_array[k], height);
                    }
                    first = 2;
                    short_wall = min((0.8*centre), sw_param);
                    ROS_INFO("RSF : Centre : %f and Short wall : %f", centre, short_wall);
                }
                else
                {
                    if((degree>=-25.0 and degree<=5.0) and scan->ranges[i]>0.1 and scan->intensities[i]!=0) 
                    {
                        if(scan->ranges[i] <=short_wall)
                        {
                            flag_sf = 1;
                        }
                    }
                    if((degree>-30.0 and degree<-10.0) and scan->intensities[i]!=0 and right_now == 1) 
					{
                        if(scan->ranges[i] <=short_wall)
                        {
                            right_side = 1;
                        }
                    }
                    if(degree>=-70.0 and degree<=-10.0 and only_twenty == 0  and ((setpoint_array[-int(degree)]*safety_var) >= scan->ranges[i]))
                    {
                        if(lidar_change == 0)
                        {
                            if((degree>=-70.0 and degree<=-10.0) and scan->intensities[i]!=0) //change maximum according to setpoint. find relation?
                            {
                                if(newdegree!=degree) //to check next float degree obtained from lidar
                                {
                                    arr[j]=scan->ranges[i];

                                    degree_array[j] = degree;

                                    j++;

                                    new_val = 1;
                                }
                                newdegree=degree;		
                            }
                        }
                        else if(lidar_change == 1)
                        {
                            if((degree>=-50.0 and degree<=-10.0) and scan->intensities[i]!=0) //change maximum according to setpoint. find relation?
                            {
                                if(newdegree!=degree) //to check next float degree obtained from lidar
                                {
                                    arr[j]=scan->ranges[i];

                                    degree_array[j] = degree;

                                    j++;

                                    new_val = 1;
                                }
                                newdegree=degree;		
                            }
                        }
                        else if(lidar_change == 2)
                        {
                            if((degree>=-70.0 and degree<=-10.0) and scan->intensities[i]!=0) //change maximum according to setpoint. find relation?
                            {
                                if(newdegree!=degree) //to check next float degree obtained from lidar
                                {
                                    arr[j]=scan->ranges[i];

                                    degree_array[j] = degree;

                                    j++;

                                    new_val = 1;
                                }
                                newdegree=degree;		
                            }
                        }
                    }
                    if(only_twenty == 1)
                    {
                        if((degree>=-30.0 and degree<=-10.0) and scan->intensities[i]!=0 and ((setpoint_array[-int(degree)]*safety_var) >= scan->ranges[i])) //change maximum according to setpoint. find relation?
                        {
                            if(newdegree!=degree) //to check next float degree obtained from lidar
                            {
                                arr[j]=scan->ranges[i];

                                degree_array[j] = degree;

                                j++;

                                new_val = 1;
                            }
                            newdegree=degree;
                        }
                    }
                    if(j >= 2 and new_val == 1)
                    {
                        if(arr[j-1]-arr[j-2] > 0)
                        {
                            second_degree = -int(degree_array[j-2]);
                            
                            second_error = setpoint_array[second_degree] - arr[j-2];

                            if(second_error >= final_error)
                            {
                                final_error = second_error;
                            }

                            ROS_DEBUG("second: [%f] final: [%f]", second_error, final_error);

                            ROS_DEBUG(" degrees : %d ", second_degree);
                            h+=2;
                        }
                        new_val = 0;
                    }
                }
            }
            ROS_DEBUG("next");

            err = final_error;

            ROS_DEBUG("  error :%f and h is [%d]",err, h);
            z++;

            if(z > 3 and first == 2)
            {
                flag1=1;
                if(stopped == 0.0)
                {
                    if(obstacle_avoidance == 0)
                    {
                        if(flag_sf == 1)
                        {
                            if(stringent == 1)
                            {
                                flag_sf = 0;//high prob that during the turn h>=8, so let it roll
                                ROS_DEBUG("yaha aake bahar nikla");
                            }
                            else
                            {
                                if(h>=presence and right_now == 0)
                                {
                                    if(first_time == 0)
                                    {
                                        timer_start = ros::Time::now();
                                        first_time = 1;
                                        ROS_DEBUG("sensing stopping started for %f seconds", time_set);
                                    }
                                    if(first_time == 1)
                                    {
                                        timer_diff = (ros::Time::now() - timer_start).toSec();

                                        if(timer_diff > time_set)
                                        {
                                            first_time = 2;
                                            right_now = 1;
                                            ROS_DEBUG("sensing stopping end, now short wall");
                                            hold_twenty = 0;
                                        }
                                        else
                                        {
                                            flag_turn = 10; //sensing stopping
                                            ROS_DEBUG(" %f seconds over", timer_diff);
                                        }
                                    }
                                }
                                if(right_now == 1)
                                {
                                    //take left turn only
                                    flag_turn = 9;
                                    ROS_WARN_THROTTLE(1, "RSF : Take left turn because obstacle");
                                    new_dir = 0;
                                }
                                else if(h<presence and lidar_change != 2  and right_now == 0)
                                {
                                    flag_turn = 10;//sensing stopping
                                    ROS_WARN_THROTTLE(1, "RSF : Sensing stopping while turn(straight)");
                                }
                                else if(h<presence and lidar_change == 2  and right_now == 0)
                                {
                                    flag_turn = 1; //shortwall turning
                                    wall_ref = 4;
                                    ROS_WARN_THROTTLE(1, "RSF : Turning right short wall (after straight of turn)");
                                }
                            }
                        }
                        if(flag_sf == 0 and right_now == 1)
                        {
                            if(right_side == 1)
                            {
                                //take left turn
                                flag_turn = 9;
                                ROS_INFO_THROTTLE(1, "RSF : Left turn because right side obstacle");
                                only_twenty = 1;
                                hold_twenty = 0;
                                new_dir = 0;
                            }
                            if(h<presence and right_side == 0)
                            {
                                if(first_time == 2)
                                {
                                    timer_start = ros::Time::now();
                                    first_time = 3;
                                    ROS_DEBUG("Timer started for 2.0 seconds");
                                }
                                if(first_time == 3)
                                {
                                    timer_diff = (ros::Time::now() - timer_start).toSec();

                                    if(timer_diff > 2.0F)
                                    {
                                        first_time = 0;
                                        only_twenty = 1;
                                        ROS_INFO(" RSF : 2 second timer over");
                                    }
                                    else
                                    {
                                        ROS_DEBUG(" %f seconds over from two seconds", timer_diff);
                                    }
                                }
                                //take right turn to align
                                flag_turn = 88;
                                ROS_INFO_THROTTLE(1, "RSF : Right alignment code after turn");
                                hold_twenty = 0;
                            }
                            if(h>=presence and right_side == 0)
                            {
                                if(hold_twenty == 0)
                                {
                                    timer_start = ros::Time::now();
                                    hold_twenty = 1;
                                    ROS_DEBUG("Timer started for 2.0 seconds inside hold_twenty");
                                }
                                if(hold_twenty == 1)
                                {
                                    timer_diff = (ros::Time::now() - timer_start).toSec();

                                    if(timer_diff > 2.0F)
                                    {
                                        right_now = 0;
                                        ROS_INFO("RSF : Turn completed");
                                        new_flag = 0;
                                        only_twenty = 0;
                                        hold_twenty = 0;

                                        new_dir = 0;
                                    }
                                    else
                                    {
                                        ROS_DEBUG(" %f seconds over from two seconds hold_twenty", timer_diff);
                                        flag_turn = 10;
                                    }
                                }
                            }
                        }
                        else if(flag_sf == 0 and right_now == 0)
                        {
                            if(h>=presence and stringent == 0 and new_flag == 0)
                            {
                                flag_turn = 0;
                                wall_ref = 0;
                                lidar_change = 0;
                                stringent = 0;

                                right_now = 0;
                                first_time = 0;
                                right_side = 0;
                                only_twenty = 0;
                                ROS_DEBUG("wall following");
                                hold_twenty = 0;
                                new_dir = 0;
                                done_pp = 0;
                            }
                            else if(h>=presence and stringent == 1 and new_flag == 0)
                            {
                                if(dist_next<=ppr_d_2)//200 is for 400ppr, so 250 is for 500 ppr
                                {
                                    flag_turn = 4;
                                    stringent = 1;
                                    wall_ref = 3;
                                    lidar_change = 2;
                                    ROS_WARN_THROTTLE(1, "RSF : step following not started due to stringentt");
                                }
                                else
                                {
                                    stringent = 0;
                                    flag_turn = 0;
                                    wall_ref = 0;
                                    lidar_change = 0;
                                    ROS_WARN_THROTTLE(1, "RSF : step following started from stringent");
                                }
                            }
                            else if(h>=presence and new_flag == 1)
                            {
                                sec_start =ros::Time::now().toSec();
                                flag_turn = 11; //stopping due to unknown object detection
                                new_flag = 2;
                                ROS_WARN_THROTTLE(1, "RSF : Stopping due to unkwon object detection");
                            }
                            else if(new_flag == 2 and done_pp != 1)
                            {
                                sec_end =ros::Time::now().toSec();
                                if(sec_end - sec_start >= 3.0000)
                                {
                                    new_flag = 0;
                                }
                                flag_turn = 11; 
                            }
                            else if(new_flag == 2 and done_pp == 1)
                            {
                                sec_end =ros::Time::now().toSec();
                                if(sec_end - sec_start >= 1.0000)
                                {
                                    new_flag = 0;
                                }
                                flag_turn = 11;
                            }
                            if(h<presence and done_pp != 2)
                            {
                                if(done_pp == 0)
                                {
                                    first_pp = right_wheel;
                                    done_pp = 1;
                                    ROS_INFO("RSF : P to P started with P to P as %f", p_to_p);
                                }
                                if(done_pp == 1)
                                {
                                    dist_pp =  int(right_wheel - first_pp);
                                    if(dist_pp>=enc_pp)
                                    {
                                        done_pp = 2;
                                        flag_turn = 5;
                                        new_flag = 0;
                                        ROS_INFO("RSF : P to P finished");
                                    }
                                    else
                                    {
                                        flag_turn = 55;
                                        ROS_DEBUG_THROTTLE(0.5, "LSF : D PP is %d", dist_pp);
                                        new_flag = 1;
                                    }
                                }
                            }
                            if(h<presence and done_pp == 2)
                            {
                                if(setpoint>0.65)
                                {
                                    flag_turn = 8;
                                    ROS_INFO_THROTTLE(2,"RSF : Inside turn");
                                }
                                else
                                {
                                    if(wall_ref == 0)
                                    {
                                        first_left = left_wheel;
                                        wall_ref = 1;
                                        ROS_INFO("RSF : Sraight before turn Started");
                                    }
                                    if(wall_ref == 1)
                                    {
                                        dist = int(left_wheel - first_left);
                                        if(dist>= enc_count)//750 is for 400ppr, so 935 is for 500ppr
                                        {
                                            flag_turn = 5;
                                            wall_ref = 2;
                                            new_flag = 0;
                                            ROS_INFO("RSF : Sraight before turn completed");
                                        }
                                        if(dist< enc_count)//750 is for 400ppr, so 935 is for 500ppr
                                        {
                                            flag_turn = 3;
                                            new_flag = 1;
                                            ROS_DEBUG("distance is %d", dist);
                                            if(dist < ratio_enc_count)//350 is for 400ppr, so 435 is for 500ppr
                                            {
                                                lidar_change = 0;
                                            }
                                            else
                                            {
                                                lidar_change = 1;
                                            }
                                        }
                                    }
                                    if(wall_ref == 2)
                                    {
                                        first_next_left = left_wheel;
                                        wall_ref = 3;
                                        new_flag = 0;
                                    }
                                    if(wall_ref == 3)
                                    {
                                        dist_next = int(left_wheel - first_next_left); //int first_next_left = 0, dist_next = 0, stringent = 0;
                                        if(dist_next<= ppr_d_2)//200 is for 400ppr, so 250 is for 500 ppr
                                        {
                                            flag_turn = 4;
                                            stringent = 1;
                                            lidar_change = 2;
                                            ROS_DEBUG("distance_next is %d", dist_next);
                                        }
                                        else
                                        {
                                            flag_turn = 4;
                                            stringent = 0;
                                            lidar_change = 2;
                                        }
                                    }
                                    if(wall_ref == 4)
                                    {
                                        flag_turn = 4;
                                        stringent = 0;
                                        lidar_change = 2;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        if(flag_sf == 1)
                        {
                            if(stringent == 1)
                            {
                                flag_sf = 0;//high prob that during the turn h>=8, so let it roll
                                ROS_DEBUG("yaha aake bahar nikla");
                            }
                            else
                            {
                                if(h>=presence)
                                {
                                    flag_turn = 10;//sensing stopping
                                    ROS_WARN_THROTTLE(1, "RSF : Sensing stopping while step present");
                                }
                                else if(h<presence and lidar_change != 2)
                                {
                                    flag_turn = 10;//sensing stopping
                                    ROS_WARN_THROTTLE(1, "RSF : Sensing stopping while turn(straight)");
                                }
                                else if(h<presence and lidar_change == 2)
                                {
                                    flag_turn = 1; //shortwall turning
                                    wall_ref = 4;
                                    ROS_WARN_THROTTLE(1, "RSF : Turning right short wall (after straight of turn)");
                                }
                            }
                        }
                        if(flag_sf == 0)
                        {
                            if(h>=presence and stringent == 0 and new_flag == 0)
                            {
                                flag_turn = 0;
                                wall_ref = 0;
                                lidar_change = 0;
                                stringent = 0;
                                ROS_DEBUG("wall following");

                                done_pp = 0;
                            }
                            else if(h>=presence and stringent == 1 and new_flag == 0)
                            {
                                if(dist_next<=ppr_d_2)//200 is for 400ppr, so 250 is for 500 ppr
                                {
                                    flag_turn = 4;
                                    stringent = 1;
                                    wall_ref = 3;
                                    lidar_change = 2;
                                    ROS_WARN_THROTTLE(1, "RSF : step following not started due to stringentt");
                                }
                                else
                                {
                                    stringent = 0;
                                    flag_turn = 0;
                                    wall_ref = 0;
                                    lidar_change = 0;
                                    ROS_WARN_THROTTLE(1, "RSF : step following started from stringent");
                                }
                            }
                            else if(h>=presence and new_flag == 1)
                            {
                                sec_start =ros::Time::now().toSec();
                                flag_turn = 11; //stopping due to unknown object detection
                                new_flag = 2;
                                ROS_WARN_THROTTLE(1, "RSF : Stopping due to unkwon object detection");
                            }
                            else if(new_flag == 2 and done_pp != 1)
                            {
                                sec_end =ros::Time::now().toSec();
                                if(sec_end - sec_start >= 3.0000)
                                {
                                    new_flag = 0;
                                }
                                flag_turn = 11; 
                            }
                            else if(new_flag == 2 and done_pp == 1)
                            {
                                sec_end =ros::Time::now().toSec();
                                if(sec_end - sec_start >= 1.0000)
                                {
                                    new_flag = 0;
                                }
                                flag_turn = 11;
                            }
                            if(h<presence and done_pp != 2)
                            {
                                if(done_pp == 0)
                                {
                                    first_pp = right_wheel;
                                    done_pp = 1;
                                    ROS_INFO("RSF : P to P started with P to P as %f", p_to_p);
                                }
                                if(done_pp == 1)
                                {
                                    dist_pp =  int(right_wheel - first_pp);
                                    if(dist_pp>=enc_pp)
                                    {
                                        done_pp = 2;
                                        flag_turn = 5;
                                        new_flag = 0;
                                        ROS_INFO("LSF : P to P finished");
                                    }
                                    else
                                    {
                                        flag_turn = 55;
                                        ROS_DEBUG("LSF : D PP is %d", dist_pp);
                                        new_flag = 1;
                                    }
                                }
                            }
                            if(h<presence and done_pp == 2)
                            {
                                if(setpoint>0.65)
                                {
                                    flag_turn = 8;
                                    ROS_INFO_THROTTLE(2, "RSF : Inside turn");
                                }
                                else
                                {
                                    if(wall_ref == 0)
                                    {
                                        first_left = left_wheel;
                                        wall_ref = 1;
                                        ROS_INFO("RSF : Straight before Turn started");
                                    }
                                    if(wall_ref == 1)
                                    {
                                        dist = int(left_wheel - first_left);
                                        if(dist>= enc_count)//750 is for 400ppr, so 935 is for 500ppr
                                        {
                                            flag_turn = 5;
                                            wall_ref = 2;
                                            new_flag = 0;
                                            ROS_INFO("LSF : Straight before Turn completed");
                                        }
                                        if(dist< enc_count)//750 is for 400ppr, so 935 is for 500ppr
                                        {
                                            flag_turn = 3;
                                            new_flag = 1;
                                            ROS_DEBUG("distance is %d", dist);
                                            if(dist < ratio_enc_count)//350 is for 400ppr, so 435 is for 500ppr
                                            {
                                                lidar_change = 0;
                                            }
                                            else 
                                            {
                                                lidar_change = 1;
                                            }
                                        }
                                    }
                                    if(wall_ref == 2)
                                    {
                                        first_next_left = left_wheel;
                                        wall_ref = 3;
                                        new_flag = 0;
                                    }
                                    if(wall_ref == 3)
                                    {
                                        dist_next = int(left_wheel - first_next_left); //int first_next_left = 0, dist_next = 0, stringent = 0;
                                        if(dist_next<= ppr_d_2)//200 is for 400ppr, so 250 is for 500 ppr
                                        {
                                            flag_turn = 4;
                                            stringent = 1;
                                            lidar_change = 2;
                                            ROS_DEBUG("distance_next is %d", dist_next);
                                        }
                                        else
                                        {
                                            flag_turn = 4;
                                            stringent = 0;
                                            lidar_change = 2;
                                        }
                                    }
                                    if(wall_ref == 4)
                                    {
                                        flag_turn = 4;
                                        stringent = 0;
                                        lidar_change = 2;
                                    }
                                }
                            }   
                        }
                    }
                }
                else
                {
                    flag_turn = 20;
                    ROS_WARN_THROTTLE(1, "RSF : Stopping due to left side obstacle, press 'R' to disable");
                }
            }
        }
        else if(starter != 40)
        {
            flag_turn = 0;
            flag1 = 0;
            integral_end = -1.0;
        }
        if(flag1 == 1)
        {
            if(flag_turn == 0)
			{
                //err = -err;

				int tuning = 0;
				ros::param::get("/tuning",tuning);

				if(tuning == 1)
				{
					ros::param::get("/KP",KP);
					ros::param::get("/KI",KI);
					ros::param::get("/KD",KD);
					param_var++;
					if(param_var>=20)
					{
						ros::param::set("/tuning", 0);
						tuning = 0;
						param_var = 0;
					}
				}

                if(integral_end == -1.0)
                {
                    ROS_INFO("RSF : Step following started");
                }

				float op =  rs_tuner.pid_tuner(KP, KI, KD, integral_end, err);

				integral_end = ros::Time::now().toSec();
				
				ROS_DEBUG("  error :%f",err);
			
				ROS_DEBUG("  op :%f",op);
				msg1.linear.x=thres;
				msg1.angular.z = max(float(-0.12), min(op, float(0.12)));
				
				cmd_pub.publish(msg1);
			}
            else if(flag_turn == 1)//short wall turn right
            {	
                msg1.linear.x=0.05233;   //rpm/191.08 =  0.0942 for 18
                msg1.angular.z=-0.2990; //rpm/33.438  = 0.5383 for 18
                ROS_DEBUG("short wall turning right");
                cmd_pub.publish(msg1);
                integral_end = -1.0; 
            }
            else if(flag_turn == 2)
            {
                msg1.linear.x=0.0;   
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside stopping due to short wall");
                integral_end = -1.0;
            }
            else if(flag_turn == 3) //go straight
            {
                msg1.linear.x=thres;   
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside straight motion");
                integral_end = -1.0;
            }
            else if(flag_turn == 55) //go straight
            {
                msg1.linear.x=0.15;
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside straight motion from PP");
                integral_end = -1.0;
            }
            else if(flag_turn == 4) //turn right
            {
                msg1.linear.x=0.05233;   //rpm/191.08 =  0.0942 for 18
                msg1.angular.z=-0.2990; //rpm/33.438  = 0.5383 for 18
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside turning due to no wall");
                integral_end = -1.0;
            }
            else if(flag_turn == 8) //turn right
            {
                msg1.linear.x=0.15;
                msg1.angular.z=-turning_radius;
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside turning due to no wall");
                integral_end = -1.0;
            }
            else if(flag_turn == 9)
            {
                msg1.linear.x=0.05233;   //rpm/191.08 =  0.0942 for 18
                msg1.angular.z=0.2990; //rpm/33.438  = 0.5383 for 18
                cmd_pub.publish(msg1);
                ROS_DEBUG("Inside Left turn");
                integral_end = -1.0;
            }
            else if(flag_turn == 88)//turn right for short wall correction
            {
                if(new_dir == 0)
                {
                    dir_rad = 0.0;
                    new_dir = 1;
                    first_next_left = left_wheel;
                }
                else
                {
                    if(left_wheel > (first_next_left+40))
                    {
                        dir_rad-=0.04;//worked on 17th october took longer turn
                        first_next_left = left_wheel;
                        ROS_DEBUG("Angular z is %f", dir_rad);
                    }
                }
                msg1.linear.x=0.15;
                msg1.angular.z= max(-0.12F, dir_rad);
                cmd_pub.publish(msg1);
                ROS_DEBUG("Inside right alignment drift");
                integral_end = -1.0;
            }
            else if(flag_turn == 10) //sensing stopping
            {
                msg1.linear.x=0.0;
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside SENSING STOPPING");
                integral_end = -1.0;
            }
            else if(flag_turn == 11) //sensing stopping
            {
                msg1.linear.x=0.0;   
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside stopping due to unknown moving object");
                integral_end = -1.0;
            }
            else if(flag_turn == 20) //sensing stopping
            {
                msg1.linear.x=0.0;
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                ROS_DEBUG("RSF : Inside sensing stopping on left side");
                integral_end = -1.0;
            }
            else
            {
                msg1.linear.x=0.0;   
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                ROS_DEBUG("inside stopping due to some error");
                integral_end = -1.0;
            }
        }
        else
        {
            ROS_DEBUG("calibrating");
            if(one_time == 0)
            {
                msg1.linear.x=0.0;   
                msg1.angular.z=0.0;
                cmd_pub.publish(msg1);
                one_time++;
            }
        }
    }
}
int main(int argc, char **argv) 
{
	ros::init(argc, argv, "brs_right_step");
    ros::NodeHandle n;

    N::debugging_next obj_rs;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>("/new_scan_far", 2, laser_scan_callback);
	ros::Subscriber start_sub = n.subscribe<std_msgs::Int8>("/starting", 10, start_callback);
    ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",10,&leftcallback);
    ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",10,&rightcallback);

    ros::Subscriber stop=n.subscribe<geometry_msgs::Vector3>("/wall_sensing",10,&stopcallback);

    ros::Publisher node_work = n.advertise<std_msgs::Bool>("/rs_work", 10);
	ros::Rate rate(30);
	// this will return false on ctrl-c or when you call ros::shutdown()

    std_msgs::Bool rs;
    while (ros::ok()) 
	{
        if(starter == 40)
        {
            allowed = obj_rs.sensor_status(1.0, 0, 1, 0, 1, 1);
            if(allowed == 0)
            {
                integral_end = -1.0;
            }
            rs.data = true;
			node_work.publish(rs);
        }
		ros::spinOnce();
		rate.sleep();
	}
	ros::spin();
	return 0;
}