/*
FOR TUNING PID ->
rosrosparam set /tuning 1 && rosparam set /KP 0.2 && rosparam set /KI 0.0 && rosparam set /KD 0.1

FOR NEW SETPOINT ->
rosparam set /new_command 1 && rosparam set /ws_setpoint 0.55 && rosparam set /presence_wall 20

FOR Height ->
rosparam set /lidar_height 0.65
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include <cmath>
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

ros::Publisher cmd_pub;

float left_enc_ppr = 500.0, right_enc_ppr = 500.0;

float KP = 0.2;
float KI = 0.0;
float KD = 0.1;

int presence, allow_shortwall;

double integral_end = 0.0, sw_param;

float thres = 0.25, lidar_size = 0.035, setpoint = 0.50000, turning_radius = 0.0, p_to_p;

float short_wall, lidar_height = 0.6296, stopped = 0.0, stopped_front = 0.0;//0.7200, 0.6200

int flag1=0, z = 0, wall_ref = 0, second_time = 0, new_command = 0, param_var = 0;

long int left_wheel = 0, first_left = 0, right_wheel = 0, first_pp = 0;

int first = 0, allowed = 0, dist = 0, dist_before_turn = 0, dist_pp = 0, enc_pp, done_pp =0;
float height = 0.0, degree;

float error_new = 0.0, centre = 0.0, newdegree=0.0;
float setpoint_array[100], lg[100], err=0, ws_setpoint, lidar_end;

int h, reference = 0, one_time = 0, flag_turn = 0, starter = 0, stepmanual = 0, flag_sf = 0;

geometry_msgs::Twist msg1;

T::tuner rw_tuner;

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
	if(allow_shortwall == 0) //disabled
	{
		if(setpoint>0.70)
		{
			stopped_front = stopped_front;
		}
		else
		{
			stopped_front = 0.0;
		}	
	}
	else if(allow_shortwall == 1)
	{
		stopped_front = 0.0;
	}
}
void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	if(allowed == 1)
	{
		int count = scan->scan_time / scan->time_increment;

		if(starter == 20)
		{
			int sf_ref = 0, p = 0, error_cal = 0, lmn, stu, pqr;

			float average_new[100] = {0.0}, average = 100.0, setpoint_new = 0.0;

			float first_error, second_error;

			one_time = 0;

			ros::param::get("/new_command",new_command);

			if(new_command == 1)
			{
				ros::param::get("/ws_setpoint",ws_setpoint);

				ros::param::get("/lidar_height",lidar_height);

				ros::param::set("/new_command", 0);

				ros::param::get("/presence_wall", presence);

                ros::param::get("/left_enc_ppr", left_enc_ppr);

                ros::param::get("/right_enc_ppr", right_enc_ppr);
                
				ros::param::get("/sw_param", sw_param);

				ros::param::get("/p_to_p", p_to_p);

				ros::param::get("/allow_shortwall", allow_shortwall);

				ros::param::get("/lidar_right_end", lidar_end);

                //setpoint = ws_setpoint + lidar_end;
				setpoint = ws_setpoint;

				enc_pp = int(((p_to_p/3.281)*right_enc_ppr)/CIRCU);

				turning_radius = setpoint*0.15*1.2;
				lidar_height = lidar_height + 0.0096;
				new_command = 0;

				dist_before_turn = int(0.75*left_enc_ppr);

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
				done_pp = 0;
				wall_ref = 0;
				flag_sf = 0;
				reference = 0;
			}
			for(int i = 0; i < count; i++)
			{
				float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

				if(first == 0)
				{
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
					for(int k=30; k<=89;k++)
					{
						height = lidar_height + ((k-40) * 0.00024); //4.4 cm total height so, 0 to 90 is 2.2 cm, so for 1 degree = 2.2/90 cm
						length_to_ground = centre/cos((k) * PI/180) - lidar_size; //which is equal to 0.00024m
						lg[k] = 0.8 * length_to_ground;
						setpoint_parallel = setpoint/sin((k) * PI/180);
						height = height * height;
						length_to_ground = length_to_ground * length_to_ground;
						cos_alpha = sqrt(1 - (height/length_to_ground));
						setpoint_array[k] = setpoint_parallel/cos_alpha;
						ROS_DEBUG(" Degree is %d and Setpoint is %f and height is %f and length is %f", k, setpoint_array[k], height, length_to_ground);
					}
					first = 2;
					short_wall = min((0.8*centre), sw_param);
					ROS_INFO("LWF : Centre value is %f and short wall is %f", centre, short_wall);
				}
				else
				{
					if(setpoint < 0.70 or allow_shortwall == 1)
					{
						if((degree>=-10.0 and degree<=1.0) and scan->ranges[i]>0.1 and scan->intensities[i]!=0) 
						{   
							if(scan->ranges[i] <=short_wall)
							{
								flag_sf = 1;
								break;
							}
							else if(scan->ranges[i] >short_wall)
							{
								flag_sf = 0;
							}
							else
							{
								flag_sf = 0;
							}	
						}
						if((degree>1.0 and degree<=10.0) and scan->ranges[i]>0.1 and scan->intensities[i]!=0) 
						{   
							if(scan->ranges[i] <=short_wall)
							{
								flag_sf = 1;
								break;
							}
							else if(scan->ranges[i] >short_wall)
							{
								flag_sf = 0;
							}
							else
							{
								flag_sf = 0;
							}	
						}
						if(reference != 1)
						{
							if((degree>-30.0 and degree<-10.0) and scan->ranges[i]>0.1 and scan->intensities[i]!=0 and sf_ref == 0) 
							{   
								if(scan->ranges[i] <=short_wall)
								{
									flag_sf = 2;
									sf_ref = 1;
								}
								else if(scan->ranges[i] >short_wall)
								{
									flag_sf = 0;
									sf_ref = 0;
								}
								else
								{
									flag_sf = 0;
								}	
							}
						}
						else if(reference == 1)
						{
							if((degree>-30.0 and degree<-10.0) and scan->ranges[i]>0.1 and scan->intensities[i]!=0) 
							{   
								if(scan->ranges[i] <=short_wall)
								{
									flag_sf = 2;
									break;
								}
								else if(scan->ranges[i] >short_wall)
								{
									flag_sf = 0;
								}
								else
								{
									flag_sf = 0;
								}
							}
						}
					}
					if((degree>=-89.0 and degree<=-30.0) and (scan->ranges[i] < (2.0 * setpoint_array[-(int(degree))])) and scan->ranges[i] < lg[-(int(degree))] and scan->intensities[i]!=0)
					{
						if(newdegree!=degree and error_cal == 0)
						{
							lmn = -int(degree);
							average_new[lmn] = scan->ranges[i];

							first_error = setpoint_array[lmn] - average_new[lmn];
							error_cal = 1;
							newdegree = degree;
							p++;
							stu = lmn;
						}
						if(newdegree!=degree and error_cal == 1)
						{
							pqr = -int(degree);
							average_new[pqr] = scan->ranges[i];

							second_error = setpoint_array[pqr] - average_new[pqr];
							error_cal = 2;
							newdegree = degree;
							p++;
						}
						if(error_cal == 2)
						{
							error_new = second_error - first_error;
							if(error_new >= 0)
							{
								stu = pqr;
							}
							else if(error_new < 0)
							{
								stu = lmn;
							}
							error_cal = 3;
						}
						if(newdegree!=degree and error_cal == 3)
						{
							lmn = -int(degree);
							average_new[lmn] = scan->ranges[i];

							first_error = setpoint_array[lmn] - average_new[lmn];
							error_cal = 4;
							newdegree = degree;
						}
						if(error_cal == 4)
						{
							second_error = setpoint_array[stu] - average_new[stu];
							//ROS_DEBUG("stu is %d [%f] and lmn is %d [%f]", stu, second_error, lmn, first_error);
							p++;
							error_new = second_error - first_error;
							if(error_new >= 0)
							{
								stu = stu;
							}
							else if(error_new < 0)
							{
								stu = lmn;
							}
							error_cal = 3;//check this
						}
					}
				}
			}
			ROS_DEBUG("P is %d", p);
			
			z++;

			if(z > 3 and first == 2)
			{
				flag1=1;
				if(stopped == 0 and stopped_front == 0.0)
				{
					if((flag_sf == 0 or flag_sf == 2) and reference != 1)
					{
						if(p>=presence)
						{
							flag_turn = 0;
							reference = 0;
							wall_ref = 0;
							done_pp = 0;
							ROS_DEBUG("wall following");

							average = average_new[stu];
							setpoint_new = setpoint_array[stu];

							err=(setpoint_new-average);
							//err=err;

							ROS_DEBUG(" Degree considered is %d and setpoint is : %f and average is : %f", stu, setpoint_array[stu], average_new[stu]);
						}
						if(p<presence and done_pp != 2)
						{
							if(done_pp == 0)
							{
								first_pp = right_wheel;
								done_pp = 1;
								ROS_INFO("RWF : P to P started with P to P as %f", p_to_p);
							}
							if(done_pp == 1)
							{
								dist_pp =  int(right_wheel - first_pp);
								if(dist_pp>=enc_pp)
								{
									done_pp = 2;
									flag_turn = 5;
									ROS_INFO("RWF : P to P finished");
								}
								else
								{
									flag_turn = 55;
									ROS_DEBUG("RWF : Distance PP  is %d", dist_pp);
								}
							}
						}
						if(p<presence and done_pp == 2)
						{
							reference = 0;
							if(setpoint <= 0.7000)
							{
								if(wall_ref == 0)
								{
									first_left = left_wheel;
									wall_ref = 1;
									ROS_INFO("RWF : Straight before Turn started");
								}
								else if(wall_ref == 1)
								{
									dist = int(left_wheel - first_left);
									if(dist>=dist_before_turn)//300 is for 400ppr, so 375 for 500ppr
									{
										flag_turn = 5;
										wall_ref = 2;
										ROS_INFO("LWF : Straight before Turn completed");
									}
									else
									{
										flag_turn = 3;
										ROS_DEBUG("RWF : Distance straight is %d", dist);
									}
								}
								else if(wall_ref == 2)
								{
									flag_turn = 4;
								}
							}
							else
							{
								flag_turn = 8;
								ROS_INFO_THROTTLE(2, "RWF : Inside turn");
							}
						}   
					}
					else if(flag_sf == 1 and reference == 0)
					{
						flag_turn = 1;
						reference = 1;
						wall_ref = 0;
						ROS_DEBUG("Short wall start");
					}
					else if((flag_sf == 1 or flag_sf == 2) and reference == 1)
					{
						flag_turn = 1;
						wall_ref = 0;
						ROS_DEBUG("Short wall turning");
					}
					else if(flag_sf == 0 and reference == 1)
					{
						reference = 0;
						wall_ref = 0;
						ROS_DEBUG("shortwall end");
					}
				}
				else
				{
					flag_turn = 12;
					ROS_WARN_THROTTLE(1, "RWF : Stopped due to Left or front obstacle");
				}
			}
		}
		else if(starter != 20)
		{
			flag_turn = 0;
			flag1 = 0;
			integral_end = -1.0;
		}
		if(flag1 == 1)
		{
			if(flag_turn == 0)
			{
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
                    ROS_INFO("RWF : Wall following started");
                }

				float op =  rw_tuner.pid_tuner(KP, KI, KD, integral_end, err);

				integral_end = ros::Time::now().toSec();
				
				ROS_DEBUG("  error :%f",err);
			
				ROS_DEBUG("  op :%f",op);
				msg1.linear.x=thres;
				msg1.angular.z = max(float(-0.15), min(op, float(0.15)));
				
				cmd_pub.publish(msg1);
			}
			else if(flag_turn == 1)//short wall turn left
			{	
				msg1.linear.x=0.07326;   //rpm/191.08 =  0.0942 for 18
				msg1.angular.z=0.418; //rpm/33.438  = 0.5383 for 18
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
				msg1.linear.x=0.15;   
				msg1.angular.z=0.0;
				cmd_pub.publish(msg1);
				ROS_DEBUG("inside straight motion");
				integral_end = -1.0;
			}
			else if(flag_turn == 4) //turn right
			{
				msg1.linear.x=0.07326;   //rpm/191.08 =  0.0942 for 18
				msg1.angular.z=-0.418; //rpm/33.438  = 0.5383 for 18
				cmd_pub.publish(msg1);
				ROS_DEBUG("inside turning due to no wall");
				integral_end = -1.0;
			}
			else if(flag_turn == 8) //turn right
			{
				msg1.linear.x=0.15;   //rpm/191.08
				msg1.angular.z=-turning_radius; //rpm/33.438
				cmd_pub.publish(msg1);
				ROS_DEBUG("inside turning due to no wall");
				integral_end = -1.0;
			}
			else if(flag_turn == 12)
			{
				msg1.linear.x=0.0;   
				msg1.angular.z=0.0;
				cmd_pub.publish(msg1);
				ROS_DEBUG("inside stopping due to obstacle");
				integral_end = -1.0;
			}
			else if(flag_turn == 55) //go straight
			{
				msg1.linear.x=0.15;   
				msg1.angular.z=0.0;
				cmd_pub.publish(msg1);
				ROS_DEBUG("inside PP motion");
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
	ros::init(argc, argv, "brs_right_wall");
	ros::NodeHandle n;

	N::debugging_next obj_rw;

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

	ros::Publisher node_work = n.advertise<std_msgs::Bool>("/rw_work", 10);
	ros::Rate rate(40);
	// this will return false on ctrl-c or when you call ros::shutdown()

	std_msgs::Bool rw;
	while (ros::ok()) 
	{
		if(starter == 20)
        {
            allowed = obj_rw.sensor_status(1.0, 0, 1, 0, 1, 1);
            if(allowed == 0)
            {
                integral_end = -1.0;
            }
			rw.data = true;
			node_work.publish(rw);
        }
		ros::spinOnce();
		rate.sleep();
	}
	ros::spin();
	return 0;
}