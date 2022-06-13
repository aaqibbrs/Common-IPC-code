#include"ros/ros.h"
#include"geometry_msgs/Vector3.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <iostream>
#include<cmath>
#include<math.h>
#include <ctime>
#include <ros/console.h>
#include <stdlib.h>
#include "timing_cal.h"
#include "debug_sensor.h"

using namespace std;

int zo = 0, allowed = 0, l =0;

#define PI 3.14159265

float direction, req_x = 0.0, linear = 0.25;
int flag_first = 0, p_data = 0, new_file = 0, first_time = 0, flag_second =0, tara = 0, mant = 0;

int *left_reference = new int(0);
int *right_reference = new int(0);

//double CIRC = 0.07853981; //(2*PI*r/400)/2
double CIRC = 0.062831853; //(2*PI*r/500)/2

float *left_enc_ppr = new float(0.0);
float *right_enc_ppr = new float(0.0);

std::ofstream myfile("/home/bros/pose_str.csv",std::ios::out | std::ios::app); //change this accordingly
std::ofstream myfile2("/home/bros/trial_file_imp.csv",std::ios::out | std::ios::app); //change this accordingly

long int left_wheel, right_wheel, left_count, right_count, left_constant, right_constant;

double distance_new, prev_degree, dir_degree, new_degree, first_degree, ss_degree;

double x = 0.0, y = 0.0;

ros::Time now_time;

void leftcallback(const std_msgs::Int64::ConstPtr& msg)
{
	left_wheel = msg->data;
	if(flag_first == 0 and *left_reference == 0)
	{
		left_constant = left_wheel;
		*left_reference = 1;
	}
}
void pose_start_callback(const std_msgs::Int8::ConstPtr& p_start)
{      
	p_data = p_start->data;
}
void rightcallback(const std_msgs::Int64::ConstPtr& msg2)
{
  	right_wheel = msg2->data;
	if(flag_first==0 and *right_reference == 0)
	{
		right_constant = right_wheel;
		*right_reference = 1;
	}
}
void dataCB(const geometry_msgs::Vector3::ConstPtr& dat)
{
	direction = dat->x;

	if(flag_first == 0 and *left_reference == 1 and *right_reference == 1)
	{
		prev_degree = direction;
		first_degree = direction;

		ros::param::get("/left_enc_ppr",*left_enc_ppr);
		ros::param::get("/right_enc_ppr",*right_enc_ppr);

		if(*left_enc_ppr == 500.0 and *right_enc_ppr == 500.0)
		{
			CIRC = 0.062831853;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
		    delete right_reference;
            flag_first = 1;
		}
		else if(*left_enc_ppr == 400.0 and *right_enc_ppr == 400.0)
		{
			CIRC = 0.07853981;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
		    delete right_reference;
            flag_first = 1;
		}
		else if(*left_enc_ppr == 0.0 or *right_enc_ppr == 0.0)
		{
			ROS_ERROR(" POSE_STR : Encoder ppr not received: Set parameter /left_enc_ppr and /right_enc_ppr");
		}
        else
        {
            CIRC = 0.07068583462;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
		    delete right_reference;
            flag_first = 1;
        }
	}
}
int main(int argc,char **argv)
{
	if(!myfile)
	{
		std::cout<<"open file failure"<<std::endl;
	}
	myfile<<std::endl;
    myfile<<"New file start \n";

	if(!myfile2)
	{
		std::cout<<"open file failure"<<std::endl;
	}
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

    myfile<<"Pose node start : \t"<<dt;
    myfile<<std::endl;
    
	myfile<<"X \t Y \t Converted Yaw \t Actual Yaw \n";

	myfile2<<std::endl;
    myfile2<<"New file start"<<dt<<std::endl;

    ROS_INFO("living_doc \t X \t ss_degree \t msg.angular.z \t WHERE");

	ros::init(argc,argv,"brs_pose_data");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	ros::NodeHandle n;

	N::debugging_next obj_pose;

	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber gyro_data = n.subscribe<geometry_msgs::Vector3>("/brick_imu", 10,&dataCB);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l", 10,&leftcallback);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r", 10,&rightcallback);
	ros::Subscriber get_pose=n.subscribe<std_msgs::Int8>("/pose_starting_str",10,&pose_start_callback);

	ros::Rate loop_rate(20);

	geometry_msgs::Twist msg;

	while (ros::ok()) 
	{
		if(flag_first == 2)
		{
			if(p_data == 1 and first_time == 0)
			{
				left_constant = left_wheel;
				right_constant = right_wheel;
				left_count = 0;
				right_count = 0;
				x=0;
				y=0;
				first_degree = direction;//comment as required, added for straight motion
				prev_degree = direction;

				first_time = 1;
			}
			if(p_data == 1 and first_time == 1)
			{
				zo++;
				if(zo>=2000)
				{
					zo = 0;
				}
				new_file = 0;
				dir_degree = direction;
				
				if((dir_degree >= 350.0 and prev_degree <= 10.0) or (dir_degree <= 10.0 and prev_degree >= 350.0))
				{
					if(dir_degree >= 350.0)
					{
						prev_degree = prev_degree + 360.0;

						new_degree = (dir_degree + prev_degree)/2.0;

						prev_degree = prev_degree - 360.0;
					}
					else if(prev_degree >=350.0)
					{
						dir_degree = dir_degree + 360.0;

						new_degree = (dir_degree + prev_degree)/2.0;

						dir_degree = dir_degree - 360.0;
					}
				}
				else
				{
					new_degree = (dir_degree + prev_degree)/2.0;
				}

				if(new_degree >= first_degree)
				{
					new_degree = new_degree - first_degree;
				}
				else if(new_degree < first_degree)
				{
					new_degree = 360.0 - (first_degree - new_degree);
				}
				else
				{
					ROS_DEBUG("error");
				}
				left_count = left_wheel - left_constant - left_count;
				right_count = right_wheel - right_constant - right_count;

				distance_new = CIRC*(left_count+right_count);

				x = x + distance_new*sin(new_degree * PI/180);
				y = y + distance_new*cos(new_degree * PI/180);

				if(dir_degree >= first_degree)
				{
					ss_degree = dir_degree - first_degree;
				}
				else if(dir_degree < first_degree)
				{
					ss_degree = 360.0 - (first_degree - dir_degree);				
				}
				
				allowed = obj_pose.sensor_status(1.0, 1, 1, 0, 0, 1);

				if(allowed == 1)
				{
					ros::param::get("/sensing_straight", tara);

					if(tara==1)
					{
						msg.linear.x = 0.0;
						msg.angular.z = 0.0;
						cmd_pub.publish(msg);
						ROS_ERROR("Straight : Straight motion stopped due to obstacle");
					}
					else
					{
						if(flag_second == 0)
						{
							req_x = 0.0;
							flag_second = 1;
						}
						else
						{
							if((x >= (req_x-2.0)) and (x <= (req_x+2.0)))
							{
								if(ss_degree >= 359.000 or ss_degree <= 1.000)
								{
									msg.linear.x = linear;
									msg.angular.z = 0.0;
									cmd_pub.publish(msg);
									ROS_DEBUG("Inside full straight");
									mant = 10;
								}
								else if(ss_degree < 359.000 and ss_degree > 310.0)
								{
									msg.linear.x = linear;
									msg.angular.z = max(-((359.0 - ss_degree)/200.0),-0.10);
									ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
									cmd_pub.publish(msg);
									ROS_DEBUG("Inside turn right 1A"); //only align
									mant = 11;
								}
								else if(ss_degree < 50.0 and ss_degree > 1.000)
								{
									msg.linear.x = linear;
									msg.angular.z = min(((ss_degree - 1.0)/200.0),0.10);
									ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
									cmd_pub.publish(msg);
									ROS_DEBUG("Inside turn left 1B"); //only align
									mant = 12;
								}
								else
								{
									ROS_ERROR("Straight : 1 should not come here");
									mant = 1010;
								}
							}
							else if(x < (req_x-2.0))
							{
								if(ss_degree >= 359.000 or ss_degree <= 1.000)
								{
									msg.linear.x = linear;
									//msg.angular.z = -0.05;//need to change this later
									msg.angular.z = max(-((-2.0 - x)/200.0),-0.10);
									cmd_pub.publish(msg);
									ROS_DEBUG("Inside turn right 2A");
									mant = 20;
								}
								else if(ss_degree < 359.000 and ss_degree > 310.0)
								{
									msg.linear.x = linear;
									msg.angular.z = max(-(((359.0 - ss_degree)/300.0)+((-2.0 - x)/300.0)),-0.15);
									ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
									cmd_pub.publish(msg);
									ROS_DEBUG("Inside turn right 2B");
									mant = 21;
								}
								else if(ss_degree < 50.0 and ss_degree > 1.000)
								{
									if((req_x - x) >= 30.0)
									{
										msg.linear.x = linear;
										//msg.angular.z = -0.10;
										msg.angular.z = max(-(((ss_degree - 1.0)/100.0)+((-30.0-x)/300.0)),-0.15);
										cmd_pub.publish(msg);
										ROS_DEBUG("Turn right due to 2C");
										mant = 220;
									}
									else if((req_x - x) < 30.0 and (req_x - x) >= 15.0)
									{
										if(ss_degree>20.0)
										{
											msg.linear.x = linear;
											msg.angular.z = 0.0;
											cmd_pub.publish(msg);
											ROS_DEBUG("Go straight due to 2C");
											mant = 2210;
										}
										else
										{
											msg.linear.x = linear;
											msg.angular.z = -0.05;
											cmd_pub.publish(msg);
											ROS_DEBUG("Go right/straight due to 2C");
											mant = 2211;
										}
									}
									else if((req_x - x) < 15.0)
									{
										if(ss_degree > 20.0)
										{
											msg.linear.x = linear;
											msg.angular.z = min((((ss_degree - 1.0)/400.0)+((-2.0-x)/400.0)),0.10);
											ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
											cmd_pub.publish(msg);
											ROS_DEBUG("Go left due to IRONIC 2C");
											mant = 2220;
										}
										else
										{
											msg.linear.x = linear;
											msg.angular.z = 0.0;
											ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
											cmd_pub.publish(msg);
											ROS_DEBUG("Go straight due to IRONIC 2C");
											mant = 2221;
										}
									}
									else
									{
										ROS_ERROR("Never come here");
										mant = 28888;
									}
								}
								else
								{
									ROS_ERROR("Straight : 2 should not come here");
									mant = 29999;
								}
							}
							else if(x > (req_x+2.0))
							{
								if(ss_degree >= 359.000 or ss_degree <= 1.000)
								{
									msg.linear.x = linear;
									//msg.angular.z = 0.05;
									msg.angular.z = min((((x - 2.0)/200.0)),0.10);
									cmd_pub.publish(msg);
									ROS_DEBUG("Inside turn left 3A");
									mant = 30;
								}
								else if(ss_degree < 359.000 and ss_degree > 310.0)
								{
									if((x - req_x) >= 30.0)
									{
										msg.linear.x = linear;
										//msg.angular.z = 0.10;
										msg.angular.z = min((((359.0 - ss_degree)/100.0)+((x-30.0)/300.0)),0.15);
										cmd_pub.publish(msg);
										ROS_DEBUG("Inside turn left due to 3B");
										mant = 310;
									}
									else if((x - req_x) < 30.0 and (x - req_x) >= 15.0)
									{
										if(ss_degree<340.0)
										{
											msg.linear.x = linear;
											msg.angular.z = 0.0;
											cmd_pub.publish(msg);
											ROS_DEBUG("Inside straight/straight due to 3B");
											mant = 3110;
										}
										else
										{
											msg.linear.x = linear;
											msg.angular.z = 0.05;
											cmd_pub.publish(msg);
											ROS_DEBUG("Inside left/straight due to 3B");
											mant = 3111;
										}
									}
									else if((x-req_x) < 15.0)
									{
										if(ss_degree < 340.0)
										{
											msg.linear.x = linear;
											msg.angular.z = max(-(((359.0 - ss_degree)/400.0)+((x-2.0)/400.0)),-0.10);
											ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
											cmd_pub.publish(msg);
											ROS_DEBUG("Go right due to IRONIC 3B");
											mant = 3120;
										}
										else
										{
											msg.linear.x = linear;
											msg.angular.z = 0.0;
											ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
											cmd_pub.publish(msg);
											ROS_DEBUG("Go straight due to IRONIC 3B");
											mant = 3121;
										}
									}
									else
									{
										ROS_ERROR(" Straight : 3 Never come here");
										mant = 3888;
									}
								}
								else if(ss_degree < 50.0 and ss_degree > 1.000)
								{
									msg.linear.x = linear;
									msg.angular.z = min((((ss_degree - 1.0)/300.0)+((x-2.0)/300.0)),0.15);
									ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
									cmd_pub.publish(msg);
									ROS_DEBUG("Inside turn left due to 3C");
									mant = 32;
								}
								else
								{
									ROS_ERROR("Straight : 3 should not come here");
									mant = 3999;
								}
							}
							myfile2<<x<<"\t"<<ss_degree<<"\t"<<msg.angular.z<<"\t"<<mant;
							myfile2<<std::endl;

							ROS_INFO("living_doc \t %f \t %f \t %f \t %d", x, ss_degree, msg.angular.z, mant);
						}
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

				ROS_INFO("kangan \t Pub \t %s \t %d", m.c_str(), zo);
				
				myfile<<x<<"\t"<<y<<"\t"<<new_degree<<"\t"<<dir_degree;
				myfile<<std::endl;

				prev_degree = dir_degree;
				
				left_count = left_wheel - left_constant;
				right_count = right_wheel - right_constant;
			}
			else if(p_data == 0)
			{
				if(new_file == 0)
				{
					myfile<<"New key start \n";

					myfile2<<std::endl;
                    myfile2<<"NEW straight start \n";
                    myfile2<<"x \t Direction \t Angular_z \t WHERE";
                    myfile2<<std::endl;
                    new_file = 1;

                    ROS_INFO("living_doc \t ---------------");
					new_file = 1;
				}
				first_time = 0;
				zo=0;
				flag_second = 0;
			}
		}
		else if(flag_first==1)
		{
			left_count = 0;
			right_count = 0;

			flag_first = 2;

			first_time = 0;

			flag_second = 0;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}