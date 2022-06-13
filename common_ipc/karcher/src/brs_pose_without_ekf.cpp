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

using namespace std;

int zo = 0;

#define PI 3.14159265

float direction;
int flag_first = 0, p_data = 0, new_file = 0, first_time = 0;

int *left_reference = new int(0);
int *right_reference = new int(0);

//double CIRC = 0.0007853981; //(2*PI*r/400)/2
double CIRC = 0.00062831853; //(2*PI*r/500)/2

float *left_enc_ppr = new float(0.0);
float *right_enc_ppr = new float(0.0);

std::ofstream myfile("/home/bros/lino_dir_without_ekf.csv",std::ios::out | std::ios::app); //change this accordingly

long int left_wheel, right_wheel, left_count, right_count, left_constant, right_constant;

double distance_new, prev_degree, dir_degree, new_degree, first_degree;

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
			CIRC = 0.00062831853;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
		    delete right_reference;
            flag_first = 1;
		}
		else if(*left_enc_ppr == 400.0 and *right_enc_ppr == 400.0)
		{
			CIRC = 0.0007853981;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
		    delete right_reference;
            flag_first = 1;
		}
		else if(*left_enc_ppr == 0.0 or *right_enc_ppr == 0.0)
		{
			ROS_ERROR(" POSE_WITHOUT_EKF : Encoder ppr not received: Set parameter /left_enc_ppr and /right_enc_ppr");
		}
        else
        {
            CIRC = 0.0007068583462;
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

	ros::init(argc,argv,"brs_pose_without_ekf");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	ros::NodeHandle n;
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/pose_dir_without_ekf", 1);
	ros::Subscriber gyro_data = n.subscribe<geometry_msgs::Vector3>("/brick_imu", 1,&dataCB);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l", 1,&leftcallback);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r", 1,&rightcallback);

	ros::Rate loop_rate(20);

	geometry_msgs::Twist msg1;

	while (ros::ok()) 
	{
		ros::spinOnce();
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

				msg1.linear.x = float(x);
				msg1.linear.y = float(y);
				msg1.linear.z = new_degree;
				msg1.angular.z = dir_degree;

				msg1.angular.y = float(zo);
				cmd_pub.publish(msg1);

				time_t now = time(0);
				char* dt = ctime(&now);

				string t = "";
				for(int z=11; z<19; z++)
				{
					t = t + dt[z];
				}
				string m = converter(t);

				//ROS_INFO("kangan \t Pub \t %s \t %d", m.c_str(), zo);
				
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
					new_file = 1;
				}
                p_data = 1;
				first_time = 0;
			}
		}
		else if(flag_first==1)
		{
			left_count = 0;
			right_count = 0;

			flag_first = 2;
		}
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}