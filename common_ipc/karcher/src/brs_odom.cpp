#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include"sensor_msgs/Imu.h"
#include"geometry_msgs/Vector3.h"
#include "std_msgs/Int64.h"
#include <fstream>
#include <iostream>
#include<cmath>
#include<math.h>

#define PI 3.14159265

double x = 0.0;
double y = 0.0;
double th = 0.0;

double radii;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

double pth, dth;
int *left_reference = new int(0);
int *right_reference = new int(0);

//double CIRC = 0.0007853981; //(2*PI*r/400)/2
double CIRC = 0.00062831853; //(2*PI*r/500)/2

float *left_enc_ppr = new float(0.0);
float *right_enc_ppr = new float(0.0);

int flag_start =0, flag_first = 0, z = 0, fixed_yaw = 0, a = 0, new_val = 0;

long int left_wheel, right_wheel, left_count, right_count;

double distance, diff;

float direction = 0.0, prev_degree = 0.0, new_degree = 0.0, dir_degree = 0.0;

double current_time;
double last_time, dt = 0.0;
ros::Time now_time;

void leftcallback(const std_msgs::Int64::ConstPtr& msg)
{      
	left_wheel = msg->data;
	if(flag_first == 0 and *left_reference == 0)
	{
		left_count = left_wheel;
		*left_reference = 1;
	}
}
void rightcallback(const std_msgs::Int64::ConstPtr& msg2)
{    
  	right_wheel = msg2->data;
	if(flag_first==0 and *right_reference == 0)
	{
		right_count = right_wheel;
		*right_reference = 1;
	}
}
void dataCB(const geometry_msgs::Vector3::ConstPtr& dat)
{
	direction = dat->x;

    new_val = 1;

    if(flag_first == 0 and *left_reference == 1 and *right_reference == 1)
    {
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
			ROS_ERROR(" ODOM : Encoder ppr not received: Set parameter /left_enc_ppr and /right_enc_ppr");
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brs_odom");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("encoder_l",50,&leftcallback);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("encoder_r",50,&rightcallback);
    ros::Subscriber gyro_data = n.subscribe<geometry_msgs::Vector3>("/brick_imu",50,&dataCB);

    tf::TransformBroadcaster odom_broadcaster;

    x = 0.0;
    y = 0.0;
    th = 0.0;

    vx = 0.1;
    vy = -0.1;
    vth = 0.1;

    current_time = ros::Time::now().toSec();
    last_time = ros::Time::now().toSec();

    ros::Rate r(20.0);
    while(n.ok())
    {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now().toSec();

        now_time = ros::Time::now();

        if(flag_first == 1)
	    { 
            flag_first = 2;
            new_val = 0;
	    }
        if(flag_first==2 and new_val == 1)
	    {
            dt = current_time - last_time;

            left_count = left_wheel - left_count;
            right_count = right_wheel - right_count;

			distance = CIRC*(left_count+right_count);

            dir_degree = direction;
			
            if((dir_degree >= 355.0 and prev_degree <= 5.0) or (dir_degree <= 5.0 and prev_degree >= 355.0))
            {
                if(dir_degree >= 355.0)
                {
                    prev_degree = prev_degree + 360.0;

                    new_degree = prev_degree - dir_degree;

                    prev_degree = prev_degree - 360.0;
                }
                else if(prev_degree >=355.0)
                {
                    dir_degree = dir_degree + 360.0;

                    new_degree = prev_degree - dir_degree;

                    dir_degree = dir_degree - 360.0;
                }
            }
            else
            {
                new_degree = prev_degree - dir_degree;
            }

            dth = new_degree*(PI/180.0);

            th = 2*PI - dir_degree*(PI/180.0);
            
            if(a<1)
            {
                dth = 0.0;
                
                a++;
            }

            x = x + distance*cos(th);
            y = y + distance*sin(th);

            if(dt>0.0)
            {
                vy = 0.0;

                radii = distance/(2*sin(fabs(dth)));

                if(dth == 0.0)
                {
                    vth = 0.0;
                    vx = distance/dt;
                }
                else if(dth>0.0)
                {
                    vth = dth/dt;
                    vx = vth*radii;
                }
                else if (dth<0.0)
                {
                    vth = dth/dt;                    
                    vx =  vth*radii;
                }
            }
            else
            {
                vx = 0.0;
                vy = 0.0;
                vth = 0.0;
            }   

            //since all odometry is 6DOF we'll need a quaternion created from yaw

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = now_time;
            odom_trans.header.frame_id = "odom_trans";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = now_time;
            odom.header.frame_id = "odom_first";

            //set the position
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;

            //publish the message
            odom_pub.publish(odom);

            last_time = current_time;

            prev_degree = dir_degree;

            left_count = left_wheel;
            right_count = right_wheel;

            new_val = 0;
        }
        r.sleep();   
    }
}