#include"ros/ros.h"
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

std::ofstream myfile("/home/bros/trial_file_imp.csv",std::ios::out); //change this accordingly

float ss_degree, x = 0.0, y = 0.0, req_x = 0.0;
double sec_start, sec_end;
int flag_first = 0, i = 0, turn = 0, starter = 0, timer = 0, tara=0;
int new_file = 0, pose_ref = 0, ref_safe = 0, allowed = 0, l =0;

double ref_start, ref_end;

float linear = 0.25;

using namespace std;
void starting(const std_msgs::Int8::ConstPtr& str)
{
	starter = str->data;
}
void dataCB(const geometry_msgs::Twist::ConstPtr& dat)
{
    pose_ref = 1;
	x = dat->linear.x;
    y = dat->linear.y;
    ss_degree = dat->angular.x;
}
int main(int argc,char **argv)
{
    if(!myfile)
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

	myfile<<std::endl;
    myfile<<"New file start"<<dt<<std::endl;

    ROS_INFO("living_doc \t X \t ss_degree \t msg.angular.z");
	ros::init(argc,argv,"brs_str");
	ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	ros::Subscriber pose_dir = n.subscribe<geometry_msgs::Twist>("/pose_dir_str", 1, &dataCB);
    ros::Subscriber start = n.subscribe<std_msgs::Int8>("/starting", 10, &starting);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	ros::Rate loop_rate(30);

	geometry_msgs::Twist msg;
	while (ros::ok()) 
	{
        if(allowed == 1)
        {
            if(starter == 1)
            {
                new_file = 0;
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
                    if(flag_first == 0)
                    {
                        req_x = 0.0;
                        flag_first = 1;
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
                            }
                            else if(ss_degree < 359.000 and ss_degree > 310.0)
                            {
                                msg.linear.x = linear;
                                msg.angular.z = -min(((359.0 - ss_degree)/200.0),0.10);
                                ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
                                cmd_pub.publish(msg);
                                ROS_DEBUG("Inside turn right 1A"); //only align
                            }
                            else if(ss_degree < 50.0 and ss_degree > 1.000)
                            {
                                msg.linear.x = linear;
                                msg.angular.z = min(((ss_degree - 1.0)/200.0),0.10);
                                ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
                                cmd_pub.publish(msg);
                                ROS_DEBUG("Inside turn left 1B"); //only align
                            }
                            else
                            {
                                ROS_ERROR("Straight : 1 should not come here");
                            }
                        }
                        else if(x < (req_x-2.0))
                        {
                            if(ss_degree >= 359.000 or ss_degree <= 1.000)
                            {
                                msg.linear.x = linear;
                                //msg.angular.z = -0.05;//need to change this later
                                msg.angular.z = -min(((-2.0 - x)/200.0),0.10);
                                cmd_pub.publish(msg);
                                ROS_DEBUG("Inside turn right 2A");
                            }
                            else if(ss_degree < 359.000 and ss_degree > 310.0)
                            {
                                msg.linear.x = linear;
                                msg.angular.z = -min((((359.0 - ss_degree)/300.0)+((-2.0 - x)/300.0)),0.15);
                                ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
                                cmd_pub.publish(msg);
                                ROS_DEBUG("Inside turn right 2B");
                            }
                            else if(ss_degree < 50.0 and ss_degree > 1.000)
                            {
                                if((req_x - x) >= 30.0)
                                {
                                    msg.linear.x = linear;
                                    //msg.angular.z = -0.10;
                                    msg.angular.z = -min((((ss_degree - 1.0)/100.0)+((-30.0-x)/300.0)),0.15);
                                    cmd_pub.publish(msg);
                                    ROS_DEBUG("Turn right due to 2C");
                                }
                                else if((req_x - x) < 30.0 and (req_x - x) >= 15.0)
                                {
                                    if(ss_degree>20.0)
                                    {
                                        msg.linear.x = linear;
                                        msg.angular.z = 0.0;
                                        cmd_pub.publish(msg);
                                        ROS_DEBUG("Go straight due to 2C");
                                    }
                                    else
                                    {
                                        msg.linear.x = linear;
                                        msg.angular.z = -0.05;
                                        cmd_pub.publish(msg);
                                        ROS_DEBUG("Go right/straight due to 2C");
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

                                    }
                                    else
                                    {
                                        msg.linear.x = linear;
                                        msg.angular.z = 0.0;
                                        ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
                                        cmd_pub.publish(msg);
                                        ROS_DEBUG("Go straight due to IRONIC 2C");
                                    }
                                }
                                else
                                {
                                    ROS_ERROR("Never come here");
                                }                          
                            }
                            else
                            {
                                ROS_ERROR("Straight : 2 should not come here");
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
                                }
                                else if((x - req_x) < 30.0 and (x - req_x) >= 15.0)
                                {
                                    if(ss_degree<340.0)
                                    {
                                        msg.linear.x = linear;
                                        msg.angular.z = 0.0;
                                        cmd_pub.publish(msg);
                                        ROS_DEBUG("Inside straight/straight due to 3B");
                                    }
                                    else
                                    {
                                        msg.linear.x = linear;
                                        msg.angular.z = 0.05;
                                        cmd_pub.publish(msg);
                                        ROS_DEBUG("Inside left/straight due to 3B");
                                    }
                                }
                                else if((x-req_x) < 15.0)
                                {
                                    if(ss_degree < 340.0)
                                    {
                                        msg.linear.x = linear;
                                        msg.angular.z = -min((((359.0 - ss_degree)/400.0)+((x-2.0)/400.0)),0.10);
                                        ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
                                        cmd_pub.publish(msg);
                                        ROS_DEBUG("Go right due to IRONIC 3B");
                                    }
                                    else
                                    {
                                        msg.linear.x = linear;
                                        msg.angular.z = 0.0;
                                        ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
                                        cmd_pub.publish(msg);
                                        ROS_DEBUG("Go straight due to IRONIC 3B");
                                    }
                                }
                                else
                                {
                                    ROS_ERROR(" Straight : 2 Never come here");
                                }
                            }
                            else if(ss_degree < 50.0 and ss_degree > 1.000)
                            {
                                msg.linear.x = linear;
                                msg.angular.z = min((((ss_degree - 1.0)/300.0)+((x-2.0)/300.0)),0.15);
                                ROS_DEBUG(" linear.x [%f] and angular.z is [%f]", msg.linear.x, msg.angular.z);
                                cmd_pub.publish(msg);
                                ROS_DEBUG("Inside turn left due to 3C");
                            }
                            else
                            {
                                ROS_ERROR("Straight : 3 should not come here");
                            }
                        }
                        myfile<<x<<"\t"<<ss_degree<<"\t"<<msg.angular.z;
                        myfile<<std::endl;

                        ROS_INFO("living_doc \t %f \t %f \t %f", x, ss_degree, msg.angular.z);
                    }
                }
            }
            else if(starter != 1)
            {
                if(new_file == 0)
                {
                    myfile<<std::endl;
                    myfile<<"NEW straight start \n";
                    myfile<<"x \t ss_degree \t Angular_z";
                    myfile<<std::endl;
                    new_file = 1;

                    ROS_INFO("living_doc \t ---------------");
                }
                ROS_DEBUG("waiting for starter");
            }
        }
        if(ref_safe == 0)
        {
            ref_start = ros::Time::now().toSec();
            ref_safe = 1;
        }
        if(ref_safe == 1)
        {
            ref_end = ros::Time::now().toSec();
            if(ref_end - ref_start >= 1.000) //increase for more risk
            {
                if(pose_ref == 0)
                {
                    allowed = 0;
                    ROS_DEBUG("No callback received due to either encoder or lidar");
                    if(starter == 1)
                    {
                        msg.linear.x = 0.0;
                        msg.angular.z = 0.0;
                        cmd_pub.publish(msg);
                        ROS_FATAL(" Straight : No pose received for 1 second");
                        l=1;
                    }
                }
                else
                {
                    allowed = 1;
                    if(l==1)
                    {
                        l=0;
                        ROS_ERROR("Not an error : Started receiving data");
                    }
                }
                pose_ref = 0;
                ref_safe = 0;
            }
        }
        ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}