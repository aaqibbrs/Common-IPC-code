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

#define PI 3.14159265
// for CM below 2 lines ( change according to pulses (500 or 400))
//double CIRC = 0.07853981; //(2*PI*r/400)/2
//double CIRC = 0.062831853; //(2*PI*r/500)/2


//for meters below 2 lines ( change according to pulses (500 or 400))
//double CIRC = 0.0007853981; //(2*PI*r/400)/2
double CIRC = 0.00062831853; //(2*PI*r/500)/2

float *left_enc_ppr = new float(0.0);
float *right_enc_ppr = new float(0.0);

int flag_first = 0, rig_data = 110;
int i = 0, mmm=0;
int *left_reference = new int(0);

std::ofstream myfile("/home/bros/dist_time.csv",std::ios::out); //change this accordingly

long int left_wheel, right_wheel, left_count, right_count, left_constant, right_constant;

double distance_full, total_distance = 0.0, wiper_length = 0.8;
double ref_start, ref_end;

double time_now, total_time = 0.0;

using namespace std;

string st = "";

void leftcallback(const std_msgs::Int64::ConstPtr& msg)
{
    left_wheel = msg->data;
	if(flag_first == 0 and *left_reference == 0)
	{
		left_constant = left_wheel;
		*left_reference = 1;
	}
}
void rigor_callback(const std_msgs::Int8::ConstPtr& rig_start)
{
	rig_data = rig_start->data;
}
void rightcallback(const std_msgs::Int64::ConstPtr& msg2)
{
  	right_wheel = msg2->data;
	if(flag_first==0 and *left_reference == 1)
	{
		right_constant = right_wheel;

		ros::param::get("/left_enc_ppr",*left_enc_ppr);
		ros::param::get("/right_enc_ppr",*right_enc_ppr);

		if(*left_enc_ppr == 500.0 and *right_enc_ppr == 500.0)
		{
			CIRC = 0.00062831853;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
            flag_first = 1;
		}
		else if(*left_enc_ppr == 400.0 and *right_enc_ppr == 400.0)
		{
			CIRC = 0.0007853981;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
            flag_first = 1;
		}
		else if(*left_enc_ppr == 0.0 or *right_enc_ppr == 0.0)
		{
			ROS_ERROR("DIST_TIME : Encoder ppr not received: Set parameter /left_enc_ppr and /right_enc_ppr");
		}
        else
        {
            CIRC = 0.0007068583462;
            delete left_enc_ppr;
            delete right_enc_ppr;

            delete left_reference;
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
	st = m;
    int u=0;
    for(int k=11; k<19; k++)
    {
        dt[k]=m[u];
        u++;
    }

    myfile<<"Cleaning Start Time : \t"<<dt;
    myfile<<std::endl;
    
	myfile<<" Distance(m) \t Time \n";

	ROS_INFO("\t Cleaning Start time :\t %s", m.c_str());

	ros::init(argc,argv,"brs_dt");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
	ros::NodeHandle n;
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Vector3>("/distance_time", 1);
	ros::Subscriber get_encl=n.subscribe<std_msgs::Int64>("/encoder_l",50,&leftcallback);
	ros::Subscriber get_encr=n.subscribe<std_msgs::Int64>("/encoder_r",50,&rightcallback);

    ros::Subscriber rigorous = n.subscribe<std_msgs::Int8>("/rigor",10, rigor_callback);

	ros::Rate loop_rate(20);

	geometry_msgs::Vector3 msg1;
	while (ros::ok())
	{
		if(flag_first == 2)
		{
			time_t now = time(0);
			char* dt = ctime(&now);

			string t = "";
			for(int z=11; z<19; z++)
			{
				t = t + dt[z];
			}
			string m = converter(t);

			if(rig_data != 110)
			{
				left_count = left_wheel - left_constant - left_count;
				right_count = right_wheel - right_constant - right_count;

				distance_full = CIRC*(left_count+right_count);
                total_distance += distance_full;

                ref_start = ros::Time::now().toSec();
                if(ref_end != -1.0)
                {
                    time_now = ref_start - ref_end;
                }
                else
                {
                    time_now = 0.0;
                }
                total_time += time_now;

				msg1.x = float(total_distance);
				msg1.y = float(total_time);
				cmd_pub.publish(msg1);

				mmm++;
				if(mmm%10 == 0)
				{
					myfile<<total_distance<<"\t"<<m;
					myfile<<std::endl;
					mmm=1;

					ROS_DEBUG("\t \t \t %s \t %f", m.c_str(), float(total_distance));
				}
				
				left_count = left_wheel - left_constant;
				right_count = right_wheel - right_constant;

                ref_end = ros::Time::now().toSec();
                i=76;
			}
			else
			{
				left_constant = left_wheel;
				right_constant = right_wheel;
				left_count = 0;
				right_count = 0;

				mmm=8;
                i++;
                if(i%80 == 0)
                {
                    msg1.x = float(total_distance);
                    msg1.y = float(total_time);
                    cmd_pub.publish(msg1);

                    myfile<<total_distance<<"\t"<<m;
                    myfile<<std::endl;

					ROS_DEBUG("\t \t \t %s \t %f", m.c_str(), float(total_distance));

					i=1;
                }
                ref_end = -1.0;
			}		
		}
		else if(flag_first==1)
		{
			left_count = 0;
			right_count = 0;

			flag_first = 2;

            ref_end = -1.0;
		}
		ros::spinOnce();
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

		float area = total_distance * wiper_length;

		string start = "abcde", end = "abcde";
		start[0] = st.at(0); start[1]=st.at(1); end[0] = m.at(0); end[1] = m.at(1);

		start[3] = st.at(3); start[4]=st.at(4); end[3] = m.at(3); end[4] = m.at(4);

		start[2] = '.'; end[2] = '.';
		
		stringstream ss, ee; float sta, en;
		ss<<start; ss>>sta; ee<<end; ee>>en;
		
		float diff = en - sta;
		if(diff>=0.60000)
		{
			if(int(diff) == 0)
			{
				diff = diff - 0.40;
			}
			else if(diff - int(diff) < 0.6000)
			{
				diff = diff - int(diff)*0.40;
			}
			else
			{
				diff = diff - (int(diff)+1)*0.40;
			}
		}
		else if(start[1] != end[1])
		{
			diff = int(sta) - (sta) + en - int(en) + 0.60 ;
		}
		
		diff = diff*100;
		
		int hour = int(diff)/60;
		int minutes = int(diff) % 60;

		myfile<<"Cleaning End Time : \t"<<dt<<std::endl;
		myfile<<"Total area covered : \t"<<area<<std::endl;
		myfile<<"Cleaning Time : \t"<<hour<<" hours \t"<<minutes<<" minutes"<<std::endl;
		myfile<<"Total distance covered : \t"<<total_distance<<" m"<<std::endl;

		ROS_INFO("\t Cleaning End time :\t %s", m.c_str());
		ROS_INFO("Total distance covered %f", float(total_distance));
		ROS_INFO("Total Area covered %f", area);

		cout<<"\t \t \t Cleaning Start time : "<<st<<endl;
		cout<<"\t \t \t Cleaning End time : "<<m<<endl<<endl;
		cout<<"\t \t \t "<<"Cleaning time : "<<hour<<" hours and "<<minutes<<" minutes"<< endl<<endl;
		cout<<"\t \t \t Total distance covered : "<<total_distance<<" m"<<endl;
		cout<<"\t \t \t Total Area covered : "<<area<<" square meters"<<endl;

	}
	ros::spin();
	return 0;
}