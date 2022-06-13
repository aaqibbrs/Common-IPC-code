#include"ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

float bthere_x = 0.0, bthere_z = 0.0;

ros::Publisher mov_cmd;
geometry_msgs::Twist cmd;

std_msgs::Int8 rig;


char c;

float S = 0.25;
float thresh2=0.15, thresh1=0.0285; //teleop_twist

void dataCB(const geometry_msgs::Twist::ConstPtr& dat)
{
	bthere_x = dat->linear.x;
    bthere_z = dat->angular.z;
    if(bthere_x > 0.0)
    {
        if(bthere_z==0.0)
        {
		cmd.linear.x=S;
		cmd.angular.z=0.0;
		mov_cmd.publish(cmd);
        }
    }
    else if (bthere_x == 0.0 && bthere_z < 0.0 )	//Right
    {
	cmd.linear.x=thresh1;
	cmd.angular.z=-thresh2;
	mov_cmd.publish(cmd);
    }
    else if(bthere_x == 0.0 && bthere_z > 0.0)	//Left
    {
	cmd.linear.x=thresh1;
	cmd.angular.z=thresh2;
	mov_cmd.publish(cmd);
    }
    else if(bthere_x < 0.0)
    {
        if(bthere_z==0.0)
        {
		cmd.linear.x=-S;
		cmd.angular.z=0.0;
		mov_cmd.publish(cmd);
        }
        else
        {
		cmd.linear.x=0.0;
		cmd.angular.z=0.0;
		mov_cmd.publish(cmd);
        }
    }
    else if(bthere_x == 0.0 and bthere_z == 0.0)
    {
		cmd.linear.x=0.0;
		cmd.angular.z=0.0;
		mov_cmd.publish(cmd);
    }
    else
    {
		cmd.linear.x=0.0;
		cmd.angular.z=0.0;
		mov_cmd.publish(cmd);
    }
    
}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"bthere_remap");
	ros::NodeHandle n;

	mov_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    ros::Subscriber cmd_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel/teleop", 50, &dataCB);

	ros::Rate loop_rate(20);

	while (ros::ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}