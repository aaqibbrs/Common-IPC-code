#include"ros/ros.h"
#include"geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"

std_msgs::Int8 v;
geometry_msgs::Twist m;
ros::Publisher 	mov_cmd;
ros::Publisher dir_cmd;
void cmdCB(const geometry_msgs::Twist::ConstPtr& dat)
{
	m.linear.x = dat->linear.x;
	m.angular.z = dat->angular.z;
	mov_cmd.publish(m);
	ROS_INFO("x:[%f], z:[%f]",m.linear.x,m.angular.z);
}

void dirCB(const std_msgs::Int8::ConstPtr& dat)
{
	v.data = dat->data;
	dir_cmd.publish(v);
	ROS_INFO("Direction [%d]",v.data);
}
	int main(int argc,char **argv)
	{
	ros::init(argc,argv,"karcher_teleop_com_def");
	ros::NodeHandle n;

	mov_cmd = n.advertise<geometry_msgs::Twist>("/mov_cmd",20);
	dir_cmd = n.advertise<std_msgs::Int8>("/dir_cmd",20);
	ros::Subscriber get_cmd = n.subscribe<geometry_msgs::Twist>("/CMD", 20, &cmdCB);
	ros::Subscriber get_dir = n.subscribe<std_msgs::Int8>("/DIR", 20, &dirCB);
	ros::Rate loop_rate(50); // Higher value results in faster update rates
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
