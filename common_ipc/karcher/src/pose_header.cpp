#include "pose.h"
#include "debug_sensor.h"

N::debugging_next pose_header;

P::pose::pose():  first_time(0),
        flag_first(0),

        pose_end_time(0),
        left_wheel(0), right_wheel(0), left_constant(0), right_constant(0), left_count(0), right_count(0),
        CIRC(0),

        left_enc_ppr(0),
        right_enc_ppr(0),

        direction(0),
        new_degree(0), prev_degree(0), ss_degree(0), first_degree(0), dir_degree(0),

        x(0), y(0),

        time_diff(0)
{
    enc_callback_left = nh.subscribe("/encoder_l", 10, &P::pose::enc_left, this);
    enc_callback_right = nh.subscribe("/encoder_r", 10, &P::pose::enc_right, this);

    imu_callback = nh.subscribe("/brick_imu", 10, &P::pose::imu_data, this);

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

}
void P::pose::enc_left(const std_msgs::Int64& msg)
{
    left_wheel = msg.data;
}
void P::pose::enc_right(const std_msgs::Int64& msg)
{
    right_wheel = msg.data;
}
void P::pose::imu_data(const geometry_msgs::Vector3& dat)
{
    direction = dat.x;
    if(flag_first == 0)
	{
		ros::param::get("/left_enc_ppr",*left_enc_ppr);
		ros::param::get("/right_enc_ppr",*right_enc_ppr);

		if(*left_enc_ppr == 500.0 and *right_enc_ppr == 500.0)
		{
			CIRC = 0.062831853;
            delete left_enc_ppr;
            delete right_enc_ppr;

            flag_first = 1;
		}
		else if(*left_enc_ppr == 400.0 and *right_enc_ppr == 400.0)
		{
			CIRC = 0.07853981;
            delete left_enc_ppr;
            delete right_enc_ppr;

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

            flag_first = 1;
        }
	}
}
geometry_msgs::Twist P::pose::pose_data(int start, int resuming, int paused, int stop)
{
    geometry_msgs::Twist pose_;

    int allowed = pose_header.sensor_status(1.0, 1, 1, 0, 0, 0);
    if(allowed)
    {
        if(flag_first==1)
        {
            if(start == 1 and first_time == 0)
            {
                left_constant = left_wheel;
                right_constant = right_wheel;

                left_count = 0;
                right_count = 0;

                x=0;
                y=0;

                first_degree = direction;
                prev_degree = direction;

                first_time = 1;

                time_diff = -1.0;
            }
            if(resuming == 1 and first_time == 1)
            {
                if(time_diff != -1.0)
                {
                    time_diff = (ros::Time::now() - pose_end_time).toSec();
                }
                
                if(time_diff > 0.05)
                {
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

                    pose_.linear.x = x;
                    pose_.linear.y = y;
                    pose_.linear.z = ss_degree;

                    pose_.angular.z = dir_degree;

                    prev_degree = dir_degree;
                        
                    left_count = left_wheel - left_constant;
                    right_count = right_wheel - right_constant;
                }
                pose_end_time = ros::Time::now();
            }
            if(stop == 1)
            {
                first_time = 0;
            }

            return pose_;
        }
        else
        {
            ROS_INFO("Waiting for encoder counts");
        }
    }
    else
    {
        ROS_INFO("Encoder/IMU not working");
        
    }
    
}