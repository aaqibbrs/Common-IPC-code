#ifndef POSE_CLASS
#define POSE_CLASS

#include <ros/ros.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Int64.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Vector3.h>

#define PI 3.14159265

namespace P
{
    class pose
    {
        private:

            ros::NodeHandle nh;

            int first_time;
            int flag_first;

            ros::Time pose_end_time;

            long int left_wheel, right_wheel, left_constant, right_constant, left_count, right_count;

            double CIRC = 0.062831853, distance_new;

            float *left_enc_ppr = new float(0.0);
            float *right_enc_ppr = new float(0.0);

            float direction;
            float new_degree, prev_degree, ss_degree, first_degree, dir_degree;

            float x, y;

            float time_diff;

            geometry_msgs::Twist msg1;
            ros::Subscriber enc_callback_left;
            ros::Subscriber enc_callback_right;

            ros::Subscriber imu_callback;
            
            ros::Publisher cmd_pub;

            void enc_left(const std_msgs::Int64& msg);
            void enc_right(const std_msgs::Int64& msg);

            void imu_data(const geometry_msgs::Vector3& msg);

        public:

            geometry_msgs::Twist pose_data(int start, int resuming, int paused, int stop);

            void runner(geometry_msgs::Twist start, geometry_msgs::Twist current, geometry_msgs::Twist togo);

            geometry_msgs::Twist path_data(geometry_msgs::Twist start, geometry_msgs::Twist current, geometry_msgs::Twist togo);
            pose();
    };
}

#endif