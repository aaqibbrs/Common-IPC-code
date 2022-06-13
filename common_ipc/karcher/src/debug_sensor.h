#ifndef MY_CLASS
#define MY_CLASS

#include <ros/ros.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Int64.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

namespace N
{
    class debugging_next
    {
        private:

            ros::NodeHandle nh;

            int ref_safe_class;
            ros::Time ref_start_class;
            int laser_slant_ref_class;
            int laser_flat_ref_class;
            int enc_ref_class;

            int imu_ref_class;

            int remem;
            int to_stop;
            int ref_safe_class_node;
            ros::Time ref_start_class_node;
            int lw_ref;
            int rw_ref;
            int ls_ref;

            int rs_ref;

            int remem_node;

            geometry_msgs::Twist msg1;
            ros::Subscriber enc_callback;

            ros::Subscriber imu_callback;
            
            ros::Subscriber flat_laser_callback;
            ros::Subscriber slant_laser_callback;

            ros::Publisher stop_sensor_status;

            ros::Subscriber lw_callback;
            ros::Subscriber rw_callback;
            ros::Subscriber ls_callback;
            ros::Subscriber rs_callback;

            void enc_status(const std_msgs::Int64& msg);

            void top_lidar_status(const sensor_msgs::LaserScan& msg);
            void slant_lidar_status(const sensor_msgs::LaserScan& msg);

            void imu_status(const geometry_msgs::Vector3& msg);

            void lw_status(const std_msgs::Bool& msg);
            void rw_status(const std_msgs::Bool& msg);
            void ls_status(const std_msgs::Bool& msg);
            void rs_status(const std_msgs::Bool& msg);

        public:
            int sensor_status(float time_diff, int imu, int enc, int top_lidar, int inc_lidar, int to_stop);
            int node_status(float time_diff, int lw, int rw, int ls, int rs);
            debugging_next();
    };
}

#endif