#include "debug_sensor.h"

N::debugging_next::debugging_next(): ref_safe_class(0),

            ref_start_class(0),
            laser_slant_ref_class(0),
            laser_flat_ref_class(0),
            enc_ref_class(0),
            imu_ref_class(0),
            remem(0),
            to_stop(0),
            
            ref_safe_class_node(0),
            ref_start_class_node(0),
            lw_ref(0),
            rw_ref(0),
            ls_ref(0),
            rs_ref(0),
            remem_node(0)
{
    enc_callback = nh.subscribe("/encoder_l", 10, &N::debugging_next::enc_status, this);

    flat_laser_callback = nh.subscribe("/scan", 10, &N::debugging_next::top_lidar_status, this);
    slant_laser_callback = nh.subscribe("/new_scan_far", 10, &N::debugging_next::slant_lidar_status, this);

    imu_callback = nh.subscribe("/brick_imu", 10, &N::debugging_next::imu_status, this);

    stop_sensor_status = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    lw_callback = nh.subscribe("/lw_work", 10, &N::debugging_next::lw_status, this);
    rw_callback = nh.subscribe("/rw_work", 10, &N::debugging_next::rw_status, this);
    ls_callback = nh.subscribe("/ls_work", 10, &N::debugging_next::ls_status, this);
    rs_callback = nh.subscribe("/rs_work", 10, &N::debugging_next::rs_status, this);

}

void N::debugging_next::top_lidar_status(const sensor_msgs::LaserScan& msg)
{
    laser_flat_ref_class = 1;
}

void N::debugging_next::slant_lidar_status(const sensor_msgs::LaserScan& msg)
{
    laser_slant_ref_class = 1;
}

void N::debugging_next::enc_status(const std_msgs::Int64& msg)
{
    enc_ref_class = 1;
}

void N::debugging_next::imu_status(const geometry_msgs::Vector3& msg)
{
    imu_ref_class = 1;
}

void N::debugging_next::lw_status(const std_msgs::Bool& msg)
{
    lw_ref = 1;
}
void N::debugging_next::rw_status(const std_msgs::Bool& msg)
{
    rw_ref = 1;
}
void N::debugging_next::ls_status(const std_msgs::Bool& msg)
{
    ls_ref = 1;
}
void N::debugging_next::rs_status(const std_msgs::Bool& msg)
{
    rs_ref = 1;
}
int N::debugging_next::sensor_status(float time_diff, int imu, int enc, int top_lidar, int inc_lidar, int to_stop)
{
    if(ref_safe_class == 0)
    {
        ref_start_class = ros::Time::now();
        ref_safe_class = 1;
    }
    if(ref_safe_class == 1)
    {
        float dt = (ros::Time::now() - ref_start_class).toSec();
        if(dt >= time_diff)
        {
            if(imu == 1)
            {
                if(imu_ref_class == 0)
                {
                    if(to_stop == 1)
                    {
                        msg1.linear.x = 0.0;
                        msg1.angular.z = 0.0;
                        stop_sensor_status.publish(msg1);
                        ROS_FATAL("Motion Stopped because IMU stopped");
                    }
                    
                    ref_safe_class = 0;
                    remem = 0;
                    return 0;
                }
                else
                {
                    imu_ref_class = 0;
                }
            }
            if(enc == 1)
            {
                if(enc_ref_class == 0)
                {
                    if(to_stop == 1)
                    {
                        msg1.linear.x = 0.0;
                        msg1.angular.z = 0.0;
                        stop_sensor_status.publish(msg1);
                        ROS_FATAL("Motion Stopped because ENCODER stopped");
                    }

                    ref_safe_class = 0;
                    remem = 0;
                    return 0;
                }
                else
                {
                    enc_ref_class = 0;
                }
            }
            if(top_lidar == 1)
            {
                if(laser_flat_ref_class == 0)
                {
                    if(to_stop == 1)
                    {
                        msg1.linear.x = 0.0;
                        msg1.angular.z = 0.0;
                        stop_sensor_status.publish(msg1);
                        ROS_FATAL("Motion Stopped because SCAN stopped");
                    }

                    ref_safe_class = 0;
                    remem = 0;
                    return 0;
                }
                else
                {
                    laser_flat_ref_class = 0;
                }
            }
            if(inc_lidar == 1)
            {
                if(laser_slant_ref_class == 0)
                {
                    if(to_stop == 1)
                    {
                        msg1.linear.x = 0.0;
                        msg1.angular.z = 0.0;
                        stop_sensor_status.publish(msg1);
                        ROS_FATAL("Motion Stopped because NEW SCAN SLANT stopped");
                    }

                    ref_safe_class = 0;
                    remem = 0;
                    return 0;
                }
                else
                {
                    laser_slant_ref_class = 0;
                }
            }

            ref_safe_class = 0;
            remem = 1;
            return 1;
        }
        else
        {
            return remem;
        }
    }
}
int N::debugging_next::node_status(float time_diff, int lw, int rw, int ls, int rs)
{
    if(ref_safe_class_node == 0)
    {
        ref_start_class_node = ros::Time::now();
        ref_safe_class_node = 1;
    }
    if(ref_safe_class_node == 1)
    {
        float dt = (ros::Time::now() - ref_start_class_node).toSec();
        if(dt >= time_diff)
        {
            if(lw == 1)
            {
                if(lw_ref == 0)
                {
                    msg1.linear.x = 0.0;
                    msg1.angular.z = 0.0;
                    stop_sensor_status.publish(msg1);
                    ROS_FATAL("Motion Stopped because Left wall node crashed or not started");
                    
                    ref_safe_class_node = 0;
                    remem_node = 0;
                    return 0;
                }
                else
                {
                    lw_ref = 0;
                }
            }
            if(rw == 1)
            {
                if(rw_ref == 0)
                {
                    msg1.linear.x = 0.0;
                    msg1.angular.z = 0.0;
                    stop_sensor_status.publish(msg1);
                    ROS_FATAL("Motion Stopped because RIGHT wall node crashed or not started");

                    ref_safe_class_node = 0;
                    remem_node = 0;
                    return 0;
                }
                else
                {
                    rw_ref = 0;
                }
            }
            if(ls == 1)
            {
                if(ls_ref == 0)
                {
                    msg1.linear.x = 0.0;
                    msg1.angular.z = 0.0;
                    stop_sensor_status.publish(msg1);
                    ROS_FATAL("Motion Stopped because Right step node crashed or not started");

                    ref_safe_class_node = 0;
                    remem_node = 0;
                    return 0;
                }
                else
                {
                    ls_ref = 0;
                }
            }
            if(rs == 1)
            {
                if(rs_ref == 0)
                {
                    msg1.linear.x = 0.0;
                    msg1.angular.z = 0.0;
                    stop_sensor_status.publish(msg1);
                    ROS_FATAL("Motion Stopped because RIGHT step node crashed or not started");

                    ref_safe_class_node = 0;
                    remem_node = 0;
                    return 0;
                }
                else
                {
                    rs_ref = 0;
                }
            }

            ref_safe_class_node = 0;
            remem_node = 1;
            return 1;
        }
        else
        {
            return remem_node;
        }
    }
}