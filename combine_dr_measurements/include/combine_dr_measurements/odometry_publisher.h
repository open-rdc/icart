#ifndef ODOMETRY_PUBLISHER_H_
#define ODOMETRY_PUBLISHER_H_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

namespace combine_dr_measurements{

    class OdometryPublisher{
    public:
        OdometryPublisher(ros::NodeHandle &nh);
        void syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu);
        void run();

    private:
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;
        ros::Publisher odom_pub_;

        nav_msgs::Odometry received_odom_;
        sensor_msgs::Imu received_imu_;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
        message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
        message_filters::Synchronizer<SyncPolicy> sync_;
        //message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu> sync_msgs_;
        ros::NodeHandle &nh_;
        double max_update_rate_;
    };

};

#endif //ODOMETRY_PUBLISHER_H_
