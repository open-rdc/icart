#include <combine_dr_measurements/odometry_publisher.h>

#include <complex>

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

namespace combine_dr_measurements{

    OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh) :
        nh_(nh),
        odom_sub_(nh, "odom", 1),
        imu_sub_(nh, "imu", 1),
        sync_(SyncPolicy(10), odom_sub_, imu_sub_)
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("combined_odom", 5, this);

        sync_.registerCallback(boost::bind(&OdometryPublisher::syncMsgsCB, this, _1, _2));
        ros::NodeHandle private_nh("~");
        private_nh.param<double>("max_update_rate", max_update_rate_, 50);
    }

    void OdometryPublisher::syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu){
        received_odom_ = *odom;
        received_imu_ = *imu;
    }
        
    void OdometryPublisher::run(){
        ros::Rate r(max_update_rate_);
        tf::TransformBroadcaster odom_broadcaster;
        nav_msgs::Odometry old_odom;
        
        while(nh_.ok()){
            if(received_odom_.header.stamp.toSec() > 0.001 && old_odom.header.stamp.toSec() > 0.001){
                ros::Time time = ros::Time::now();
                geometry_msgs::TransformStamped odom_trans;
                nav_msgs::Odometry odom = received_odom_;
                sensor_msgs::Imu imu = received_imu_;

                odom.header.stamp    = odom_trans.header.stamp = time;
                odom.header.frame_id = odom_trans.header.frame_id = "odom";
                odom.child_frame_id  = odom_trans.child_frame_id  = "base_link";
               
                //const double diff = std::abs((odom.header.stamp - imu.header.stamp).toSec());
                //ROS_INFO_STREAM("diff = " << diff);
                
                const double dt = (odom.header.stamp - old_odom.header.stamp).toSec();
                if(dt < 0.001){
                    ROS_WARN_STREAM("Interval too small to integrate with");
                }else{
                                                        
                    odom.pose.pose.orientation = odom_trans.transform.rotation = imu.orientation;
                    
                    tf::Quaternion old_q(
                        old_odom.pose.pose.orientation.x,
                        old_odom.pose.pose.orientation.y,
                        old_odom.pose.pose.orientation.z,
                        old_odom.pose.pose.orientation.w
                    );

                    tf::Quaternion q(
                        odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w
                    );

                    double roll, pitch, yaw, old_roll, old_pitch, old_yaw;
                    tf::Matrix3x3(old_q).getRPY(old_roll, old_pitch, old_yaw);
                    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    const double linear = odom.twist.twist.linear.x * dt;
                    const double angular = yaw - old_yaw;
                    
                    if(std::abs(angular) < 10e-3){
                        double direction = old_yaw + angular * 0.5;
                        odom.pose.pose.position.x = old_odom.pose.pose.position.x + linear * cos(direction);
                        odom.pose.pose.position.y = old_odom.pose.pose.position.y + linear * sin(direction);
                    }else{
                        const double r = linear / angular;
                        odom.pose.pose.position.x = old_odom.pose.pose.position.x + r * (sin(yaw) - sin(old_yaw));
                        odom.pose.pose.position.y = old_odom.pose.pose.position.y - r * (cos(yaw) - cos(old_yaw));
                    }

                    odom_trans.transform.translation.x = odom.pose.pose.position.x;
                    odom_trans.transform.translation.y = odom.pose.pose.position.y;
                    odom_trans.transform.translation.z = odom.pose.pose.position.z;

                    odom_broadcaster.sendTransform(odom_trans);
                    odom_pub_.publish(odom);
                       
                    old_odom = odom;
                }

            }else{
                ROS_INFO("Wait until receive messages.");
                old_odom = received_odom_;
            }
            
            ros::spinOnce();
            r.sleep();
        }
    }
}; //namespace combine_dr_measurements

int main(int argc, char *argv[]){
    ros::init(argc, argv, "combine_dr_measurements");

    ros::NodeHandle n;
    combine_dr_measurements::OdometryPublisher odom_publisher(n);
    odom_publisher.run();

}

