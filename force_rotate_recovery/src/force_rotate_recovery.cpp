#include <force_rotate_recovery/force_rotate_recovery.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(force_rotate_recovery::ForceRotateRecovery, nav_core::RecoveryBehavior);

namespace force_rotate_recovery {
    ForceRotateRecovery::ForceRotateRecovery() : 
        global_costmap_(NULL), 
        local_costmap_(NULL), 
        initialized_(false), 
        world_model_(NULL) 
    {

    } 

    void ForceRotateRecovery::initialize(std::string name, tf::TransformListener* tf,
                                         costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
        if(!initialized_){
            name_ = name;
            global_costmap_ = global_costmap;
            local_costmap_ = local_costmap;

            //get some parameters from the parameter server
            ros::NodeHandle private_nh("~/" + name_);
            ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

            //we'll simulate every degree by default
            private_nh.param("sim_granularity", sim_granularity_, 0.017);
            private_nh.param("frequency", frequency_, 20.0);

            blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
            blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
            blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
            blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

            world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

            initialized_ = true;
        }else{
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }
    
    void ForceRotateRecovery::runBehavior(){
        if(!initialized_){
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        if(global_costmap_ == NULL || local_costmap_ == NULL){
            ROS_ERROR("The costmaps passed to the ForceRotateRecovery object cannot be NULL. Doing nothing.");
            return;
        }
        
        ROS_WARN("rotate recovery behavior -f started.");

        ros::Rate r(frequency_);
        ros::NodeHandle n;
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
      
        tf::Stamped<tf::Pose> global_pose;
        local_costmap_->getRobotPose(global_pose);

        double current_angle = -1.0 * M_PI;

        bool got_180 = false;

        double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
        while(n.ok()){
            local_costmap_->getRobotPose(global_pose);

            double norm_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
            current_angle = angles::normalize_angle(norm_angle + start_offset);

            //compute the distance left to rotate
            double dist_left = 4 * M_PI - current_angle;

            double x = global_pose.getOrigin().x(), y = global_pose.getOrigin().y();

            //check if that velocity is legal by forward simulating
            double sim_angle = 0.0;
            while(sim_angle < dist_left){
                double theta = tf::getYaw(global_pose.getRotation()) + sim_angle;

                double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
                if(footprint_cost < 0.0){
                    ROS_WARN("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f", footprint_cost);
                }

                sim_angle += sim_granularity_;
            }

            double vel = sqrt(2 * acc_lim_th_ * dist_left);

            vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = vel;

            vel_pub.publish(cmd_vel);

            if(current_angle < 0.0)
              got_180 = true;

            if(got_180 && current_angle >= (0.0 - tolerance_))
                return;

            r.sleep();
      }
    }
};

