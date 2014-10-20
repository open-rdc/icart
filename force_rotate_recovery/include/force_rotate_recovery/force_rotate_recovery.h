#ifndef FORCE_ROTATE_RECOVERY_H
#define FORCE_ROTATE_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

namespace force_rotate_recovery{
    class ForceRotateRecovery : public nav_core::RecoveryBehavior {
    public:
        ForceRotateRecovery();
        ~ForceRotateRecovery(){ delete world_model_; }
        void initialize(std::string name, tf::TransformListener* tf, 
                        costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

        void runBehavior();

    private:
        costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
        costmap_2d::Costmap2D costmap_;
        std::string name_;
        bool initialized_;
        double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
        base_local_planner::CostmapModel* world_model_;
    };
};

#endif  //FORCE_ROTATE_RECOVERY_H
