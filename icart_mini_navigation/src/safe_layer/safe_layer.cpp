#include "safe_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(safe_layer_namespace::SafeLayer, costmap_2d::Layer)

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    namespace safe_layer_namespace
{

    SafeLayer::SafeLayer() {}

    void SafeLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                &SafeLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void SafeLayer::matchSize()
    {
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
    }

    void SafeLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_=config.enabled;
    }

    void SafeLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if(!enabled_)
            return;

        double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
        unsigned int mx;
        unsigned int my;
        // if(worldToMap(mark_x, mark_y, mx, my))
        //     setCost(mx, my, LETHAL_OBSTACLE);
        
        *min_x = std::min(*min_x, mark_x_);
        *min_y = std::min(*min_y, mark_y_);
        *max_x = std::max(*max_x, mark_x_);
        *max_y = std::max(*max_y, mark_y_);
    }

    void SafeLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {   
        if (!enabled_)
            return;

        for(int j = min_j; j < max_j; j++)
        {
            for(int i = min_i; i < max_i; i++)
            {
                 if(master_grid.getCost(i, j) != FREE_SPACE)
                 {
                     master_grid.setCost(i, j, FREE_SPACE);
                     continue;
                 }
                master_grid.setCost(i, j, NO_INFORMATION);
            }
        }
        // for (int j = min_j; j < max_j; j++)
        // {
        //     for (int i = min_i; i < max_i; i++)
        //     {
        //         int index = getIndex(i, j);
        //         if (costmap_[index] == NO_INFORMATION)
        //             continue;
        //         master_grid.setCost(i, j, LETHAL_OBSTACLE);
        //     }
        // }

    }
}// end namespace


