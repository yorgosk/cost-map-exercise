#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_

// ROS
#include <ros/ros.h>
// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
// Cost Map
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
// NaN detection and absolute values
#include <math.h>

namespace grid_layer {

  class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D {
  protected:
    /* class private variable-members */
    ros::NodeHandle nodeHandle_;
    ros::Subscriber subscriber_;
    /* our traversability map, for easy access from any function */
    grid_map::GridMap map_;
    /* callback function */
    virtual void topicCallback(const grid_map_msgs::GridMap& map_msg);
    bool new_map_;

  public:
    GridLayer() { layered_costmap_ = NULL; };
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized() { return true; }
    virtual void matchSize();
  };
}

#endif
