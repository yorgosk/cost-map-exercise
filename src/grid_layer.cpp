#include <../include/simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(grid_layer::GridLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace grid_layer {

  void GridLayer::onInitialize() {
    ROS_INFO("initializing nodeHandle\n");
    nodeHandle_ = ros::NodeHandle("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    // dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nodeHandle_);
    // dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&GridLayer::reconfigureCB, this, _1, _2);
    // dsrv_->setCallback(cb);
    ROS_INFO("A\n");

    subscriber_ = nodeHandle_.subscribe("/traversability_estimation/traversability_map", 10, &GridLayer::topicCallback, this);
    ROS_INFO("B\n");
  }

  void GridLayer::topicCallback(const grid_map_msgs::GridMap& map_msg) {
    ROS_INFO("Here I am!!!\n");
    grid_map::GridMapRosConverter::fromMessage(map_msg, map_);
    ROS_INFO("I am done!\n");
  }

  void GridLayer::matchSize() {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
  }

  /*
    Main idea:
    traversability: [-5, -4] (and uncertainty range: [0, 0.4] ) --> FREE_SPACE
    traversability: (-4, -3.5] (and uncertainty range: (0.4, 1.0) ) --> INSCRIBED_INFLATED_OBSTACLE
    traversability: (-3.5, -2.1]  (and uncertainty range: [1.0, +inf)  ) --> LETHAL_OBSTACLE
    traversability: anything else (and uncertainty range: anything else ) --> NO_INFORMATION

    We will also need to convert between two different 2D coordinate systems, like they do here:
    https://gamedev.stackexchange.com/questions/32555/how-do-i-convert-between-two-different-2d-coordinate-systems
    core idea:
    xRatio = (Math.abs(srcMax-srcMin))/(Math.abs(destMax-destMin));
    destX = x*xRatio+destMin;
  */

  /* My modified update bounds */
  void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {
    if (!enabled_) return;

    /* get traversability map's "traversability" and "uncertainty" layers */
    grid_map::Matrix& trav_data = map_["traversability"];
    grid_map::Matrix& uncert_data = map_["uncertainty_range"];
    /* get traversability map's resolution and dimensions in meters and cells */
    double trav_map_res = map_.getResolution(), trav_map_met_x = map_.getLength().x(), trav_map_met_y = map_.getLength().y();
    int trav_map_cells_y = map_.getSize()[0], trav_map_cells_x = map_.getSize()[1], trav_data_size = trav_data.size();
    /* get costmap's resolution and dimensions in meters and cells */
    double cost_map_res = getResolution(), cost_map_met_x = getSizeInMetersX(), cost_map_met_y = getSizeInMetersY();
    int cost_map_cells_x = getSizeInCellsX(), cost_map_cells_y = getSizeInCellsY();
    /* the map coordinates for which we will determine a cost */
    unsigned int mx, my;
    /* the world coordinates for which we will determine a cost */
    double mark_x, mark_y;
    /* cost variable */
    unsigned char cost;
    /* ratios to help as with conversion -- xRatio = (Math.abs(srcMax-srcMin))/(Math.abs(destMax-destMin)); */
    double trav_max_x = trav_map_met_x, trav_min_x = 0, trav_max_y = trav_map_met_y, trav_min_y = 0,
           cost_max_x = cost_map_met_x, cost_min_x = 0, cost_max_y = cost_map_met_y, cost_min_y = 0,
           x_ratio = std::abs(trav_max_x - trav_min_x) / std::abs(cost_max_x - cost_min_x), y_ratio = std::abs(trav_max_y - trav_min_y) / std::abs(cost_max_y - cost_min_y);


    /* iterate traversability map, while calculating costs */
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
        /* get linear index */
        const int i = iterator.getLinearIndex();

        /* if it is not NaN, determine cost */
        if (!std::isnan(trav_data(i))) {
          /* apply our heuristic rule */
          if (trav_data(i) <= -4 && uncert_data(i) <= 0.4) {       // traversability: [-5, -4] (and uncertainty range: [0, 0.4] ) --> FREE_SPACE
            cost = FREE_SPACE;
          }
          else if (trav_data(i) < -3.5 && uncert_data(i) < 1.0) {  // traversability: (-4, -3.5] (and uncertainty range: (0.4, 1.0) ) --> INSCRIBED_INFLATED_OBSTACLE
            cost = INSCRIBED_INFLATED_OBSTACLE;
          }
          else if (trav_data(i) <= -2.1 && uncert_data(i) >= 1) {  // traversability: (-3.5, -2.1]  (and uncertainty range: [1.0, +inf)  ) --> LETHAL_OBSTACLE
            cost = LETHAL_OBSTACLE;
          }
          else {                                                   // traversability: anything else (and uncertainty range: anything else ) --> NO_INFORMATION
            cost = NO_INFORMATION;
          }
        }
        /* if it is Nan, consider it as NO_INFORMATION */
        else {
          cost = NO_INFORMATION;
        }

        /* position from traversability map's world to costmap's world */
        int trav_data_x = (i+1 / trav_map_cells_x) - 1, trav_data_y = (i+1 % trav_map_cells_x) - 1;
        double trav_data_world_x = trav_data_x * trav_map_res, trav_data_world_y = trav_data_y * trav_map_res;
        /* destX = x*xRatio+destMin; */
        mark_x = trav_data_world_x * x_ratio - cost_min_x;
        mark_y = trav_data_world_y * x_ratio - cost_min_y;

        /* set cost to map */
        if(worldToMap(mark_x, mark_y, mx, my)) {
          setCost(mx, my, cost);
        }

        /* in the end we will have our desired bounds */
        *min_x = std::min(*min_x, mark_x);
        *min_y = std::min(*min_y, mark_y);
        *max_x = std::max(*max_x, mark_x);
        *max_y = std::max(*max_y, mark_y);
    }
  }

  void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) return;

    for (int j = min_j; j < max_j; j++) {
      for (int i = min_i; i < max_i; i++) {
        int index = getIndex(i, j);
        if (costmap_[index] == NO_INFORMATION)
          continue;
        master_grid.setCost(i, j, costmap_[index]);
      }
    }
  }

} // end namespace
