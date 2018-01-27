#ifndef COST_MAP_EXERCISE_NODE_H
#define COST_MAP_EXERCISE_NODE_H

// ROS
#include <ros/ros.h>
// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>
// TF
#include <tf/transform_listener.h>
// NaN detection
#include <math.h>

/* our node's namespace */
namespace cost_map_exercise {
  /* our node's class */
  class CostMapExercise {
    /* class private variable-members */
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;
    ros::Publisher publisher;

    /* callback function */
    void topicCallback(const grid_map_msgs::GridMap& map_msg) {
      ROS_INFO("Here I am!!!\n");
      grid_map::GridMap map;
      grid_map::GridMapRosConverter::fromMessage(map_msg, map);

      for (std::vector<std::string>::const_iterator i = map.getLayers().begin(); i != map.getLayers().end(); ++i)
        std::cout << *i << ' ';

      grid_map::Matrix& trav_data = map["traversability"];
      grid_map::Matrix& uncert_data = map["uncertainty_range"];

      std::cout << std::endl << "length_x = " << map.getLength().x() << ", length_y = " << map.getLength().y() << ", size_0 = " << map.getSize()[0] << ", size_1 = " << map.getSize()[1] << ", resolution = " << map.getResolution() << std::endl;
      std::cout << "trav_data_size = " << trav_data.size() << "uncert_data_size" << uncert_data.size()  << std::endl;

      double cost;

      for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const int i = iterator.getLinearIndex();
        if (!std::isnan(trav_data(i))) ROS_INFO("i = %d: trav_data = %f uncert_data = %f\n", i, trav_data(i), uncert_data(i));

        grid_map::Position position;
        map.getPosition(*iterator, position);

        if (trav_data(i) < 0) {
          double sum = 0;
          int counter = 0;
          double windowRadius = 3.0*map.getResolution();
          for (grid_map::CircleIterator circleIterator(map, position, windowRadius); !circleIterator.isPastEnd(); ++circleIterator) {
            sum += map.at("traversability", *circleIterator);
            counter++;
          }

          cost = sum / counter;
        }
        else
          cost = trav_data(i);

        // if (!std::isnan(trav_data(i))) ROS_INFO("(%f, %f) %f\n", position.x(), position.y(), cost);

        ros::Duration duration(0.01);
        duration.sleep();
      }

      ROS_INFO("I am done!\n");
    }

  public:
    /* class constructor */
    CostMapExercise(ros::NodeHandle& nodeHandle) : nodeHandle(nodeHandle) {
      ROS_INFO("A\n");
      std::string topic;
      int queue_size;

      /* get parameters */
      if (!nodeHandle.getParam("topic", topic)) ROS_ERROR("Could not find topic parameter!");
      if (!nodeHandle.getParam("queue_size", queue_size)) ROS_ERROR("Could not find queue_size parameter!");
      ROS_INFO("B\n");

      this->subscriber = nodeHandle.subscribe(topic, queue_size, &CostMapExercise::topicCallback, this);
      // this->publisher = nodeHandle.advertise<...>("...", ?);
      ROS_INFO("C\n");
    }
    /* class destructor */
    virtual ~CostMapExercise() {}
  };
}

#endif
