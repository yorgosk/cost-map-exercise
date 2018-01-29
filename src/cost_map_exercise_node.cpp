#include "../include/header.hpp"

/* our node's main function */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cost_map_exercise");

  // ros::NodeHandle nodeHandle("~");
  // cost_map_exercise::CostMapExercise costMapExercise(nodeHandle);

  printf("creating gridLayer\n");
  cost_map_exercise::GridLayer gridLayer();

  ros::spin();

  return 0;
}
