#include "../include/header.h"

/* our node's main function */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cost_map_exercise");

  // ros::NodeHandle nodeHandle("~");
  // cost_map_exercise::CostMapExercise costMapExercise(nodeHandle);

  printf("creating gridLayer\n");
  grid_layer::GridLayer gridLayer();

  ros::spin();

  return 0;
}
