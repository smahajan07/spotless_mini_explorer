#include<iostream>
#include "spotless_mini_explorer/pathPlanner.hpp"

int main(int argc, char **argv) {
  // initialize node
  ros::init(argc, argv, "frontier_explorer");
  // create object of pathplanner class
  pathPlanner explorerObj;
  // do a full scan
  explorerObj.fullScan();
  // update map 
  explorerObj.updateMap();
  // process frontiers
  explorerObj.processFrontiers();
  // ros spin or add spinOnce in loop to control end of node
  ros::spin();

  return 0;
}
