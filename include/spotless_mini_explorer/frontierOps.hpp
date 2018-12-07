#ifndef INCLUDE_FRONTIEROPS_HPP_
#define INCLUDE_FRONTIEROPS_HPP_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/OccupancyGrid.h"

class frontierOps {
 private:
  // setting values as private members instead of #define
  int OCC_THRESHOLD = 10;
  int MAP_OPEN_LIST = 1;
  int MAP_CLOSE_LIST = 2;
  int FRONTIER_OPEN_LIST = 3;
  int FRONTIER_CLOSE_LIST = 4;
 public:
  // constructor
  frontierOps();
  // process frontiers
  std::vector<std::vector<int> > processFrontiers(
    const nav_msgs::OccupancyGrid&, int, int, int);
  // check if point is frontier
  bool isFrontier(const nav_msgs::OccupancyGrid&, int, int, int);
  // get adjacent points
  void getAdjacentPts(int*, int , int);
  // get nearest frontier
  int getNearestFrontier(const sensor_msgs::PointCloud);
  // get farthest frontier
  int getFarthestFrontier(const sensor_msgs::PointCloud);
  // get distance from a point
  float getDistance(float, float, float, float);
  // destructor
  ~frontierOps();
};

#endif  // INCLUDE_FRONTIEROPS_HPP_
