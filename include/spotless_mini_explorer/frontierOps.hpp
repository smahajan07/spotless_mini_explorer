#ifndef INCLUDE_FRONTIEROPS_HPP_
#define INCLUDE_FRONTIEROPS_HPP_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/OccupancyGrid.h"

class frontierOps {
 private:
  sensor_msgs::PointCloud frontierPtCloud;
  ros::Publisher publisherPtCloud;
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
  // get row
  int getRow(int, int);
  // get column
  int getCol(int, int);
  // get nearest frontier
  int getNearestFrontier(const sensor_msgs::PointCloud);
  // get farthest frontier
  int getFarthestFrontier(const sensor_msgs::PointCloud);
  // destructor
  ~frontierOps();
};

#endif  // INCLUDE_FRONTIEROPS_HPP_
