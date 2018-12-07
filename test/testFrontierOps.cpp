#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "../include/spotless_mini_explorer/frontierOps.hpp"
#include "../include/spotless_mini_explorer/pathPlanner.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"

TEST(testFrontier, checkFrontierOccupiedMoreThanThresh) {
  pathPlanner testObj;
  nav_msgs::OccupancyGrid customFrontier;
  customFrontier.info.resolution = 1.0;
  customFrontier.info.width = 6;
  customFrontier.info.height = 6;
  int ptCloud[] = {12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
                   12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
                   12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};
  std::vector<signed char> v(ptCloud, ptCloud + 36);
  customFrontier.data = v;
  int point = 10;
  int mapSize = 36;
  int mapWidth = 6;
  ASSERT_EQ(false, testObj.isFrontier(customFrontier, point, mapSize, mapWidth));
}

TEST(testFrontier, checkFrontierNoData) {
  pathPlanner testObj;
  nav_msgs::OccupancyGrid customFrontier;
  customFrontier.info.resolution = 1.0;
  customFrontier.info.width = 6;
  customFrontier.info.height = 6;
  int ptCloud[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  std::vector<signed char> v(ptCloud, ptCloud + 36);
  customFrontier.data = v;
  int point = 15;
  int mapSize = 36;
  int mapWidth = 6;
  ASSERT_EQ(false, testObj.isFrontier(customFrontier, point, mapSize, mapWidth));
}
