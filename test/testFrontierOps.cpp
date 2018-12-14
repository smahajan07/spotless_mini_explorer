#include <gtest/gtest.h>
#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "../include/spotless_mini_explorer/frontierOps.hpp"
#include "../include/spotless_mini_explorer/pathPlanner.hpp"

TEST(testFrontier, checkFrontierOccupiedMoreThanThresh) {
  unsigned int seed = time(NULL);
  frontierOps testObj;
  nav_msgs::OccupancyGrid customFrontier;
  customFrontier.info.resolution = 1.0;
  customFrontier.info.width = 6;
  customFrontier.info.height = 6;
  int mapSize = 36;
  int mapWidth = 6;
  int point =  rand_r(&seed)%mapSize/2;;
  int ptCloud[mapSize] = {5};
  ptCloud[point] = -1;
  ptCloud[point-1] = 0;
  std::vector<signed char> v(ptCloud, ptCloud + mapSize);
  customFrontier.data = v;
  ASSERT_EQ(true, testObj.isFrontier(customFrontier, point, mapSize,
    mapWidth)) << "Diagnostic : " << point;
}

TEST(testFrontier, checkFrontierNoData) {
  unsigned int seed = time(NULL);
  frontierOps testObj;
  nav_msgs::OccupancyGrid customFrontier;
  customFrontier.info.resolution = 1.0;
  customFrontier.info.width = 6;
  customFrontier.info.height = 6;
  int mapSize = 36;
  int mapWidth = 6;
  int point =  rand_r(&seed)%mapSize/2;;
  int ptCloud[mapSize] = {12};
  std::vector<signed char> v(ptCloud, ptCloud + mapSize);
  customFrontier.data = v;
  ASSERT_EQ(false, testObj.isFrontier(customFrontier, point, mapSize,
    mapWidth)) << "Diagnostic : " << point;
}

TEST(testProcessFrontier, getFrontiers) {
  frontierOps testObj;
  nav_msgs::OccupancyGrid customFrontier;
  std::vector<std::vector<int> > frontiers;
  int mapHeight = 6;
  int mapWidth = 6;
  int pose = 0;

  customFrontier.info.resolution = 1.0;
  customFrontier.info.width = mapWidth;
  customFrontier.info.height = mapHeight;

  int ptCloud[225] = {0};
  std::vector<signed char> v(ptCloud, ptCloud + 225);
  customFrontier.data = v;

  frontiers = testObj.processFrontiers(customFrontier, mapHeight, mapWidth,
    pose);
  EXPECT_EQ(0, frontiers.size()) << "diagnostic message " << frontiers.size();
}

TEST(testDistance, checkEuclideanDistance) {
  frontierOps testObj;
  float x1 = 1;
  float y1 = 1;
  float x2 = 2;
  float y2 = 2;
  float dist = sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
  ASSERT_NEAR(dist, testObj.getDistance(x1, x2, y1, y2),
    0.0001) << "Diagnostic : " << dist;
}
