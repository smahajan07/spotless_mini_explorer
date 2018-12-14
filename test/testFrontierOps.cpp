/** MIT License

 Copyright (c) 2018 Sarthak Mahajan

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

/**
 *@copyright Copyright (c) 2018 Sarthak Mahajan
 *@file testFrontierOps.cpp
 *@author Sarthak Mahajan
 *@brief Frontier Ops is the base class and has important functions related to
 * processing the frontiers. In this file we wish to test the functions of this
 * class which can be isolated. Futher developments will be made
 */

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

/**
 *@brief Test for isFrontier()
 * Checks if the given point is a frontier or not. Even if one frontier
 * is present the test should be able to detect it and return appropriate
 * state.
 */
TEST(testFrontier, checkFrontierOccupiedMoreThanThresh) {
  // set random seed for rand_r() function
  unsigned int seed = time(NULL);
  // create object of frontierOps
  frontierOps testObj;
  // create a custom message
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
  // test isFrontier()
  ASSERT_EQ(true, testObj.isFrontier(customFrontier, point, mapSize,
    mapWidth)) << "Diagnostic : " << point;
}

/**
 *@brief Test for isFrontier()
 * Checks if the given point is a frontier or not. Even no frontier is
 * present the test should be able to detect it and return appropriate state.
 */
TEST(testFrontier, checkFrontierNoData) {
  // set random seed for rand_r() function
  unsigned int seed = time(NULL);
  // create object of frontierOps
  frontierOps testObj;
  // create custom message
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
  // test isFrontier()
  ASSERT_EQ(false, testObj.isFrontier(customFrontier, point, mapSize,
    mapWidth)) << "Diagnostic : " << point;
}

/**
 *@brief Test for processFrontiers()
 * Test for processing frontiers. Checks for number of frontiers present
 * in a map
 */
TEST(testProcessFrontier, getFrontiers) {
  // create frontierOps object
  frontierOps testObj;
  // create custom message
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
  // test processFrontiers()
  frontiers = testObj.processFrontiers(customFrontier, mapHeight, mapWidth,
    pose);
  EXPECT_EQ(0, frontiers.size()) << "diagnostic message " << frontiers.size();
}

/**
 *@brief Test for getDistance()
 * Checks the euclidean distance between two points
 */
TEST(testDistance, checkEuclideanDistance) {
  // create frontierOps object
  frontierOps testObj;
  float x1 = 1;
  float y1 = 1;
  float x2 = 2;
  float y2 = 2;
  float dist = sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
  // test getDistance()
  ASSERT_NEAR(dist, testObj.getDistance(x1, x2, y1, y2),
    0.0001) << "Diagnostic : " << dist;
}
