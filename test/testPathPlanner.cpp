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
 *@file testPathPlanner.cpp
 *@author Sarthak Mahajan
 *@brief Path Planner class is the derived class and primarily takes care of
 * the call back function, for getting the map of the surrounding, and moving
 * the robot.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "std_srvs/Empty.h"
#include "../include/spotless_mini_explorer/pathPlanner.hpp"

/**
 *@brief Test for getMedian()
 * Checks if the center point of a frontier is as expected
 */
TEST(TestPathPlannerFunc, testGetCenterPt) {
  pathPlanner testObj;
  std::vector<int> frontier{ 7, 4, 5, 6, 1, 3, 2};
  ASSERT_EQ(4, testObj.getMedian(frontier));
}

/**
 *@brief Test for fullScan() and updateMap()
 * Checks if the topic has been correctly set. Moreover it calls the call back
 * function for map and also calls other connected functions.
 */
TEST(TestRosActivity, testMapCB) {
  ros::NodeHandle nh;
  pathPlanner testObj;
  // pausing gazebo service untill it starts
  ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>(
    "/gazebo/pause_physics");
  std_srvs::Empty emptySrv;
  pauseGazebo.call(emptySrv);
  // wait till gazebo starts
  ros::Duration(3).sleep();
  // test
  while(nh.ok()) {
    EXPECT_NO_FATAL_FAILURE(testObj.fullScan());
    EXPECT_NO_FATAL_FAILURE(testObj.updateMap());
    ros::spinOnce();
    break;
  }
}
