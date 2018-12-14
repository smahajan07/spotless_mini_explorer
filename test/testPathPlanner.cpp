#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "std_srvs/Empty.h"
#include "../include/spotless_mini_explorer/pathPlanner.hpp"

TEST(TestPathPlannerFunc, testGetCenterPt) {
  pathPlanner testObj;
  std::vector<int> frontier{ 7, 4, 5, 6, 1, 3, 2};
  ASSERT_EQ(4, testObj.getMedian(frontier));
}

TEST(TestRosActivity, testMapCB) {
  ros::NodeHandle nh;
  pathPlanner testObj;
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
