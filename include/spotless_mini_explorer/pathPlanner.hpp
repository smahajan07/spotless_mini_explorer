#ifndef INCLUDE_PATHPLANNER_HPP_
#define INCLUDE_PATHPLANNER_HPP_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"
#include "spotless_mini_explorer/frontierOps.hpp"

/**
 * @brief Path planner class
 */

class pathPlanner : public frontierOps {
 private:
  // create a object to publish velocities
  geometry_msgs::Twist msg;
  // create a node handle
  ros::NodeHandle nh;
  // create a subsciber for the laserscan topic
  ros::Subscriber mapSub;
  // create a publisher to publish velocities
  ros::Publisher velPub;
 public:
  // constructor
  pathPlanner();
  // map call back
  void mapCallback(const nav_msgs::OccupancyGrid&);
  // full sweep and scan
  void fullScan();
  // set map subscriber
  void updateMap();
  // move turtlebot
  void moveBot(const sensor_msgs::PointCloud);
  // get distance from a point
  float getDistance(float, float, float, float);
  // get median
  int getMedian(std::vector<int>);
  // destructor
  ~pathPlanner();
};

#endif  // INCLUDE_PATHPLANNER_HPP_
