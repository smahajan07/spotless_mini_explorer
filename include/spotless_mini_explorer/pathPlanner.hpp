#ifndef INCLUDE_PATHPLANNER_HPP_
#define INCLUDE_PATHPLANNER_HPP_

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"

/**
 * @brief Path planner class
 */

class pathPlanner {
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
  pathPlanner();
  void mapCallback(const nav_msgs::OccupancyGrid& map);
  void fullScan();
  void updateMap();
  int processFrontiers();
  ~pathPlanner();
};

#endif  // INCLUDE_PATHPLANNER_HPP_
