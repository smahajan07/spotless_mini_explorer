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
 *@file pathPlanner.hpp
 *@author Sarthak Mahajan
 *@brief Here all the class variables and methods are declared. pathPlanner is
 * the derived class (inherits from the frontierOps class). Primarily, this
 * class is at the front end of the system. It subscribes to the map of the
 * surrounding and sends it to methods of the frontierOps class to process it
 * and send the computed goal. Now it takes control again and calls the method
 * to move the bot to the desired location.
 */

#ifndef INCLUDE_SPOTLESS_MINI_EXPLORER_PATHPLANNER_HPP_
#define INCLUDE_SPOTLESS_MINI_EXPLORER_PATHPLANNER_HPP_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"
#include "spotless_mini_explorer/frontierOps.hpp"

/**
 *@brief Path planner class
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
  // point cloud for frontiers
  sensor_msgs::PointCloud frontierPtCloud;
  // publisher for frontier point clouds
  ros::Publisher publisherPtCloud;

 public:
  /**
  *@brief Constructor for the pathPlanner class. It defines the publishers of
  * the class.
  */
  pathPlanner();

  /**
  *@brief This is the call back function of the map subscriber.
  *
  *@param map Occupancy grid of the surrounding. Unknown points correspond to
  * -1, obstacles have a value greater than 0 and empty space will have the
  * value 0. It is processed like a one dimensional array.
  *
  *@return None
  */
  void mapCallback(const nav_msgs::OccupancyGrid& map);

  /**
  *@brief Method to make the bot rotate 360 degrees and get a sweep scan of the
  * surrounding. This is called in the beginning to get the initial
  * understanding of the environment.
  *
  *@param None
  *
  *@return None
  */
  void fullScan();

  /**
  *@brief Sets the map subscriber to the correct topic.
  *
  *@param None
  *
  *@return None
  */
  void updateMap();

  /**
  *@brief This is where the magic happens. At the end of every iteration of
  * scanning and processing, the system generates a point cloud of frontiers.
  * This method sets the goal as the nearest frontier for the bot using the
  * move base package. In case the execution was not successful, which should be
  * rare, it tries to go to the farthest frontier.
  *
  *@param frontierCloud It's the point cloud generated with the frontier median
  * (centre) points
  *
  *@return None
  */
  void moveBot(const sensor_msgs::PointCloud frontierCloud);

  /**
  *@brief Receives the array of frontier points and it sends back the centre
  * point of the frontier.
  *
  *@param frontier Integer array of frontier points
  *
  *@return The center point from the frontier points
  */
  int getMedian(std::vector<int> frontier);

  /**
  *@brief Destructor for class pathPlanner. Left empty for future development
  */
  ~pathPlanner();
};

#endif  // INCLUDE_SPOTLESS_MINI_EXPLORER_PATHPLANNER_HPP_
