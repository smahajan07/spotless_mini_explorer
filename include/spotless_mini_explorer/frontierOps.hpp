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
 *@file frontierOps.hpp
 *@author Sarthak Mahajan
 *@brief Here all the class variables and methods are declared. frontierOps is
 * the base class for the system and provides helper functions to process the
 * map obtained of the surrounding. Broadly speaking it receives the map and
 * sends back the point cloud of medians of the frontiers so that the base class
 * can now use that information to set a goal for the bot and move it there.
 */

#ifndef INCLUDE_SPOTLESS_MINI_EXPLORER_FRONTIEROPS_HPP_
#define INCLUDE_SPOTLESS_MINI_EXPLORER_FRONTIEROPS_HPP_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/OccupancyGrid.h"

/**
*@brief The frontierOps class. It forms the base class and pathPlanner class
* inherits from it. It mainly has methods to process Frontiers in a map.
*/
class frontierOps {
 private:
  // threshold to check if a certain point is occupied or not
  int OCC_THRESHOLD = 10;
  /**
  *The following are the indices to maintain elements for the breadth first
  * search algorithm
  */
  //  map open list index
  int MAP_OPEN_LIST = 1;
  // map close list index
  int MAP_CLOSE_LIST = 2;
  // frontier open list index
  int FRONTIER_OPEN_LIST = 3;
  // frontier close list index
  int FRONTIER_CLOSE_LIST = 4;

 public:
  /**
  *@brief Constructor for class. Left empty for future development
  *@return None
  */
  frontierOps();

  /**
  *@brief Takes in the map and finds out the potential frontiers by doing a
  * breadth first search. It uses a queue to maintain the frontiers. Makes calls
  * to other methods of the class as a part of computing the frontiers.
  *
  *@param map Occupancy grid of the surrounding. Unknown points correspond to
  * -1, obstacles have a value greater than 0 and empty space will have the
  * value 0. It is processed like a one dimensional array.
  *@param mapHeight Visually map is a two dimensional object but it is passed
  * as a one dimensional array. So mapHeight is the height (or length) of the
  * map.
  *@param mapWidth Width of the map
  *@param pose Pose of the robot
  *
  *@return Frontiers of type: a two dimensional integer vector
  */
  std::vector<std::vector<int> > processFrontiers(
    const nav_msgs::OccupancyGrid& map, int mapHeight, int mapWidth, int pose);

  /**
  *@brief Checks if a given point on a given map is a frontier or not. Checks
  * the occupancy grid values of the point ad its neighbours and determines
  * whether or not it is a frontier
  *
  *@param map Occupancy grid of the surrounding. Unknown points correspond to
  * -1, obstacles have a value greater than 0 and empty space will have the
  * value 0. It is processed like a one dimensional array.
  *@param point The point in the array that needs to be checked as a frontier
  *@param mapSize Map size is map height * map width
  *@param mapWidth Width of the map
  *
  *@return True if the given point is a frontier, False if not
  */
  bool isFrontier(const nav_msgs::OccupancyGrid& map, int point, int mapSize,
    int mapWidth);

  /**
  *@brief Receives a pointer to an empty array and a point. It fills the array
  * with eight adjacent neighbours of the point on a given map.
  *
  *@param loc It's the pointer to the starting position of an empty array
  *@param position The point on the map for which we will find the adjacent
  * neighbours
  *@param mapWidth Width of the map
  *
  *@return None
  */
  void getAdjacentPts(int *loc, int position, int mapWidth);

  /**
  *@brief Get the nearest frontier (euclidean distance)
  *
  *@param frontierCloud The point cloud created using frontier medians (centre
  * points)
  *
  *@return The frontier (number) of the nearest frontier from the point cloud
  */
  int getNearestFrontier(const sensor_msgs::PointCloud frontierCloud);

  /**
  *@brief At times when the bot fails to move to a nearby frontier we can move
  * it all the way to the farthest frontier to cover more area
  *
  *@param frontierCloud The point cloud created using frontier medians (center
  * points)
  *
  *@return The frontier (number) of the farthest frontier from the point cloud
  */
  int getFarthestFrontier(const sensor_msgs::PointCloud frontierCloud);

  /**
  *@brief Get euclidean distance between two points
  *
  *@param x1 x coordinate of point 1
  *@param x2 x coordinate of point 2
  *@param y1 y coordinate of point 1
  *@param y2 y coordinate of point 2
  *
  *@return Euclidean distance between the points in float
  */
  float getDistance(float x1, float x2, float y1, float y2);

  /**
  *@brief Destructor for class frontierOps. Left empty for future development
  */
  ~frontierOps();
};

#endif  // INCLUDE_SPOTLESS_MINI_EXPLORER_FRONTIEROPS_HPP_
