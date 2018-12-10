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
 *@file main.cpp
 *@author Sarthak Mahajan
 *@brief Creates an object of the pathPlanner class and initiates the loop of
 * processing the map and moving towards the goal.
 */

#include<iostream>
#include "spotless_mini_explorer/pathPlanner.hpp"

/**
* Main function is initializes the frontier_explorer node and creates an object
* of the pathPlanner class. Before starting the processing of the map, the
* turtlebot takes a complete scan by rotating 360 degrees a couple of times.
* The node does not die untill the user issues a CTRL+C termination request.
*/
int main(int argc, char **argv) {
  // initialize node
  ros::init(argc, argv, "frontier_explorer");
  // create object of pathplanner class
  pathPlanner explorerObj;
  // do a full scan
  explorerObj.fullScan();
  // update map 
  explorerObj.updateMap();
  // ros spin or add spinOnce in loop to control end of node
  ros::spin();

  return 0;
}
