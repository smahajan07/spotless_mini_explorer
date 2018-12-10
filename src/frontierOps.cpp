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
 *@file frontierOps.cpp
 *@author Sarthak Mahajan
 *@brief Here all the class variables and methods are defined. frontierOps is
 * the base class for the system and provides helper functions to process the
 * map obtained of the surrounding. Broadly speaking it receives the map and
 * sends back the point cloud of medians of the frontiers so that the base class
 * can now use that information to set a goal for the bot and move it there.
 */

#include <iostream>
#include <vector>
#include <queue>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "spotless_mini_explorer/frontierOps.hpp"

frontierOps::frontierOps() {
}

std::vector<std::vector<int> > frontierOps::processFrontiers(
  const nav_msgs::OccupancyGrid& map, int mapHeight, int mapWidth, int pose) {
  // declare a two dimensional vector for managing frontiers
  std::vector<std::vector<int> > frontiers;
  int mapSize = mapHeight * mapWidth;
  std::map<int, int> cell_states;
  // declare queue for implementation of breadth first search
  std::queue<int> q_m;
  q_m.push(pose);
  cell_states[pose] = MAP_OPEN_LIST;
  int adj_vector[8];
  int v_neighbours[8];
  // Breadth First Search (BFS)
  while(!q_m.empty()) {
    int cur_pos = q_m.front();
    q_m.pop();
    if(cell_states[cur_pos] == MAP_CLOSE_LIST)
      continue;
    if(isFrontier(map, cur_pos, mapSize, mapWidth)) {
      std::queue<int> q_f;
      std::vector<int> new_frontier;
      q_f.push(cur_pos);
      cell_states[cur_pos] = FRONTIER_OPEN_LIST;
      // Second BFS
      while(!q_f.empty()) {
        int n_cell = q_f.front();
        q_f.pop();
        if(cell_states[n_cell] == MAP_CLOSE_LIST ||
          cell_states[n_cell] == FRONTIER_CLOSE_LIST)
          continue;
        if(isFrontier(map, n_cell, mapSize, mapWidth)) {
          new_frontier.push_back(n_cell);
          getAdjacentPts(adj_vector, n_cell, mapWidth);
          for(int i = 0; i < 8; i++) {
            if(adj_vector[i] < mapSize && adj_vector[i] >= 0) {
              if(cell_states[adj_vector[i]] != FRONTIER_OPEN_LIST &&
                cell_states[adj_vector[i]] != FRONTIER_CLOSE_LIST &&
                cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
                if(map.data[adj_vector[i]] != 100) {
                  q_f.push(adj_vector[i]);
                  cell_states[adj_vector[i]] = FRONTIER_OPEN_LIST;
                }
              }
            }
          }
        }
        cell_states[n_cell] = FRONTIER_CLOSE_LIST;
      }
      if(new_frontier.size() > 5)
        frontiers.push_back(new_frontier);

      for(unsigned int i = 0; i < new_frontier.size(); i++) {
        cell_states[new_frontier[i]] = MAP_CLOSE_LIST;
      }
    }
    getAdjacentPts(adj_vector, cur_pos, mapWidth);

    for (int i = 0; i < 8; ++i) {
      if(adj_vector[i] < mapSize && adj_vector[i] >= 0) {
        if(cell_states[adj_vector[i]] != MAP_OPEN_LIST &&
          cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
          getAdjacentPts(v_neighbours, adj_vector[i], mapWidth);
          bool map_open_neighbor = false;
          for(int j = 0; j < 8; j++) {
            if(v_neighbours[j] < mapSize && v_neighbours[j] >= 0) {
              if(map.data[v_neighbours[j]] < OCC_THRESHOLD &&
                map.data[v_neighbours[j]] >= 0) {
                map_open_neighbor = true;
                break;
              }
            }
          }
          if(map_open_neighbor) {
            q_m.push(adj_vector[i]);
            cell_states[adj_vector[i]] = MAP_OPEN_LIST;
          }
        }
      }
    }
    cell_states[cur_pos] = MAP_CLOSE_LIST;
  }
  return frontiers;
}

void frontierOps::getAdjacentPts(int *loc, int position, int mapWidth) {
  // modify the empty integer array with neighbours of the given point
  loc[0] = position - mapWidth - 1;
  loc[1] = position - mapWidth;
  loc[2] = position - mapWidth + 1;
  loc[3] = position - 1;
  loc[4] = position + 1;
  loc[5] = position + mapWidth - 1;
  loc[6] = position + mapWidth;
  loc[7] = position + mapWidth + 1;
}

bool frontierOps::isFrontier(const nav_msgs::OccupancyGrid& map,
  int point, int mapSize, int mapWidth) {
  // check if the received point is a frontier or not
  const int MIN_FOUND = 1;
  // The point under consideration must be known
  if(map.data[point] != -1) {
    return false;
  }
  int locations[8];
  getAdjacentPts(locations, point, mapWidth);
  int found = 0;
  for(int i = 0; i < 8; i++) {
    if(locations[i] < mapSize && locations[i] >= 0) {
      // None of the neighbours should be occupied space.
      if(map.data[locations[i]] > OCC_THRESHOLD) {
        return false;
      }
      // At least one of the neighbours is open and known space,
      // hence frontier point
      if(map.data[locations[i]] == 0) {
        found++;
        if(found == MIN_FOUND)
          return true;
      }
    }
  }
  return false;
}

int frontierOps::getNearestFrontier(
  const sensor_msgs::PointCloud frontierCloud) {
  // get the nearest frontier from the point cloud
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  map_listener.waitForTransform("/map", "/base_link",
                               ros::Time(0), ros::Duration(3.0));
  map_listener.lookupTransform("/map", "/base_link", ros::Time(0),
    map_transform);
  int frontier_i = 0;
  float closest_frontier_distance = 100000;
  for (int i = 0; i < frontierCloud.points.size(); i++) {
    float distance = getDistance(frontierCloud.points[i].x,
                                 map_transform.getOrigin().x(),
                                 frontierCloud.points[i].y,
                                 map_transform.getOrigin().y());
    if (distance > .7 && distance <= closest_frontier_distance) {
      closest_frontier_distance = distance;
      frontier_i = i;
    }
  }
  return frontier_i;
}

int frontierOps::getFarthestFrontier(
  const sensor_msgs::PointCloud frontierCloud) {
  // get the farthest frontier from the point cloud
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  map_listener.waitForTransform("/map", "/base_link",
                               ros::Time(0), ros::Duration(3.0));
  map_listener.lookupTransform("/map", "/base_link", ros::Time(0),
    map_transform);
  int frontier_i = 0;
  float closest_frontier_distance = 0;
  for (int i = 0; i < frontierCloud.points.size(); i++) {
    float distance = getDistance(frontierCloud.points[i].x,
                                 map_transform.getOrigin().x(),
                                 frontierCloud.points[i].y,
                                 map_transform.getOrigin().y());
    if (distance >= closest_frontier_distance) {
      closest_frontier_distance = distance;
      frontier_i = i;
    }
  }
  return frontier_i;
}

float frontierOps::getDistance(float x1, float x2, float y1, float y2) {
  // compute euclidean distance
  return sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
}

frontierOps::~frontierOps() {
}
