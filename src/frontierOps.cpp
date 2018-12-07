#include <iostream>
#include <queue>
#include "spotless_mini_explorer/frontierOps.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

frontierOps::frontierOps() {
}

std::vector<std::vector<int> > frontierOps::processFrontiers(
  const nav_msgs::OccupancyGrid& map, int mapHeight, int mapWidth, int pose) {
  std::vector<std::vector<int> > frontiers;
  int mapSize = mapHeight * mapWidth;
  std::map<int, int> cell_states;
  std::queue<int> q_m;
  q_m.push(pose);
  cell_states[pose] = MAP_OPEN_LIST;
  int adj_vector[8];
  int v_neighbours[8];
  //
  while(!q_m.empty()) {
    int cur_pos = q_m.front();
    q_m.pop();
    if(cell_states[cur_pos] == MAP_CLOSE_LIST)
      continue;
    if(this->isFrontier(map, cur_pos, mapSize, mapWidth)) {
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
        if(this->isFrontier(map, n_cell, mapSize, mapWidth)) {
          new_frontier.push_back(n_cell);
          this->getAdjacentPts(adj_vector, n_cell, mapWidth);
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
    this->getAdjacentPts(adj_vector, cur_pos, mapWidth);

    for (int i = 0; i < 8; ++i) {
      if(adj_vector[i] < mapSize && adj_vector[i] >= 0) {
        if(cell_states[adj_vector[i]] != MAP_OPEN_LIST &&
          cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
          this->getAdjacentPts(v_neighbours, adj_vector[i], mapWidth);
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
  const int MIN_FOUND = 1;
  // The point under consideration must be known
  if(map.data[point] != -1) {
    return false;
  }
  //
  int locations[8];
  this->getAdjacentPts(locations, point, mapWidth);
  int found = 0;
  for(int i = 0; i < 8; i++) {
    if(locations[i] < mapSize && locations[i] >= 0) {
      // None of the neighbours should be occupied space.
      if(map.data[locations[i]] > OCC_THRESHOLD) {
        return false;
      }
      //At least one of the neighbours is open and known space,
      //hence frontier point
      if(map.data[locations[i]] == 0) {
        found++;
        //
        if(found == MIN_FOUND)
          return true;
      }
      //}
    }
  }
  return false;
}

int frontierOps::getNearestFrontier(
  const sensor_msgs::PointCloud frontierCloud) {
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  map_listener.waitForTransform("/map", "/base_link",
                               ros::Time(0), ros::Duration(3.0));
  map_listener.lookupTransform("/map", "/base_link", ros::Time(0), map_transform);
  int frontier_i = 0;
  float closest_frontier_distance = 100000;
  for (int i = 0; i < frontierCloud.points.size(); i++) {
    float distance = this->getDistance(frontierCloud.points[i].x,
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
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  map_listener.waitForTransform("/map", "/base_link",
                               ros::Time(0), ros::Duration(3.0));
  map_listener.lookupTransform("/map", "/base_link", ros::Time(0), map_transform);
  int frontier_i = 0;
  float closest_frontier_distance = 0;
  for (int i = 0; i < frontierCloud.points.size(); i++) {
    float distance = this->getDistance(frontierCloud.points[i].x,
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
  return sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
}

frontierOps::~frontierOps() {
}
