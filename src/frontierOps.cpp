#include <iostream>
#include "spotless_mini_explorer/frontierOps.hpp"

frontierOps::frontierOps() {

}

std::vector<std::vector<int> > frontierOps::processFrontiers(
  const nav_msgs::OccupancyGrid& map, int mapHeight, int mapWidth, int pose) {
  
}

void frontierOps::getAdjacentPts(int *loc, int position, int mapWidth) {

}


bool frontierOps::isFrontier(const nav_msgs::OccupancyGrid& map, int point,
  int mapSize, int mapWidth) {
  
}

int frontierOps::getRow(int offset, int width) {

}

int frontierOps::getCol(int offset, int width) {

}

int frontierOps::getNearestFrontier(
  const sensor_msgs::PointCloud frontierCloud) {

}

int frontierOps::getFarthestFrontier(
  const sensor_msgs::PointCloud frontierCloud) {

}

frontierOps::~frontierOps() {

}
