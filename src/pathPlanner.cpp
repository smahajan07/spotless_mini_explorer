#include <iostream>
#include "spotless_mini_explorer/pathPlanner.hpp"

pathPlanner::pathPlanner() {
  velPub = nh.advertise <geometry_msgs::Twist>("/mobile_base/commands/velocity"
    , 10);
}

void pathPlanner::updateMap() {
  mapSub = nh.subscribe("map", 1, &pathPlanner::mapCallback, this);
}

void pathPlanner::fullScan() {
  int count = 0;
  ros::Rate loop_rate(2);
  // rotate for a few seconds and get map
  while (nh.ok() && count < 60) {
    ROS_INFO_STREAM_ONCE("Rotating initially for approx 6 seconds..");
    msg.linear.x = 0;
    msg.angular.z = 0.3;
    velPub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count ++;
  }
  // stop rotation
  msg.linear.x = 0;
  msg.angular.z = 0.0; 
  velPub.publish(msg);
}

void pathPlanner::mapCallback(const nav_msgs::OccupancyGrid& map) {
  ROS_INFO_STREAM("Map received");
  float resolution = map.info.resolution;
  ROS_INFO_STREAM("Map resolution : " << resolution);
  // frontierOps::processFrontiers();
}

void pathPlanner::moveBot(const sensor_msgs::PointCloud frontierCloud) {
  
}

float pathPlanner::getDistance(float x1, float x2, float y1, float y2) {

}

int pathPlanner::getMedian(std::vector<int> frontier) {

}

pathPlanner::~pathPlanner() {

}
