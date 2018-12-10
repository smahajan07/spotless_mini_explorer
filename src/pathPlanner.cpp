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
 *@file pathPlanner.cpp
 *@author Sarthak Mahajan
 *@brief Here all the class variables and methods are defined. pathPlanner is
 * the derived class (inherits from the frontierOps class). Primarily, this
 * class is at the front end of the system. It subscribes to the map of the
 * surrounding and sends it to methods of the frontierOps class to process it
 * and send the computed goal. Now it takes control again and calls the method
 * to move the bot to the desired location.
 */

#include <iostream>
#include <vector>
#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "spotless_mini_explorer/pathPlanner.hpp"
#include "spotless_mini_explorer/frontierOps.hpp"

pathPlanner::pathPlanner() {
  // define the publishers
  velPub = nh.advertise <geometry_msgs::Twist>("/mobile_base/commands/velocity"
    , 10);
  publisherPtCloud = nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
}

void pathPlanner::updateMap() {
  // define the subscriber
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
  float resolution = map.info.resolution;
  float map_x = map.info.origin.position.x / resolution;
  float map_y = map.info.origin.position.y / resolution;
  float x = 0. - map_x;
  float y = 0. - map_y;
  // send the map for processing frontiers
  std::vector<std::vector<int> > frontiers = frontierOps::processFrontiers(
      map, map.info.height, map.info.width, x + (y * map.info.width));
  std::vector<std::vector<int> > map_2d(map.info.height,
    std::vector<int>(map.info.width, 0));

  for (int i = 0; i < map.info.height; i++) {
    for (int j = 0; j < map.info.width; j++) {
      map_2d[i][j] = (int) map.data[j + i * map.info.width];
    }
  }

  // get the centre points of the frontiers
  ROS_INFO_STREAM("frontiers size"<< frontiers.size());
  std::vector<int> frontierMedians;
  for (int i = 0; i < frontiers.size(); i++) {
    int j = getMedian(frontiers[i]);
    frontierMedians.push_back(j);
    }

  frontierPtCloud.points.resize(frontierMedians.size());
  for (int i = 0; i < frontierMedians.size(); i++) {
    frontierPtCloud.points[i].x =
    ((frontierMedians[i] % map.info.width) + map_x) * resolution;
    frontierPtCloud.points[i].y =
    ((frontierMedians[i] / map.info.width) + map_y) * resolution;
    frontierPtCloud.points[i].z = 0;
    }

  int num_points = 0;
  for (int i = 0; i < frontiers.size(); i++) {
    for (int j = 0; j < frontiers[i].size(); j++) {
      num_points++;
    }
  } 

  ROS_INFO_STREAM("Num of frontier points "<< num_points);
  for (int i = 0; i < frontiers.size(); i++) {
    for (int j = 0; j < frontiers[i].size(); j++) {
    auto y= (frontiers[i][j]%map.info.width);
    auto x= (frontiers[i][j]/map.info.width);
    map_2d[x][y]  = 5;
    }
  } 
  for (int i = 0; i < frontierMedians.size(); i++) {
    auto y= (frontierMedians[i]%map.info.width);
    auto x= (frontierMedians[i]/map.info.width);
    map_2d[x][y]  = 10;
  }

  // publish the frontiers
  publisherPtCloud.publish(frontierPtCloud);
  // send the point cloud to the moveBot method so that the bot can start
  // moving to the nearest frontier
  moveBot(frontierPtCloud);
}

void pathPlanner::moveBot(const sensor_msgs::PointCloud frontierCloud) {
  // if there are not frontiers, return
  if (frontierCloud.points.size() == 0)
    return;
  
  bool at_target = false;
  int frontier_i = frontierOps::getNearestFrontier(frontierCloud);
  ROS_INFO("Closest frontier: %d", frontier_i);
  // move to the closest frontier
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = frontierCloud.points[frontier_i].x;
  goal.target_pose.pose.position.y = frontierCloud.points[frontier_i].y;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  goal.target_pose.pose.orientation = odom_quat;
  ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x,
           goal.target_pose.pose.position.y);
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("move_base action server active");
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(20.0));
  ROS_INFO("move_base goal published");
  int count = 0;

  if (
    ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || count < 2) {
    at_target = true;
    ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    geometry_msgs::Quaternion odom_quat =
              tf::createQuaternionMsgFromYaw(3.14);
    goal.target_pose.pose.orientation = odom_quat;
    ac.sendGoal(goal);
    ac.waitForResult();
  count++;
  } else {
  frontier_i = (rand() % frontierCloud.points.size());
  ROS_WARN_STREAM("Frontier navigation failed, rerouting to farthest frontier");
  goal.target_pose.pose.position.x = frontierCloud.points[frontier_i].x;
  goal.target_pose.pose.position.y = frontierCloud.points[frontier_i].y;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  goal.target_pose.pose.orientation = odom_quat;
  ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x,
           goal.target_pose.pose.position.y);
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("move_base action server active");
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(20.0));
  ROS_INFO("move_base goal published");
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("ATTEMPT #: " << count );
    at_target = true;
    ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    geometry_msgs::Quaternion odom_quat =
              tf::createQuaternionMsgFromYaw(3.14);
    goal.target_pose.pose.orientation = odom_quat;
    ac.sendGoal(goal);
    ac.waitForResult();
    count++;
    }
  }
}

int pathPlanner::getMedian(std::vector<int> frontier) {
  // return the centre point of the frontier points
  sort(frontier.begin(), frontier.end());
  return frontier[frontier.size()/2];
}

pathPlanner::~pathPlanner() {
}
