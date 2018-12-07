#include <iostream>
#include <cstdlib> 
#include "spotless_mini_explorer/pathPlanner.hpp"
#include "spotless_mini_explorer/frontierOps.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

pathPlanner::pathPlanner() {
  velPub = nh.advertise <geometry_msgs::Twist>("/mobile_base/commands/velocity"
    , 10);
  publisherPtCloud = nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
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
  float resolution = map.info.resolution;
  float map_x = map.info.origin.position.x / resolution;
  float map_y = map.info.origin.position.y / resolution;
  float x = 0. - map_x;
  float y = 0. - map_y;
  std::vector<std::vector<int> > frontiers = frontierOps::processFrontiers(
      map, map.info.height, map.info.width, x + (y * map.info.width));
  std::vector<std::vector<int> > map_2d(map.info.height,
    std::vector<int>(map.info.width, 0));

  for (int i = 0; i < map.info.height; i++) {
    for (int j = 0; j < map.info.width; j++) {
      map_2d[i][j] = (int) map.data[j + i * map.info.width];
    }
  }

  ROS_INFO_STREAM("frontiers size"<< frontiers.size());
  std::vector<int> frontierMedians;
  for (int i = 0; i < frontiers.size(); i++) {
    int j = this->getMedian(frontiers[i]);
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

  publisherPtCloud.publish(frontierPtCloud);
  this->moveBot(frontierPtCloud);
}

void pathPlanner::moveBot(const sensor_msgs::PointCloud frontierCloud) {
  if (frontierCloud.points.size() == 0)
    return;
  
  bool at_target = false;
  int frontier_i = frontierOps::getNearestFrontier(frontierCloud);
  ROS_INFO("Closest frontier: %d", frontier_i);

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

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || count < 2) {
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
  sort(frontier.begin(), frontier.end());
  return frontier[frontier.size()/2];
}

pathPlanner::~pathPlanner() {
}
