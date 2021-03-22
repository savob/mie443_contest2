#pragma once
#include <vector>
#include "boxes.h"
#include <robot_pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>

double loopCost(double **adjMatrix, std::vector<int> movePlan);
std::vector<int> findOptimalPath(Boxes boxes, RobotPose startingPose, bool printResult); // Returns best route as box IDs to take

bool checkPlan(ros::NodeHandle& nh, std::vector<float> startCoord, std::vector<float> goalCoord);
bool clearCostMap(ros::NodeHandle& nh);
std::vector<float> faceBoxPoint(std::vector<float> boxCoords, std::vector<float> start, ros::NodeHandle& nh);