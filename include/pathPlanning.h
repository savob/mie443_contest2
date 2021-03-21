#pragma once
#include <vector>
#include "boxes.h"
#include <robot_pose.h>

double loopCost(double **adjMatrix, std::vector<int> movePlan);
std::vector<int> findOptimalPath(Boxes boxes, RobotPose startingPose, bool printResult); // Returns best route as box IDs to take