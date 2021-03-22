#pragma once

#include <vector>
#include "boxes.h"
#include <robot_pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>

class pathPlanning {
    private:
        float deg2rad(float angle);
        float rad2deg(float angle);

        ros::NodeHandle nh;

        Boxes boxes;

        double loopCost(double **adjMatrix, std::vector<int> movePlan);
        std::vector<int> findOptimalPath(bool printResult); // Returns best route as box IDs to take


    public:
        std::vector<float> startCoord;
        std::vector<int> idealOrder; // Holds the order to take through the world
        std::vector<std::vector<float> > stopCoords; // Stores the coordinates of each stop corrisponding to a box

        pathPlanning(ros::NodeHandle& n, Boxes boxesIn, std::vector<float> startPosition);
        bool clearCostMap();
        std::vector<float> faceBoxPoint(std::vector<float> boxCoords);
        bool checkPossible(std::vector<float> goalCoord);


};