#pragma once

#include <vector>
#include "boxes.h"
#include <robot_pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>
#include "navigation.h"

// define marcos used solely for constants
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

class pathPlanning {
    private:
        float deg2rad(float angle);
        float rad2deg(float angle);

        // Setting up points constants when approaching boxes
        const float offsetAngleLimit = DEG2RAD(50.0);
        const float offsetAngleStep = DEG2RAD(10.0);

        const float offsetDistStart = 0.35; // Any less the 0.35and the rover gets stuck
        const float offsetDistStep = 0.05;
        const float offsetDistLimit = 0.5;

        // Internal copies/references so they don't need to contantly be passed in
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
        std::vector<float> faceBoxPoint(int boxIndex);
        bool checkPossible(std::vector<float> goalCoord, bool printStuff = false);

        bool goToCoords(std::vector<float> target);
        bool goToStop(int index);
};