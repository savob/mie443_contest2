#include "pathPlanning.h"

float pathPlanning::deg2rad(float angle) {
    return (angle * M_PI) / 180.0;
}
float pathPlanning::rad2deg(float angle) {
    return (angle * 180.0) / M_PI;
}

bool pathPlanning::goToStop(int index) {
    return goToCoords(stopCoords[idealOrder[index]]);
}

bool pathPlanning::goToCoords(std::vector<float> target) {
    
    bool gotThere = Navigation::moveToGoal(target);

    // Handle initial failure
    if (!gotThere) {
        ROS_INFO("Initial attempt failed, clearing cost table.");
        
        clearCostMap(); // Clear cost map and reattempt motion
        gotThere = Navigation::moveToGoal(target);
        
        // Absolute failure
        if (!gotThere) ROS_ERROR("Failed to reach point (%5.2f, %5.2f, %6.3f)", target[0], target[1], target[2]);
    }

    return gotThere;
}

pathPlanning::pathPlanning(ros::NodeHandle& n, Boxes boxesIn, std::vector<float> startPosition, bool printStuff) {
    nh = n;
    startCoord = startPosition;
    boxCoordList = boxesIn.coords;

    //Initialize list
    for (int i = 0; i < boxCoordList.size(); i++) {
        stopCoords.push_back(faceBoxPoint(i));
    }

    idealOrder = findOptimalPath(printStuff);
}

std::vector<float> pathPlanning::faceBoxPoint(int boxIndex) {
    std::vector<float> output(3,0);

    std::vector<float> boxCoords = boxCoordList[boxIndex];

    // Generate points starting from middle and then going outwards up to a limit
    // Also gradually increase distance as needed
    for (float offAngle = 0; offAngle <= offsetAngleLimit; offAngle = offAngle + offsetAngleStep) {
    for (float offDist = offsetDistStart; offDist <= offsetDistLimit; offDist = offDist + offsetDistStep) {
        
            
            // Adjust position coordinates based off box face
            output[0] = boxCoords[0] + offDist * cosf(boxCoords[2] + offAngle);
            output[1] = boxCoords[1] + offDist * sinf(boxCoords[2] + offAngle);
            
            // Set angle to face the point
            output[2] = boxCoords[2] + offAngle;
            if (output[2] > 0) output[2] = output[2] - M_PI;
            else output[2] = output[2] + M_PI;

            // Adjust increment for next iteration
            if (offAngle > 0) offAngle = 0.0 - offAngle;       // Flip from positive to negative
            else offAngle = (0.0 - offAngle) + offsetAngleStep;   // Flip and add increment (once positive again)

            // Check if the plotted point is valid
            bool validPoint = checkPossible(output);

            if (validPoint) return output; // Return first possible point
        }
    } 

    // Alert user to failure
    ROS_ERROR("No valid location to offset to.\n\tPoint: (%5.2f, %5.2f. %5.1f)\n\tOffset Dist.: %5.2f m\tOffset Angle: %5.2f",
        boxCoords[0], boxCoords[1], boxCoords[2], offsetDistLimit, rad2deg(offsetAngleLimit));
    
    return boxCoords; // Return the input if failed to find a point
}

bool pathPlanning::clearCostMap() {
    // Clear cost map using the service
    std_srvs::Empty srv;
    ros::ServiceClient clear = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    bool callExecuted = clear.call(srv);

    if (callExecuted) ROS_INFO("Cleared cost map");

    return callExecuted;
}

bool pathPlanning::checkPossible(std::vector<float> goalCoord, bool printStuff) {
    
    bool callExecuted, validPlan;

    nav_msgs::GetPlan srv;

    // Set start position
    geometry_msgs::PoseStamped start;
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(startCoord[2]);
    start.header.frame_id = "map";
    start.pose.position.x = startCoord[0];
    start.pose.position.y = startCoord[1];
    start.pose.position.z = 0;
    start.pose.orientation.x = 0;
    start.pose.orientation.y = 0;
    start.pose.orientation.z = phi.z;
    start.pose.orientation.w = phi.w;

    // Set goal position
    geometry_msgs::PoseStamped goal;
    phi = tf::createQuaternionMsgFromYaw(goalCoord[2]);
    goal.header.frame_id = "map";
    goal.pose.position.x = goalCoord[0];
    goal.pose.position.y = goalCoord[1];
    goal.pose.position.z = 0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = phi.z;
    goal.pose.orientation.w = phi.w;
    
    // Set up the service and call it
    ros::ServiceClient checkPath = nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");
    
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = 0.0;
    callExecuted = checkPath.call(srv);
    
    // Output print statments
    if(!callExecuted){
        ROS_ERROR("Call to check plan NOT sent");
    }

    if(srv.response.plan.poses.size() > 0){
        validPlan = true;
        ROS_INFO_COND(printStuff, "Successful plan.\n\tStart: (%5.2f, %5.2f. %6.3f)\n\tGoal:  (%5.2f, %5.2f. %6.3f)",
            startCoord[0], startCoord[1], startCoord[2], goalCoord[0], goalCoord[1], goalCoord[2]);
    }
    else{
        validPlan = false;
        ROS_INFO_COND(printStuff, "Unsuccessful plan.\n\tStart: (%5.2f, %5.2f. %6.3f)\n\tGoal:  (%5.2f, %5.2f. %6.3f)",
            startCoord[0], startCoord[1], startCoord[2], goalCoord[0], goalCoord[1], goalCoord[2]);
    }
    
    return validPlan;
}

double pathPlanning::loopCost(double **adjMatrix, std::vector<int> movePlan) {
    // Note, adjMatrix has been passed in by reference so any changes to it will 
    // be reflected in the variable used when calling this
    double cost = 0;
    for(int i = 1; i < movePlan.size(); ++i) {
        cost += (adjMatrix[movePlan[i]])[movePlan[i-1]];
    }
    cost += adjMatrix[movePlan[movePlan.size() - 1]][movePlan[0]];
    return cost;
}

std::vector<int> pathPlanning::findOptimalPath(bool printResult) {
    ROS_INFO("Determining optimal path using brute force method.");

    // Create an adjacency matrix
    int tour_points = stopCoords.size() + 1;
    double adjMatrix[tour_points][tour_points];
    
    for(int i = 0; i < tour_points; ++i) {
        for(int j = 0; j < tour_points; ++j) {
            if(i == j){
                adjMatrix[i][j] = 0;
            }
            else if(i == 0) {
                double dx = startCoord[0] - stopCoords[j-1][0];
                double dy = startCoord[1] - stopCoords[j-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
            else if(j == 0) {
                double dx = startCoord[0] - stopCoords[i-1][0];
                double dy = startCoord[1] - stopCoords[i-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
            else {
                double dx = stopCoords[i-1][0] - stopCoords[j-1][0];
                double dy = stopCoords[i-1][1] - stopCoords[j-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
        }
    }

    // Initialize vector as a set of numbers from 0 to the number of tour points
    std::vector<int> movePlan(tour_points);
    for(int i = 0; i < tour_points; ++i) {
        movePlan[i] = i;
    }

    // Prepare pointer to pass adjMatrix
    double *temp[tour_points];
    for(int i = 0; i < tour_points; ++i) temp[i] = adjMatrix[i];

    double bestScore = loopCost(temp, movePlan);
    std::vector<int> bestRoute = movePlan;

    while(std::next_permutation(movePlan.begin() + 1, movePlan.end())) {
        double s = loopCost(temp, movePlan);
        if(s < bestScore) {
            bestScore = s;
            bestRoute = movePlan;
        }
    }

    ROS_INFO("Best path determined for %d given boxes. Estimated travel: %.2f m.",(int) bestRoute.size(), bestScore);

    if(printResult) {
        char buffer[50];
        for (int i = 0; i < bestRoute.size(); i++) { 
            //sprintf(buffer, "Stop %2d - Box %2d\t(%5.2f, %5.2f)\n", i + 1, 
            //    bestRoute[i], stopCoords[bestRoute[i]][0], stopCoords[bestRoute[i]][1]);
            
            //std::cout << buffer;
            std::cout << "Stop " << i+1 << " - Box " << bestRoute[i] << std::endl;
        }
    }
    
    return bestRoute;
}
