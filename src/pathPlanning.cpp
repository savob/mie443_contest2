#include "pathPlanning.h"

#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

std::vector<float> faceBoxPoint(std::vector<float> boxCoords, std::vector<float> start, ros::NodeHandle& nh) {
    std::vector<float> output(3,0);
    float offsetDist = 0.4; // Offset in meters
    float offsetAngle = 0;  // Angle offset from face normal
    bool validPoint = false;

    const float offsetAngleLimit = DEG2RAD(50);
    const float offsetAngleStep = DEG2RAD(10);

    // Generate points starting from middle and then going outwards up to a limit
    do {
        // Adjust position coordinates based off box face
        output[0] = boxCoords[0] + offsetDist * cosf(boxCoords[2] + offsetAngle);
        output[1] = boxCoords[1] + offsetDist * sinf(boxCoords[2] + offsetAngle);
        
        // Set angle to face the point
        output[2] = boxCoords[2] + offsetAngle;
        if (output[2] > 0) output[2] = output[2] - M_PI;
        else output[2] = output[2] + M_PI;

        // Adjust increment for next iteration
        if (offsetAngle > 0) offsetAngle = 0.0 - offsetAngle;       // Flip from positive to negative
        else offsetAngle = (0.0 - offsetAngle) + offsetAngleStep;   // Flip and add increment (once positive again)

        // Check if the plotted point is valid
        validPoint = checkPlan(nh, start, output);

    } while (!validPoint && (offsetAngle < offsetAngleLimit));

    // Alert user to failure
    if (!validPoint) {
        ROS_ERROR("No valid location to offset to.\n\tPoint: (%5.2f, %5.2f. %6.3f)\n\tOffset Dist.: %5.2f m\tOffset Angle: %5.2f",
            boxCoords[0], boxCoords[1], boxCoords[2], offsetDist, RAD2DEG(offsetAngleLimit));
        
        output = boxCoords; // Return the input
    }

    return output;  
}

bool clearCostMap(ros::NodeHandle& nh) {
    // Clear cost map using the service
    std_srvs::Empty srv;
    ros::ServiceClient clear = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    bool callExecuted = clear.call(srv);

    if (callExecuted) ROS_INFO("Cleared cost map");

    return callExecuted;
}

bool checkPlan(ros::NodeHandle& nh, std::vector<float> startCoord, std::vector<float> goalCoord) {
    
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
        ROS_INFO("Successful plan.\n\tStart: (%5.2f, %5.2f. %6.3f)\n\tGoal:  (%5.2f, %5.2f. %6.3f)",
            startCoord[0], startCoord[1], startCoord[2], goalCoord[0], goalCoord[1], goalCoord[2]);
    }
    else{
        validPlan = false;
        ROS_WARN("Unsuccessful plan.\n\tStart: (%5.2f, %5.2f. %6.3f)\n\tGoal:  (%5.2f, %5.2f. %6.3f)",
            startCoord[0], startCoord[1], startCoord[2], goalCoord[0], goalCoord[1], goalCoord[2]);
    }
    
    return validPlan;
}

double loopCost(double **adjMatrix, std::vector<int> movePlan) {
    // Note, adjMatrix has been passed in by reference so any changes to it will 
    // be reflected in the variable used when calling this
    double cost = 0;
    for(int i = 1; i < movePlan.size(); ++i) {
        cost += (adjMatrix[movePlan[i]])[movePlan[i-1]];
    }
    cost += adjMatrix[movePlan[movePlan.size() - 1]][movePlan[0]];
    return cost;
}

std::vector<int> findOptimalPath(Boxes boxes, RobotPose startingPose, bool printResult) {
    ROS_INFO("Determining optimal path using brute force method.");

    // Create an adjacency matrix
    int tour_points = boxes.coords.size() + 1;
    double adjMatrix[tour_points][tour_points];
    
    for(int i = 0; i < tour_points; ++i) {
        for(int j = 0; j < tour_points; ++j) {
            if(i == j){
                adjMatrix[i][j] = 0;
            }
            else if(i == 0) {
                double dx = startingPose.x - boxes.coords[j-1][0];
                double dy = startingPose.y - boxes.coords[j-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
            else if(j == 0) {
                double dx = startingPose.x - boxes.coords[i-1][0];
                double dy = startingPose.y - boxes.coords[i-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
            else {
                double dx = boxes.coords[i-1][0] - boxes.coords[j-1][0];
                double dy = boxes.coords[i-1][1] - boxes.coords[j-1][1];
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

    ROS_INFO("Best path determined for given boxes. Estimated travel: %.2f m.", bestScore);

    if(printResult) {
        for (int i = 0; i< bestRoute.size() - 1; i++) {
            char buffer[50];
            sprintf(buffer, "Stop %2d - Box %2d\t(%5.2f, %5.2f)\n", i + 1, 
                bestRoute[i], boxes.coords[bestRoute[i]][0], boxes.coords[bestRoute[i]][1]);
            
            std::cout << buffer;
        }
    }
    
    return bestRoute;
}
