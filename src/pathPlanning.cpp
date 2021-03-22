#include "pathPlanning.h"

bool checkPlan(ros::NodeHandle& nh, std::vector<float> startCoord, std::vector<float> goalCoord) {
	// Returns true if there is a valid path from start to end
    // https://answers.ros.org/question/264369/move_base-make_plan-service-is-returning-an-empty-path/
    
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
    ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");
    
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = 0.0;
    callExecuted = check_path.call(srv);
    
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
