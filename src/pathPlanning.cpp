#include "pathPlanning.h"

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

std::vector<int> findOptimalPath(Boxes boxes, RobotPose startingPose) {
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

    return bestRoute;
}