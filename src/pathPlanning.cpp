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