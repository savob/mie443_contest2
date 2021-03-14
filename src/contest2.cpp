#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <cmath>
#include <algorithm>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        ROS_FATAL("Could not load coords or templates");
        return -1;
    }
    else {
        ROS_INFO("Box coordinates:");

        for(int i = 0; i < boxes.coords.size(); ++i) {
            std::cout << i << "\tx: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                    << boxes.coords[i][2] << std::endl;
        }
    }

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    // Create an adjacency matrix
    double adjMatrix[boxes.coords.size() + 1][boxes.coords.size() +1];
    
    for(int i = 0; i < boxes.coords.size() + 1; ++i) {
        for(int j = 0, j , boxes.coords.size() + 1; ++j) {
            if(i == j){
                adjMatrix[i][j] = 0;
            }
            else if(i == 0) {
                double dx = robotPose.x - boxes.coords[j-1][0];
                double dy = robotPose.y - boxes.coords[j-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
            else if(j == 0) {
                double dx = robotPose.x - boxes.coords[i-1][0];
                double dy = robotPose.y - boxes.coords[i-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
            else {
                double dx = boxes.coords[i-1][0] - boxes.coords[j-1][0];
                double dy = boxes.coords[i-1][1] - boxes.coords[j-1][1];
                adjMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
        }
    }

    int movePlan[boxes.coords.size() + 1];
    for(int i = 0; i < boxes.coords.size() + 1; ++i) {
        movePlan[i] = i;
    }

    double loopCost(int movePlan, double adjMatrix) {
        double cost = 0;
        for(int i = 1; i < movePlan.size(); ++i) {
            cost += adjMatrix[movePlan[i]][moveplan[i-1]];
        }
        cost += adjMatrix[movePlan.end()][movePlan.begin()];
        return cost;
    }

    double bestScore = loopCost(movePlan, adjMatrix);
    int bestRoute = movePlan;
    while(std::next_permutation(movePlan.begin() + 1, movePlan.end())) {
        double s = loopCost(movePlan, adjMatrix);
        if(s < bestScore) {
            bestScore = s;
            bestRoute = movePlan;
        }
    }

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi


        // Vision stuff past here, no touchy

        // Location of test file (needs to be absolute)
        //char testFile[] = "/home/brobot/catkin_ws/src/mie443_contest2/testpics/crab5.png";
        //imagePipeline.loadImage(testFile);
        
        //imagePipeline.getTemplateID(boxes);

        // End of vision stuff

        ros::Duration(0.01).sleep(); // Two second sleep per step
    }
    return 0;
}
