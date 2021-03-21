#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <dirent.h>         // Used for reading in the test files
#include <cmath>
#include <algorithm>
#include "file_write.h"

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
        ROS_INFO("Box coordinates loaded successfully:");

        for(int i = 0; i < boxes.coords.size(); ++i) {
            std::cout << i << "\tx: " << boxes.coords[i][0] << "\ty: " << boxes.coords[i][1] << "\tz: " 
                    << boxes.coords[i][2] << std::endl;
        }
    }

    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    // Create an adjacency matrix
    int tour_points = boxes.coords.size() + 1;
    double adjMatrix[tour_points][tour_points];
    
    for(int i = 0; i < tour_points; ++i) {
        for(int j = 0; j < tour_points; ++j) {
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

    std::vector<int> boxIDs(boxes.coords.size()); // Recoding IDs of each box

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi


        // Vision stuff past here, no touchy
           
#ifdef FILE_WRITE_TEST
        for (int i = 0; i < boxes.coords.size(); i++) boxIDs[i] = i / 2;
        writeLog(boxes, movePlan, boxIDs);
        return 0;
#endif

        // Decide to include vision test or not (comment the #define in the image header) 
#ifdef TESTING_VISION_SAMPLES 

        ROS_INFO("\n\nRUNNING VISION TEST\n(will terminate once complete)\n");

        // Test parameters
        std::string testFileFolder = "/home/brobot/catkin_ws/src/mie443_contest2/testpics/";
        bool printInnerWorks = true;
        std::string searchTerm = "blank";
        // Leave as "" for all files in folder (not recommended since too many consecutive searches results in errors)

        // Load in all test files
        std::vector<std::string> fileNames;

        DIR *dr;
        struct dirent *en;
        dr = opendir(testFileFolder.c_str()); // Open directory
        if (dr) {
            while ((en = readdir(dr)) != NULL) {
                std::string temp = en->d_name; // Grab file names

                // Add files that end in PNG and contain search term
                if (temp.find(".png") != std::string::npos) {
                    if (temp.find(searchTerm) != std::string::npos) {
                        fileNames.push_back(temp); 
                    }
                }
            }
            closedir(dr); // Close directory
        }

        std::sort (fileNames.begin(), fileNames.end()); // Sort files alphabetically
        int result[fileNames.size()];

        // Go through each test file to ID
        for (int i = 0; i < fileNames.size(); i++) {
            std::string testFile = testFileFolder + fileNames[i];
            imagePipeline.loadImage(testFile);
            result[i] = imagePipeline.getTemplateID(boxes, printInnerWorks);
        }

        // Print result summary
        printf("\nResults of analysis, ID# and file name.\n");
        for (int i = 0; i < fileNames.size(); i++) {
            printf("ID %2d: %s\n", result[i], fileNames[i].c_str());
        }

        return 0; // Only run this all once
#else
        ROS_INFO_ONCE("\n\nNOT RUNNING VISION TEST\nRegular vision system operation will commence.\n");

        int ID = imagePipeline.getTemplateID(boxes, false); // Check if there is something present, do not print internals
        // NOTE: DO NOT CALL IN RAPID SUCCESSION
        // TODO: See if this is related to period between calls or quantity of calls
        if (ID == -1) {
            // Handle error... or don't
        }
        else {
            // Completed scan without event
            // 0 for blank/nothing to spot
            // >0 matches template ID spotted
        }
#endif
        // End of vision stuff

        ros::Duration(0.1).sleep(); // Two second sleep per step
    }
    return 0;
}
