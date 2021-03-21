#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <cmath>
#include <algorithm>
#include "fileWrite.h"
#include <time.h>
#include "tests.h"
#include "pathPlanning.h"

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n);
    ros::spinOnce(); // Initiate everything

    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        ROS_FATAL("Could not load coords or templates");
        return -1;
    }
    else {
        ROS_INFO("Box coordinates loaded successfully:");
        std::cout << "Box #\tx (m)\ty (m)\tyaw (rad)\n"; // Header

        // Output data for each box nicely
        for(int i = 0; i < boxes.coords.size(); ++i) {
            char buffer[100];
            sprintf(buffer, "%3d\t%5.2f\t%5.2f\t%6.3f\n", i, 
                boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
            std::cout << buffer;
        }
    }

    // Path planning from current location
    std::vector<int> bestRoute = findOptimalPath(boxes, robotPose, false);

    // Variable to record identification of boxes
    std::vector<int> boxIDs(boxes.coords.size()); // Recoding IDs of each box

    // Monitor time elapsed
    time_t startTime = time(NULL);
    int secondsElapsed = 0;
    const int timeLimit = 8 * 60; // Time limit in seconds

    while(ros::ok() && (secondsElapsed < timeLimit)) {
        ros::spinOnce();

        // =======================================================
        // Tests for features, these will only be executed once
        // Configured in "tests.h"
#ifdef FILE_WRITE_TEST
        fileWriteTest(boxes, bestRoute, false);
        return 0;
#endif

#ifdef VISION_SAMPLES_TEST
        // Leave search term for vision as "" for all test cases
        visionSystemTest("pup", boxes, imagePipeline, false);
        return 0;
#endif

        // =======================================================
        // Actual loop code
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        // =======================================================
        // Vision code
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
            // boxIDs[current stop in path] = ID;
        }

        // End of vision stuff

        // Sleep and record elapsed time
        ros::Duration(0.1).sleep();
        secondsElapsed = time(NULL) - startTime;
    }

    // Time's up handle proper closure
    ROS_WARN("\nTIME'S UP! (%d seconds)\n RECORDING OUTPUT AND TERMINATING.\n", timeLimit);

    writeLog(boxes, bestRoute, boxIDs); // Write results before closing
    return 0;
}
