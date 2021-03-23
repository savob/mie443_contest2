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

const int timeLimit = 8 * 60; // Time limit in seconds

int main(int argc, char** argv) {
    // Monitor time elapsed
    time_t startTime = time(NULL);
    float secondsElapsed = 0;

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

    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n, boxes);

    // Spin a couple times to sync properly
    for (int i = 0; i < 3; i++) {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    // Record starting position
    std::vector<float> startPosition(3);
    startPosition[0] = robotPose.x;
    startPosition[1] = robotPose.y;
    startPosition[2] = robotPose.phi;
    ROS_INFO("Starting position:\n\tx: %5.2f\ty: %5.2f\tyaw: %5.2f", startPosition[0], startPosition[1], startPosition[3]);

    // Initialize path planner
    pathPlanning pathPlanned(n, boxes, startPosition, true); // Path data

    // Variable to record identification of boxes
    std::vector<int> boxIDs(boxes.coords.size()); // Recoding IDs of each box
    int currentStop = 0;

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
        visionSystemTest("", boxes, imagePipeline, true);
        return 0;
#endif
#ifdef MOTION_TEST
        navigationSystemTest(pathPlanned);
        return 0;
#endif

        // =======================================================
        // Actual loop code
        int boxNumber = pathPlanned.idealOrder[currentStop]; // Stores current box index

        // =======================================================
        // Motion code
        // Start by checking if it is time to return to start and terminate

        if (currentStop == pathPlanned.idealOrder.size()) {
            ROS_WARN("Done all stops, returning to start.");
            bool atEnd = pathPlanned.goToCoords(pathPlanned.startCoord);

            // Check if it has successfully returned to the start
            if (atEnd) ROS_WARN("Reached end in %.1f seconds\n", secondsElapsed);
            else ROS_ERROR("FAILED TO REACH END! (%.1f seconds elapsed).\n\tTerminating anyways.\n", secondsElapsed);
            
            break; // Break out of while loop (to terminate)
        }

        // Inform us what step is started and target
        ROS_INFO("Starting step %d, target is box %d.", currentStop + 1, boxNumber);

        // Do normal motion otherwise
        bool atSpotToScan = pathPlanned.goToStop(currentStop);
        
        if (atSpotToScan) ROS_INFO("Reached stop %d (box %d)", currentStop + 1, boxNumber);
        else ROS_ERROR("Failed to reach stop %d (box %d)", currentStop + 1, boxNumber);

        if (atSpotToScan) {
            // Perhaps some code to try and align bot with target even if unreachable
        }

        // =======================================================
        // Vision code
        // Scan regardless of if we succcessfully reached destination or not
        // since we might be lucky and be able to see it from where we stand

        ros::spinOnce(); // Update video feed after moving

        // Check if there is something present
        int ID = imagePipeline.getTemplateID(boxes, false);
        
        if (ID >= 0) boxIDs[currentStop] = ID; // Good scan (error-free) 
        else {
            // Handle error... or don't 
        }  

        // =======================================================
        // Record results 
        // Every step, so in the event of interruption our progress isn't lost
        writeLog(boxes, pathPlanned.idealOrder, boxIDs);

        // =======================================================
        // Increment to next step
        currentStop++;
        std::cout << "\n\n"; // Make a break in terminal between stops

        // Sleep and record elapsed time
        ros::Duration(0.1).sleep();
        secondsElapsed = time(NULL) - startTime;
    }
    
    // =======================================================
    // Handle proper closure

    if (secondsElapsed >= timeLimit) {
        // Time's up handle proper closure
        ROS_WARN("\nTIME'S UP! (%d seconds)\n RECORDING OUTPUT AND TERMINATING.\n", timeLimit);
    }
    writeLog(boxes, pathPlanned.idealOrder, boxIDs, true); // Write results before closing

    ROS_FATAL("Ending now. Goodbye.");
    return 0;
}
