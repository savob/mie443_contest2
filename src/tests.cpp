#include "tests.h"

void navigationSystemTest(pathPlanning pathPlanner) {
    ROS_WARN("\n\nMOTION TEST \n(will terminate once complete)\n");

    std::vector<float> testPoint(3, 0); // Initialize with 0s

    /* Random point test
    srand(time(NULL)); // Seed the random number generator with the current time

    // Generate a goal it can reach within the 6x6 maze
    do {
        testPoint[0] = -3.0 + (float)(rand() % 600) / 100.0;
        testPoint[1] = -3.0 + (float)(rand() % 600) / 100.0;
    } while(pathPlanner.checkPossible(testPoint) == false);

    Navigation::moveToGoal(testPoint);
    pathPlanner.clearCostMap();
    */

    // Go to all boxes
    for (int i = 0; i < pathPlanner.stopCoords.size(); i ++ ) {
        ROS_INFO("\n\tGOING TO STOP %d", i);
        testPoint = pathPlanner.stopCoords[i];
        bool gotThere = Navigation::moveToGoal(testPoint);

        // Handle initial failure
        if (!gotThere) {
            ROS_INFO("Initial attempt failed, recalculating target and retrying.");
            testPoint = pathPlanner.faceBoxPoint(i);
            //pathPlanner.clearCostMap(); // Clear cost map and reattempt motion
            gotThere = Navigation::moveToGoal(testPoint);
            
            // Absolute failure
            if (!gotThere) ROS_ERROR("Failed to reach box %d", i);
        }
    }
    
}

void fileWriteTest(Boxes boxes, std::vector<int> movePlan, bool printStuff) {
    ROS_WARN("\n\nRUNNING FILE OUTPUT TEST\n(will terminate once complete)\n");

    std::vector<int> boxIDs(boxes.coords.size()); // Dummy box IDs

    for (int i = 0; i < boxes.coords.size(); i++) {
        boxIDs[i] = i / 2; // i/2 to get duplicates
    }

    writeLog(boxes, movePlan, boxIDs);
}

void visionSystemTest(std::string searchTerm, Boxes boxes, ImagePipeline &imagePipeline, bool printInnerWorks) {
    ROS_WARN("\n\nRUNNING VISION TEST \nSearch term: \"%s\".\n(will terminate once complete)\n", searchTerm.c_str());

    // Find folder on user
    char * homeDir = std::getenv("HOME"); // Get home directory for user
    const std::string folderLocation = homeDir + testPhotoFolder;

    ROS_INFO("Test photo location used:\n%s\n", folderLocation.c_str());

    // Load in test files
    std::vector<std::string> fileNames;

    DIR *dr;
    struct dirent *en;
    dr = opendir(folderLocation.c_str()); // Open directory
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

    // Check if empty
    if (fileNames.empty()) {
        ROS_FATAL("\n\nNO TEST FILES FOUND!\nCheck for valid search string or folder location.\n");
        return;
    }

    std::sort(fileNames.begin(), fileNames.end()); // Sort files alphabetically

    int result[fileNames.size()]; // Result array

    // Go through each test file to ID
    for (int i = 0; i < fileNames.size(); i++) {
        std::string testFile = folderLocation + fileNames[i];
        imagePipeline.loadImage(testFile);
        result[i] = imagePipeline.getTemplateID(boxes, printInnerWorks);
    }

    // Print result summary
    printf("\nResults of image analysis, ID# and file name.\n");
    for (int i = 0; i < fileNames.size(); i++) {
        printf("ID %2d: %s\n", result[i], fileNames[i].c_str());
    }
}