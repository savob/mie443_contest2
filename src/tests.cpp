#include "tests.h"

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