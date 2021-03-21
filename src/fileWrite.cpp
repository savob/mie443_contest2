#include "fileWrite.h"

void writeLog(Boxes boxList, std::vector<int> movePlan, std::vector<int> boxIDs, bool printInfo) {

    // Opens/creates log file (clears the contents if there was one prior to this)
    char * homeDir = std::getenv("HOME"); // Get home directory for user
    std::string fileLocation = homeDir + logfile; // Combines with the prefered file path
    
    std::ofstream outputFile;
    outputFile.open(fileLocation.c_str(), std::ios_base::out | std::ios_base::trunc);

    if (outputFile.is_open()) {

        std::string headerText = "Tag ID (0 for no ID) - Coordinates (x (m), y (m), yaw (rad)) - new or duplicate (dup)\n\n";
        outputFile << headerText;
        if (printInfo) {
            ROS_INFO("Results of our run.\n");
            std::cout << headerText;
        }

        // ================================================
        // Read box IDs and their coordinates into the file

        // Records if a template has already appeared (plus one for blank (at index 0))
        bool alreadyTagged[boxList.templates.size() + 1];
        for (int i = 0; i < (boxList.templates.size() + 1); i++) alreadyTagged[i] = false;

        // Go through each entry and record things
        for (int i = 0; i < boxIDs.size(); i++) {

            // Record tag ID for that stop
            char tagText[10];
            sprintf(tagText,"Tag %2d", boxIDs[i]);

            // Record duplicate status
            char dupText[7];
            if (alreadyTagged[boxIDs[i]] == true) {
                sprintf(dupText, "dup");
            }
            else {
                sprintf(dupText, "new");
                alreadyTagged[boxIDs[i]] = true; // Mark down it has already been listed
            }

            // Record coordinates for the stop from move list
            std::vector<float> curCoords = boxList.coords[movePlan[i]];
            char coordText[25];
            sprintf(coordText, "(%5.2f, %5.2f, %6.3f)", curCoords[0], curCoords[1], curCoords[2]);

            // Output the entry
            char outputBuffer[150];
            sprintf(outputBuffer, "%s - %s - %s\n", tagText, coordText, dupText);

            outputFile <<  outputBuffer; // Write to file
            if (printInfo) std::cout << outputBuffer;
        }

        outputFile.close(); // Must close file once complete

        ROS_INFO("\n\nFile with results written to:\n%s\n", fileLocation.c_str());
    }
    else {
        // File failed to open
        ROS_FATAL("\n\nUnable to prepare output file at:\n%s\n", fileLocation.c_str());
    }
}