#include "file_write.h"

void writeLog(Boxes boxList, std::vector<int> movePlan, std::vector<int> boxIDs) {

    // Opens/creates log file (clears the contents if there was one prior to this)
    std::ofstream outputFile;
    outputFile.open(logfile, std::ios_base::out | std::ios_base::trunc);

    if (outputFile.is_open()) {

        std::string headerText = "Tag ID (0 for blank/failed to ID) - Duplicate Status - Coordinates (x, y, yaw)\n";
        outputFile << headerText;
        
        // ================================================
        // Read box IDs and their coordinates into the file

        // Records if a template has already appeared (plus one for blank (at index 0))
        bool alreadyTagged[boxList.templates.size() + 1];
        for (int i = 0; i < (boxList.templates.size() + 1); i++) alreadyTagged[i] = false;

        // Go through each entry and record things
        for (int i = 0; i < boxIDs.size(); i++) {

            // Record tag ID for that stop
            char tagText[10];
            sprintf(tagText,"Tag %2d - ", boxIDs[i]);
            outputFile << tagText;

            // Record duplicate status
            char dupText[7];
            if (alreadyTagged[boxIDs[i]] == true) {
                sprintf(dupText, "dup - ");
            }
            else {
                sprintf(dupText, "new - ");
                alreadyTagged[boxIDs[i]] = true; // Mark down it has already been listed
            }
            outputFile << dupText;

            // Record coordinates for the stop from move list
            std::vector<float> curCoords = boxList.coords[movePlan[i]];
            
            char coordText[25];
            sprintf(coordText, "(%5.2f, %5.2f, %6.3f)", curCoords[0], curCoords[1], curCoords[2]);
            outputFile << coordText;

            outputFile << "\n"; // Move to next line

            std::cout << tagText << dupText << coordText << "\n";
        }

        outputFile.close(); // Must close file once complete

        ROS_INFO("\n\nFile written to \"%s\".\n", logfile);
    }
    else {
        // File failed to open
        ROS_FATAL("\n\nUnable to open output file.\n\n");
    }
}