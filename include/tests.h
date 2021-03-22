#pragma once

#include "fileWrite.h"
#include "imagePipeline.h"
#include <dirent.h>         // Used for reading in the test files
#include "pathPlanning.h"
#include "navigation.h"

// "#define"s used to run tests. Comment out any unwanted tests
// These are used over standard "if"s since these will omit the code from the compilation properly

//#define FILE_WRITE_TEST       // Execute file output test
//#define VISION_SAMPLES_TEST   // Run test code for vision (go through test files)
#define MOTION_TEST           // Test motion functions / systems

const std::string testPhotoFolder = "/catkin_ws/src/mie443_contest2/testpics/"; // Relative to user home

void fileWriteTest(Boxes boxes, std::vector<int> movePlan, bool printStuff);
void visionSystemTest(std::string searchTerm, Boxes boxes, ImagePipeline &imagePipeline, bool printInnerWorks);
void navigationSystemTest(ros::NodeHandle& n, std::vector<float> startPosition);