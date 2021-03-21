#pragma once

#include "file_write.h"
#include "imagePipeline.h"
#include <dirent.h>         // Used for reading in the test files

//#define FILE_WRITE_TEST // Execute file test, comment out when not testing

#define VISION_SAMPLES_TEST  // Run test code for vision (go through test files)
// Comment above out if not testing vision

const std::string testPhotoFolder = "/catkin_ws/src/mie443_contest2/testpics/"; // Relative to user home

void fileWriteTest(Boxes boxes, std::vector<int> movePlan, bool printStuff = false);
void visionSystemTest(std::string searchTerm, Boxes boxes, ImagePipeline &imagePipeline, bool printInnerWorks = false);