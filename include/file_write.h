#pragma once

#include <fstream>
#include "boxes.h"
#include <vector>

//#define FILE_WRITE_TEST // Execute file test, comment out when not testing

const std::string logfile = "/Documents/locations.txt"; // Where to record (relative to home)

void writeLog(Boxes boxList, std::vector<int> movePlan, std::vector<int> boxIDs, bool printInfo = false);
