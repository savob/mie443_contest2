#pragma once

#include <fstream>
#include "boxes.h"
#include <vector>

#define FILE_WRITE_TEST // Execute file test, comment out when not testing

const char logfile[] = "~/Documents/locations.txt"; // The file to record results to

void writeLog(Boxes boxList, std::vector<int> movePlan, std::vector<int> boxIDs);
