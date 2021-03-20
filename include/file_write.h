#pragma once

#include <fstream>
#include "boxes.h"
#include <vector>

const char logfile[] = "~/Documents/locations.txt"; // The file to record results to

void writeLog(Boxes boxList, std::vector<int> movePlan, std::vector<int> boxIDs);
