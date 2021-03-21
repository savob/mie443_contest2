#pragma once

#include <fstream>
#include "boxes.h"
#include <vector>

const std::string logfile = "/Documents/team22results.txt"; // Where to record (relative to home)

void writeLog(Boxes boxList, std::vector<int> movePlan, std::vector<int> boxIDs, bool printInfo = false);
