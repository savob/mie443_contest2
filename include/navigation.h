#pragma once
#include <vector>

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		static bool moveToGoal(std::vector<float> goal); // Overloaded to simplify our code which uses vectors to store coordinates
};
