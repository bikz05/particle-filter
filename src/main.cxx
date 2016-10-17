#include <iostream>
#include "../include/pose.h"

int main(int argc, char ** argv){
	std::cout << "Particle Filter Assignment" << std::endl;
	// Set the Pose
	// (x, y, theta, weight)
	str::Pose<double> pose(2, 3, 2.3, 1);
	// Print the pose
	std::cout << pose;
}
