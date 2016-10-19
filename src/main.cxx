#include <iostream>
#include <numeric>
#include <cstdlib>
#include "../include/pose.h"
#include "../include/odometry_reading.h"
#include "../include/laser_reading.h"
#include "../include/map.h"
#include "../include/particle_filter.h"

int main(int argc, char ** argv){
	std::cout << "Particle Filter Assignment" << std::endl;

	// Example --> Set the Pose
	// (x, y, theta, weight)
	str::Pose<double> pose(2, 3, 2.3, 1);
	// Print the pose
	std::cout << pose;

	// Example --> Represent an odometry measurement
	str::OdometryReading<double> odoRdg(pose, 12);
	// or str::OdometryReading<double> odoRdg(2, 3, 2.3, 12);
	std::cout << odoRdg << std::endl;

	// Example --> Represent a Laser Scan
	std::vector<double> range(180);
	std::iota(range.begin(), range.end(), 0);
	str::LaserReading<double> laserRdg(pose, range, 13);
	std::cout << laserRdg << std::endl;

	// Example --> Representing a map
	std::vector<std::vector<double>> map_values(10, std::vector<double>(10));
	// Randomly fill values
	for(auto& it_row: map_values){
		for(auto& it_col: it_row)
			it_col = ((double) std::rand() / (RAND_MAX));
	}
	str::Map<double> map(map_values);
	std::cout << map;
}
