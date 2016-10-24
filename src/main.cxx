#include <iostream>
#include <numeric>
#include <cstdlib>
#include "../include/pose.h"
#include "../include/odometry_reading.h"
#include "../include/laser_reading.h"
#include "map.cpp"
#include "particle_filter.cpp"
#include "../include/log_data_parser.h"
#include "../include/measurement_model.h"
//#include "../data/bee-map.c"

int main(int argc, char ** argv){
	std::cout << "Particle Filter Assignment" << std::endl;

	// Example --> Set the Pose
	// (x, y, theta, weight)
	str::Pose<double> pose_1(2, 3, 2.3, 1);
	str::Pose<double> pose_2(39, 7, 10.3, 1);
	// Print the pose
	std::cout << pose_1;

	// Example --> Represent an odometry measurement
	str::OdometryReading<double> odoRdg_1(pose_1, 12);
	str::OdometryReading<double> odoRdg_2(pose_2, 12.05);
	// or str::OdometryReading<double> odoRdg(2, 3, 2.3, 12);
	std::cout << odoRdg_1 << std::endl;

	// Example --> Represent a Laser Scan
	std::vector<double> range(180);
	std::iota(range.begin(), range.end(), 0);
	str::LaserReading<double> laserRdg(pose_1, range, 13.00);
	std::cout << laserRdg << std::endl;

	// Example --> Representing a map from vector
	//	std::vector<std::vector<double>> map_values(10, std::vector<double>(10));
	//	// Randomly fill values
	//	for(auto& it_row: map_values){
	//		for(auto& it_col: it_row)
	//			it_col = ((double) std::rand() / (RAND_MAX));
	//	}
	//	str::Map<double> map(map_values);
	//	std::cout << map;

	std::cout << "Testing Parser" << std::endl;
	// Example --> Parsing log data
	str::LogDataParser data_parser("../data/log/robotdata1.log");

	// test 5 lines
	for (int i = 0; i < 10; ++i)
	{
		auto parsing_result = data_parser.parseDataPerLine();
		if (parsing_result == LASER)
		{
			std::cout << data_parser.laser_reading << std::endl;
		}
		else if(parsing_result == ODOM){
			std::cout << data_parser.odom_reading << std::endl;
		}
		else{
			std::cout << "Parsing result is either laser or odom" << std::endl;
		}
	}

	data_parser.closeFile();


	// std::cout << "parser output " << data_parser.parseDataPerLine() << std::endl;
	// std::cout << "parser result:  \n"  << data_parser.laser_reading << std::endl;
	// std::cout << "parser output " << data_parser.parseDataPerLine() << std::endl;
	// std::cout << "parser result:  \n"  << data_parser.odom_reading << std::endl;

	

	std::cout << "Testing Parser Over" << std::endl;

	// Test Particle Filter
	str::ParticleFilter<double> particleFilter(5);
	// Some random poses
	std::vector<str::Pose<double>> xtm1;
	xtm1.push_back(str::Pose<double>(2, 3, 2, 2));
	xtm1.push_back(str::Pose<double>(2, 3, 2, 2));
	xtm1.push_back(str::Pose<double>(7, 3, 2, 2));
	xtm1.push_back(str::Pose<double>(7, 3, 2, 2));
	xtm1.push_back(str::Pose<double>(2, 3, 2, 2));
	std::pair<str::OdometryReading<double>, str::OdometryReading<double>> odoReadingPair = std::make_pair(odoRdg_1, odoRdg_2);
	//	odoReadingPair.first = odoRdg;
	//	odoReadingPair.second = odoRdg;

	// Odometry Reading
	auto new_samples = particleFilter.mcl(xtm1, odoReadingPair, laserRdg);

	for(auto sample: new_samples)
		std::cout << sample;

	// Example --> Show Vector
	str::Map<double> map("../data/map/wean.dat");
	// std::cout << map;
	cv::Mat im = map.getImage();
	cv::namedWindow("MAP", cv::WINDOW_NORMAL);
	cv::imshow("MAP", im);
	cv::waitKey(0);
}
