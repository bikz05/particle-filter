#include <iostream>
#include <sstream>
#include <fstream>
// #include <random>
#include "motion_model_odometry.h"


int main(int argc, char** argv)
{
	// cout << "Hello world " << endl;

	// std::string filename;

	// if(argc > 1){
	// 	filename = argv[1];
	// }

	// std::ifstream file(filename.c_str());

	// std::string input((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));

	// std::istringstream input_stream;
	// input_stream.str(input);

	// // std::ofstream output_file;

	// // output_file.open("odom_only_data1.txt");

	// for (std::string line; std::getline(input_stream, line);){
		
	// 	if (line[0] == 'O'){
	// 		line.erase(0,2);	
	// 		std::cout << line << std::endl;
	// 		// output_file << line << std::endl;
	// 	}
			
	// }


	Motion_Model_Odom motion_model;

	str::Pose<double> pose(-94.234001, -139.953995, -1.342158, 1);

	str::OdometryReading<double> odoRdg1(-94.234001, -139.953995, -1.342158, 1);

	str::OdometryReading<double> odoRdg2(-93.980003, -140.207993, -1.340413, 2);

	Motion_Model_Odom::pControl u(odoRdg1, odoRdg2);

	std::cout << motion_model.Sample(u, pose) << std::endl;

	

}
