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




	// Motion_Model_Odom motion_model;

	// str::Pose<double> pose(-94.234001, -139.953995, -1.342158, 1);

	// str::OdometryReading<double> odoRdg1(-94.234001, -139.953995, -1.342158, 1);

	// str::OdometryReading<double> odoRdg2(-93.980003, -140.207993, -1.340413, 2);

	// std::cout << motion_model.Sample(std::make_pair(odoRdg1,odoRdg2), pose) << std::endl;



	
	std::string filename;

	if(argc > 1){
		filename = argv[1];
	}

	// std::ifstream file(filename.c_str());

	// std::string input((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));

	std::ifstream data_file;

	data_file.open(filename);

	// std::istringstream iss(input);

	std::string cur_line;

	std::getline(data_file, cur_line);

	std::istringstream iss(cur_line);

	std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                      			std::istream_iterator<std::string>{}};

    for(int i = 1; i < tokens.size(); ++i){
    	std::cout << std::stod(tokens[i]) << std::endl;
    }

    data_file.close();
}



