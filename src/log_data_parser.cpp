#include "../include/log_data_parser.h"
#include <vector>
#include <iterator>
/**
 * @brief Constructor
**/
str::LogDataParser::LogDataParser(std::string fileName)
{
	data_file_.open(fileName);

	if(!data_file_.is_open())
	{
		std::cerr << "[ERROR] Could not open file " << fileName << std::endl;
		exit(-1);
	}
}


/**
 * @brief Destructor
**/
str::LogDataParser::~LogDataParser()
{

}

/**
 * @brief Parser
**/

int str::LogDataParser::parseDataPerLine()
{
	std::string cur_line;

	std::getline(data_file_, cur_line);

	std::istringstream iss(cur_line);

	std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
	                      			std::istream_iterator<std::string>{}};


	if( cur_line[0] == 'L')
	{
		//Parsing data to laser data structure

	    // TODO: should clear the laser reading everytime we get here  
	    // need a clear function for laser_reading class           

	    laser_reading.setX(std::stod(tokens[4]));
	    laser_reading.setY(std::stod(tokens[5]));
	    laser_reading.setTheta(std::stod(tokens[6]));

	    std::vector<double> vec_reading;

	    for(unsigned int i = 7; i < tokens.size()-1 ; ++i){
	    	// std::cout << std::stod(tokens[i]) << std::endl;
	    	vec_reading.push_back(std::stod(tokens[i]));
    	}
    	laser_reading.setRanges(vec_reading);
    	laser_reading.setTimestamp(std::stod(tokens[tokens.size()-1]));

    	return LASER;
	}
	else if( cur_line[0] == 'O' )
	{
		//Parsing data to odometry data structure

		// TODO: should clear the odom reading everytime we get here  
	    // need a clear function for odom_reading class  

	    odom_reading.setX(std::stod(tokens[1]));
	    odom_reading.setY(std::stod(tokens[2]));
	    odom_reading.setTheta(std::stod(tokens[3]));
		odom_reading.setTimestamp(std::stod(tokens[4]));

		return ODOM;
	}
	else
	{
		std::cerr << "[ERROR] Wrong data type " << cur_line[0] << std::endl;
		return -1;
	}
}

void str::LogDataParser::closeFile()
{
	data_file_.close();
}
