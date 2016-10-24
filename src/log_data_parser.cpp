#include "../include/log_data_parser.h"
#include <vector>

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

void str::LogDataParser::parseDataPerLine()
{
	std::string cur_line;
	if(std::getline(data_file_, cur_line))
	{
		std::cout<<cur_line<<std::endl;
	}

	if( cur_line[0] == 'L')
	{
		//Parsing data to laser data structure
	}
	else if( cur_line[0] == 'O' )
	{
		//Parsing data to odometry data structure
	}
	else
	{
		std::cerr << "[ERROR] Wrong data type " << cur_line[0] << std::endl;
	}
}

void str::LogDataParser::closeFile()
{
	data_file_.close();
}
