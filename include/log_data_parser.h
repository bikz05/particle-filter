#ifndef LOG_DATA_PARSER_H
#define LOG_DATA_PARSER_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../include/pose.h"
#include "../include/laser_reading.h"
#include "../include/odometry_reading.h"

enum Output
{
	LASER = 0,
	ODOM
};

namespace str
{
	class LogDataParser;
}

namespace str
{
	class LogDataParser
	{
		public:
			LogDataParser(std::string fileName);
			virtual ~LogDataParser();
			void closeFile();
			int parseDataPerLine();
		private:
			std::ifstream data_file_;
		public:
			OdometryReading<double> odom_reading;
			LaserReading<double> laser_reading;
			
	};

}


#endif //LOG_DATA_PARSER_H
