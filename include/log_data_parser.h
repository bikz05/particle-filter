#ifndef LOG_DATA_PARSER_H
#define LOG_DATA_PARSER_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

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
			void parseDataPerLine();
		private:
			std::ifstream data_file_;
	};

}


#endif //LOG_DATA_PARSER_H
