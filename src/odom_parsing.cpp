#include <iostream>
#include <sstream>
#include <fstream>


// using namespace std;

int main(int argc, char** argv)
{
	// cout << "Hello world " << endl;

	std::string filename;

	if(argc > 1){
		filename = argv[1];
	}

	std::ifstream file(filename.c_str());

	std::string input((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));

	std::istringstream input_stream;
	input_stream.str(input);

	std::ofstream output_file;

	output_file.open("odom_only_data1.txt");

	for (std::string line; std::getline(input_stream, line);){
		
		if (line[0] == 'O'){
			line.erase(0,2);	
			std::cout << line << std::endl;
			output_file << line << std::endl;
		}
			
	}
	output_file.close();


	// std::cout << filename << std::endl;

}
