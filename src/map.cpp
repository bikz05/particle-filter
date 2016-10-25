#include "../include/map.h"

template <typename T>
cv::Mat_<T> str::Map<T>::vec2Mat(){
	int rows = this->global_map_.first;
	int cols = this->global_map_.second;
	cv::Mat_<T> image(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		//image.row(i) = cv::Mat(map_[i]).t();
		image.row(i) = cv::Mat(map_[i]).t();
	}
	return image;
}

template <typename T>
std::vector<std::vector<T>> str::Map<T>::getMap()
{
	if(!is_load_map)
		std::cerr << "[ERROR] Map is not loaded " << std::endl;
	return map_;
}

template <typename T>
cv::Mat str::Map<T>::getImage(){
	auto im_raw = this->vec2Mat();
	cv::Mat image;
	T min, max;
	cv::minMaxIdx(im_raw, &min, &max);
	cv::convertScaleAbs(im_raw, image, 255 / max);
	return image;
}

template <typename T>
int str::Map<T>::loadMap(std::string fileName){

	std::ifstream fin(fileName);

	if(!fin.is_open()){
		std::cerr << "[ERROR] Could not open file " << fileName << std::endl;
		return -1;
	}

	int line_no = 0;
	std::string line;
	std::string value;
	std::stringstream line_stream(line);
	/**
	 	int global_mapsize_x_, global_mapsize_y_, resolution_;
		int autoshifted_x_, autoshifted_y_;
		std::pair<int> global_map_;

		robot_specifications->global_mapsize_x 8000
		robot_specifications->global_mapsize_y 8000
		robot_specifications->resolution 10
		robot_specifications->autoshifted_x 0
		robot_specifications->autoshifted_y 0

		global_map[0]: 800 800
	 */
	// Line No 1

	// TODO Automate this all.
	this->global_mapsize_x_ = 8000;
	this->global_mapsize_y_ = 8000;
	this->resolution_ = 10;
	this->autoshifted_x_ = 0;
	this->autoshifted_y_ = 0;
	this->global_map_ = {800, 800};

	for(int i = 0; i < 7; i++){
		std::getline(fin, line);
		// std::cout << line << std::endl;
	}

	while(std::getline(fin, line)){
		this->map_.push_back(std::vector<T>());
		std::stringstream line_stream(line);
		while(std::getline(line_stream, value, ' ')){
			if(typeid(T) == typeid(int))
				this->map_[line_no].push_back(std::stoi(value));
			else if(typeid(T) == typeid(float))
				this->map_[line_no].push_back(std::stof(value));
			else
				this->map_[line_no].push_back(std::stod(value));
		}
		line_no++;
	}

	is_load_map = true;
	return 0;
}
