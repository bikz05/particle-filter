#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <typeinfo>
#include <opencv2/opencv.hpp>

namespace str{
	template <typename T>
	class Map;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const str::Map<T>& map);

namespace str{

template <typename T>
class Map{
	private:
		std::vector<std::vector<T>> map_;
		int global_mapsize_x_, global_mapsize_y_, resolution_;
		int autoshifted_x_, autoshifted_y_;
		std::pair<int, int> global_map_;
	public:
		Map(std::vector<std::vector<T>> map):map_(map){
		}

		Map(std::string fileName){
			loadMap(fileName);
		}

		void setLocation(int x, int y, T value){
			this->map_[y][x] = value;
		}

		T getLocation(int x, int y) const{
			return this->map_[y][x];
		}

		int loadMap(std::string fileName);

		cv::Mat_<T> vec2Mat();

		cv::Mat getImage();

		template <typename U>
		friend std::ostream& ::operator<<(std::ostream& out, const str::Map<U>& map);
};

}


template <typename T>
std::ostream& operator<<(std::ostream& out, const str::Map<T>& map){
	for(auto it_row: map.map_){
		for(auto it_col: it_row)
			out << it_col << "\t";
			out << std::endl;
	}
	return out;
}

#endif //MAP_H
