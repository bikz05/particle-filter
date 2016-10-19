#ifndef MAP_H
#define MAP_H

#include <vector>

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
	public:
		Map(std::vector<std::vector<T>> map):map_(map){
		}

		void setLocation(int x, int y, T value){
			this->map_[y][x] = value;
		}

		T getLocation(int x, int y) const{
			return this->map_[y][x];
		}

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
