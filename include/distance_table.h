#ifndef DISTANCE_TABLE_H
#define DISTANCE_TABLE_H
#include <vector>
#include <unordered_map>
#include "../include/pose.h"
#include "../include/map.h"
#include "../include/global_nums.h"

typedef std::vector<std::vector<std::vector<double>>> distTable;
namespace str
{
	class DistanceTable;
}

struct correspondence
{
	unsigned int x0;
	unsigned int y0;
	unsigned int x1;
	unsigned int y1;
	correspondence(unsigned int px0, unsigned int py0, unsigned int px1, unsigned int py1):
	x0(px0),y0(py0),x1(px1),y1(py1){}
};

namespace str
{
	class DistanceTable
	{
		public:
			DistanceTable(const std::vector<std::vector<double>>& map);
			DistanceTable(str::Map<double>& map);
			virtual ~DistanceTable();
			void buildDistanceTable(distTable& dist_table);
			void calculateDistancePerGrid(const unsigned int& x, const unsigned int& y, std::vector<double>& dist_per_grid);
			std::vector<double> getDistPerGrid(const unsigned int& x, const unsigned int& y);
			std::vector<correspondence> getCorrespondencePerGrid(const unsigned int& x, const unsigned int& y);
			//for unit test
			std::vector<double> dist_per_grid_;
			std::vector<correspondence> correspondence_per_grid_;



			
			
		private:
			#define MEASUREMENT_PER_GRID		360//measure 360 per grid
			str::Map<double> c_map_;
			double ray_step_size_;//unit in grid
			double theta_step_size_;//unit in rad
			std::vector<std::vector<double>> map_;
			std::unordered_map<unsigned int, std::vector<double>> dist_hashtable_;
			std::unordered_map<unsigned int, std::vector<correspondence>> corr_hashtable_;
			inline unsigned int coordToGridID(const unsigned int& x, const unsigned int& y){return x + MAP_X_SIZE_IN_GRID*y;};
			
			double calculateDistance(const unsigned int& x, const unsigned int& y, const double& theta);
			inline double getMapValue(const unsigned int& x, const unsigned int& y){return map_[y][x];}
			
		
		
	};

}


#endif //DISTANCE_TABLE_H
