#include <cmath>
#include <opencv2/opencv.hpp>
#include "../include/distance_table.h"
#include "map.cpp"

/**
 * @brief Constructor
**/
str::DistanceTable::DistanceTable(const std::vector<std::vector<double>>& map):
ray_step_size_(0.5), theta_step_size_(2*M_PI/MEASUREMENT_PER_GRID), map_(map)
{

}

/**
 * @brief Constructor
**/
str::DistanceTable::DistanceTable(str::Map<double>& map):
ray_step_size_(0.5), theta_step_size_(2*M_PI/MEASUREMENT_PER_GRID)
{
	c_map_ = map;
	map_ = c_map_.getMap();
}

/**
 * @brief Destructor
**/
str::DistanceTable::~DistanceTable()
{

}

/**
 * @brief Build distance table
**/
void str::DistanceTable::buildDistanceTable(distTable& dist_table)
{
	dist_table.clear();

	for(unsigned int x = 0; x < MAP_X_SIZE_IN_GRID; x++)
	{
		std::vector<std::vector<double>> dist_vector_in_y;
		for(unsigned int y = 0; y < MAP_Y_SIZE_IN_GRID; y++)	
		{
			std::vector<double> dist_vector_per_grid(MEASUREMENT_PER_GRID);
			
			this->calculateDistancePerGrid(x, y, dist_vector_per_grid);

			dist_vector_in_y.push_back(dist_vector_per_grid);	
		}
		dist_table.push_back(dist_vector_in_y);
	}
}

std::vector<double> str::DistanceTable::getDistPerGrid(const unsigned int& x, const unsigned int& y)
{
	unsigned int id = this->coordToGridID(x,y);
	return dist_hashtable_[id];
}

std::vector<correspondence> str::DistanceTable::getCorrespondencePerGrid(const unsigned int& x, const unsigned int& y)
{
	unsigned int id = this->coordToGridID(x,y);
	return corr_hashtable_[id];
}


void str::DistanceTable::calculateDistancePerGrid(const unsigned int& x, const unsigned int& y, std::vector<double>& dist_per_grid)
{	
	double map_value = c_map_.getLocation(x,y);
	std::cout<<"map_value = "<<map_value<<std::endl;

	if( map_value == -1.0 || map_value == 1.0)
	{
		std::fill(dist_per_grid.begin(), dist_per_grid.begin()+MEASUREMENT_PER_GRID, 0.0);
	}
	else
	{
		double theta_in_rad = 0.0;
		for (unsigned int num = 0; num < MEASUREMENT_PER_GRID; num++) 
		{
            double dist = this->calculateDistance(x, y, theta_in_rad);
            //std::cout<<"dist = "<<dist<<std::endl;
            dist *= MAP_RESOLUTION; //unit from grid to cm
            dist_per_grid[num] = dist;
            theta_in_rad += theta_step_size_;
            
        }
	}
	dist_per_grid_ = dist_per_grid; //for unit test

	unsigned int id = this->coordToGridID(x,y);
	dist_hashtable_[id] = dist_per_grid;
	corr_hashtable_[id] = correspondence_per_grid_;
}

/**
 * @brief Calculate distance by grid map
 * @param x: grid coord
 * @param y: grid coord
 * @param theta: angle in rad
**/
double str::DistanceTable::calculateDistance(const unsigned int& x, const unsigned int& y, const double& theta)
{
	double cur_x = x;
	double cur_y = y;
	double step_size_x = ray_step_size_*cos(theta);
	double step_size_y = ray_step_size_*sin(theta);

	//std::cout<<step_size_x<<" "<<step_size_y<<" "<<std::endl;

	while( cur_x >= 0 && cur_x < MAP_X_SIZE_IN_GRID && cur_y >= 0 && cur_y < MAP_Y_SIZE_IN_GRID)
	{
		//double map_value = this->getMapValue(static_cast<unsigned int>(cur_x), static_cast<unsigned int>(cur_y) );
		double map_value = c_map_.getLocation(static_cast<unsigned int>(cur_x), static_cast<unsigned int>(cur_y) );
		
		std::cout<<map_value<<" ";
		
		if( map_value >= 0.98)
			break;
		cur_x += step_size_x;
		cur_y += step_size_y;
		//std::cout<<cur_x<<" "<<cur_y<<" "<<std::endl;
	}
	std::cout<<std::endl;
	double dx = cur_x - static_cast<double>(x);
	double dy = cur_y - static_cast<double>(y);
	//std::cout<<dx<<" "<<dy<<" "<<std::endl;
	std::cout<<cur_x<<" "<<cur_y<<" "<<std::endl;
	//for unit test
	correspondence corrd(x, y, static_cast<unsigned int>(x + dx), static_cast<unsigned int>(y + dy));
	correspondence_per_grid_.push_back(corrd);

	return sqrt(dx*dx + dy*dy);
}



//Unit test
int main(int argc, char** argv)
{
	str::Map<double> map("../data/map/wean.dat");
	// std::cout << map;
	cv::Mat im = map.getImage();
	cvtColor( im, im, cv::COLOR_GRAY2BGR );
	
	//cv::transpose(im, im);
    //cv::flip(im, im, 0);

	str::DistanceTable distance_table(map);
	distTable dist_table;
	//distance_table.buildDistanceTable(dist_table);

	unsigned int x = atoi(argv[1]);
	unsigned int y = atoi(argv[2]);

	std::vector<double> dist_per_grid(MEASUREMENT_PER_GRID);
	distance_table.calculateDistancePerGrid(x, y, dist_per_grid);
	
	cv::Point2i robot_pos = cv::Point2i(x, y);



	std::cout<<"distance per grid"<<std::endl;
	for(auto dist:distance_table.getDistPerGrid(x,y))
	{
		std::cout<<dist<<" ";
	}
	std::cout<<std::endl;

	for(auto coord:distance_table.getCorrespondencePerGrid(x,y))
	{
		std::cout<<coord.x0<<" "<<coord.y0<<" "<<coord.x1<<" "<<coord.y1<<std::endl;
		cv::Point2i pt1 =  cv::Point2i(coord.x0, coord.y0);
		cv::Point2i pt2 =  cv::Point2i(coord.x1, coord.y1);
		cv::line(im, pt1, pt2, cv::Scalar(255,0,0), 1, 8, 0);
	}
	cv::circle(im, robot_pos, 2, cv::Scalar(0,0,255), 3, 8, 0);
	cv::namedWindow("MAP", cv::WINDOW_NORMAL);
	cv::imshow("MAP", im);
	cv::waitKey(0);
	return 0;
}