#include <iostream>
#include <numeric>
#include <cstdlib>
#include "../include/pose.h"
#include "../include/odometry_reading.h"
#include "../include/laser_reading.h"
#include "map.cpp"
#include "particle_filter.cpp"
#include "../include/log_data_parser.h"
#include "../include/measurement_model.h"
//#include "../data/bee-map.c"

std::vector<str::Pose<double>> getUniformParticles(str::Map<double> map, const int& num_particles, cv::Mat &image)
{
	const double k_map_val_threshold = 0.1;

	int count = 0;

	std::vector<str::Pose<double>> out_vec_pose;

	while(count < num_particles){

		double x = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*7999;
		double y = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*7999;
		double theta = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*6.283185;

		if( std::fabs(theta) > 6.283185)
			theta = std::fmod(theta, 6.283185);
		if(theta > 3.14159)
			theta -= 6.283185;
		if(theta < -3.14159)
			theta += 6.283185;

		int x_grid = std::round(x/10.0) >= 800 ? 799 : std::round(x/10.0);
		int y_grid = std::round(y/10.0) >= 800 ? 799 : std::round(y/10.0);

		if(map.getLocation(x_grid,y_grid) < k_map_val_threshold && map.getLocation(x_grid,y_grid) >= 0){

			out_vec_pose.push_back(str::Pose<double>(x,y,theta,1));

			cv::Point2i point = cv::Point2i(x_grid,y_grid);

			// std::cout << "wu la la " << std::endl;

			cv::circle(image, point, 1, cv::Scalar(255,0,0), -1, 8, 0);


			++count;
		}
	
	}

	return out_vec_pose;
}



int main(int argc, char ** argv)
{
	int num_of_sample = 3000;
	std::cout << "Particle Filter Assignment" << std::endl;


	// Example --> Represent an odometry measurement
	str::OdometryReading<double> odoRdg_t;
	str::OdometryReading<double> odoRdg_t_1;

	// // test 10 lines
	str::Map<double> map("../data/map/wean.dat");
	// std::cout << map;

	cv::Mat im = map.getImage();
	cvtColor( im, im, cv::COLOR_GRAY2BGR );
	cv::Mat im_display = im.clone();
	cv::namedWindow("MAP", cv::WINDOW_NORMAL);
	str::LogDataParser data_parser("../data/log/robotdata1.log");
	//str::LogDataParser data_parser("../data/log/ascii-robotdata3.log");
	auto uniform_particles = getUniformParticles(map, num_of_sample, im_display);
	cv::transpose(im_display, im_display);
 	cv::flip(im_display, im_display, 0);
	cv::imshow("MAP", im_display);
	cv::waitKey(5);
	auto new_samples = uniform_particles;
	bool odometry_execute_flag = false;
	bool odometry_first_reading = false;
	str::ParticleFilter<double> particleFilter(num_of_sample, new_samples);

	for (int i = 0; i < 2200; ++i)
	{
		// if(i<100)
		// 	num_of_sample = 5000;
		// else
		// 	num_of_sample = 3000;

		im_display = im.clone();
		auto parsing_result = data_parser.parseDataPerLine();
		if (parsing_result == LASER)
		{
			auto laserRdg = data_parser.laser_reading;
			// std::cout<<"laser reading: "<<laserRdg<<std::endl;
			
			odometry_execute_flag = true;
	 		new_samples = particleFilter.update(laserRdg,1);
	 		//std::cout << "LASER" << std::endl;

	 		// for (auto s:new_samples)
	 		// {
	 		// 	std::cout << s << std::endl;
	 		// }
		}
		else if((parsing_result == ODOM) && (odometry_execute_flag))
		{
			
			odoRdg_t = data_parser.odom_reading;
			if(odometry_first_reading == false)
			{
				odometry_first_reading = true;
				odoRdg_t_1 = odoRdg_t;
			}
			else
			{
				//std::cout<<"odometry reading t: "<<odoRdg_t<<std::endl;
				//std::cout<<"odometry reading t1: "<<odoRdg_t_1<<std::endl;
				//std::cout << "ODOM" << std::endl;

				std::pair<str::OdometryReading<double>, str::OdometryReading<double>> odoReadingPair = std::make_pair(odoRdg_t_1, odoRdg_t);
	 			new_samples = particleFilter.predict(new_samples, odoReadingPair);
	 			// for (auto s:new_samples)
		 		// {
		 		// 	std::cout << s << std::endl;
		 		// }
			}

			odoRdg_t_1 = odoRdg_t;
		}
		else
		{
			std::cout << "Parsing result is either laser or odom" << std::endl;
		}
		
		for(auto sample:new_samples)
		{
			cv::Point2i pt = cv::Point2i(static_cast<int>(sample.getX()/10),static_cast<int>(sample.getY()/10));
			cv::circle(im_display, pt, 3, cv::Scalar(0,0,255), -1, 8, 0);
		}	
		cv::transpose(im_display, im_display);
 		cv::flip(im_display, im_display, 0);


		cv::imshow("MAP", im_display);
		cv::waitKey(1);
		std::cout<<i<<std::endl;
	}

	data_parser.closeFile();
	
}





