#include <cmath>
#include "../include/measurement_model.h"
#include "../include/global_nums.h"
#include "map.cpp"
/**
 * @brief Constructor
**/
str::MeasurementModel::MeasurementModel(str::Map<double>& map)
{
	map_ = map;
	dist_table_ = new str::DistanceTable(map_);
}

/**
 * @brief Destructor
**/
str::MeasurementModel::~MeasurementModel()
{
	
}

/**
 * @brief TBD, should be main interface
**/
double str::MeasurementModel::getProbability(const str::LaserReading<double>& laser_reading, const str::Pose<double>& pose)
{
	//Step1. get pre-cashing table using pose, should return 180 values
	//Step2. using table value and laser reading to compute prob using this->getProbFromBeamModel
	//Step3. calculate overall probability, product of all 180 measurements

	double overall_prob = 1;
	std::vector<double> predict_distance;
	this->getDistTableByPose(pose, predict_distance);
	std::vector<double> laser_data = laser_reading.getRanges();
	
	for (int i = 0; i < 180; ++i)
	{
		// get z_t_k* using pose and measurement idx
		double z_t_k_star = predict_distance.at(i);

		double prob = this->getProbFromBeamModel( laser_data[i] ,z_t_k_star);
		overall_prob *= prob;
	}

	return overall_prob;
}

/**
 * @brief Get predict distance from distance table based on laser pose
**/
void str::MeasurementModel::getDistTableByPose(const str::Pose<double>& pose, std::vector<double>& predict_distance)
{	
	//std::cout<<pose.getX()<<" "<<pose.getY()<<" "<<pose.getTheta()<<std::endl;
	str::Pose<double> laser_pose = this->getSensorPose(pose);
	//std::cout<<laser_pose.getX()<<" "<<laser_pose.getY()<<" "<<laser_pose.getTheta()<<std::endl;

	str::Pose<unsigned int> laser_pose_grid;
	this->poseCoordToGrid(laser_pose, laser_pose_grid);//this function transfer theta to degree


	unsigned int x = laser_pose_grid.getX(); 
	unsigned int y = laser_pose_grid.getY();
	unsigned int theta = laser_pose_grid.getTheta();
	std::vector<double> dist_table_per_grid(MEASUREMENT_PER_GRID);
	
	//std::cout<<x<<" "<<y<<" "<<theta<<std::endl;
	dist_table_->calculateDistancePerGrid(x, y, dist_table_per_grid);

	// for(auto ele:dist_table_per_grid)
	// {
	// 	std::cout<<ele<<" ";
	// }
	// std::cout<<std::endl;
	dist_table_per_grid = dist_table_->getDistPerGrid(x, y);
	std::vector<correspondence>corr_per_grid =  dist_table_->getCorrespondencePerGrid(x,y);
	
	//std::cout<<"corr size = "<<corr_per_grid.size()<<std::endl;
	this->selectDistTableByTheta(theta, dist_table_per_grid, corr_per_grid);
	// for(auto ele:dist_table_per_grid)
	// {
	// 	std::cout<<ele<<" ";
	// }

	predict_distance = dist_table_per_grid;
	selected_dist_table_ = dist_table_per_grid;
	selected_corr_table_ = corr_per_grid;
}

/**
 * @brief select dist table (1~180) by theta
**/
void str::MeasurementModel::selectDistTableByTheta(const unsigned int& theta_in_grid, std::vector<double>& dist_table_per_grid, std::vector<correspondence>& corr_per_grid)
{
	//TODO
	int robot_theta = theta_in_grid;
	unsigned int theta_start = (robot_theta - 90 + 1 + 360)%360;
	unsigned int theta_end = (robot_theta + 90)%360;
	
	//std::cout<<"size = "<<corr_per_grid.size()<<std::endl;
	if( theta_start < theta_end )
	{
		//std::cout<<"case 1"<<std::endl;
		unsigned int start_idx = theta_start;
		unsigned int end_idx = theta_end;
		std::cout<<start_idx<<" "<<end_idx<<std::endl;
		if( (end_idx - start_idx + 1) != 180)
			std::cout<<"wrong index in section 1"<<std::endl;
		std::vector<double> selected_dist_table(dist_table_per_grid.begin()+start_idx, dist_table_per_grid.begin()+end_idx+1);

		std::vector<correspondence> selected_corr(corr_per_grid.begin()+start_idx, corr_per_grid.begin()+end_idx+1);

		dist_table_per_grid = selected_dist_table;
		//corr_per_grid = selected_corr;
	}
	else if( theta_start > theta_end )
	{
		//std::cout<<"case 2"<<std::endl;
		unsigned int start1_idx = theta_start;
		unsigned int end1_idx = 359;
		

		unsigned int start2_idx = 0;
		unsigned int end2_idx = theta_end;
		
		if( (end1_idx - start1_idx + end2_idx - start2_idx + 2)!=180 )
			std::cout<<"wrong index in section 2"<<std::endl;

		std::vector<double> dist_temp_1(dist_table_per_grid.begin()+start1_idx, dist_table_per_grid.begin()+end1_idx+1);
		std::vector<double> dist_temp_2(dist_table_per_grid.begin()+start2_idx, dist_table_per_grid.begin()+end2_idx+1);

		std::vector<correspondence> corr_temp_1(corr_per_grid.begin()+start1_idx, corr_per_grid.begin()+end1_idx+1);
		std::vector<correspondence> corr_temp_2(corr_per_grid.begin()+start2_idx, corr_per_grid.begin()+end2_idx+1);
		// for(auto ele:dist_temp_1)
		// {
		// 	std::cout<<ele<<" ";
		// }
		// std::cout<<std::endl;

		std::vector<double> selected_dist_table = dist_temp_1;
		selected_dist_table.insert(selected_dist_table.end(), dist_temp_2.begin(), dist_temp_2.end());

		std::vector<correspondence> selected_corr = corr_temp_1;
		selected_corr.insert(selected_corr.end(), corr_temp_2.begin(), corr_temp_2.end());

		dist_table_per_grid = selected_dist_table;
		corr_per_grid = selected_corr;
	}
	else
	{
		std::cout<<"something wrong.."<<std::endl;
	}

}

/**
 * @brief tranfer pose coord to grid, theta in degree
**/
void str::MeasurementModel::poseCoordToGrid(const str::Pose<double>& pose_coord, str::Pose<unsigned int>& pose_grid)
{
	if(pose_coord.getX()<0 || pose_coord.getY() <0)
		std::cout<<"Error: coord out of range coord: "<<pose_coord.getX()<<" "<<pose_coord.getY()<<std::endl;


	str::Pose<double> copy_pose = pose_coord;
	pose_grid.setX(static_cast<unsigned int>(round(copy_pose.getX()/MAP_RESOLUTION)));
	pose_grid.setY(static_cast<unsigned int>(round(copy_pose.getY()/MAP_RESOLUTION)));
	//std::cout<<"theta = "<<copy_pose.getTheta()<<std::endl;
	// if( copy_pose.getTheta() < 0)
	// 	std::cout<<"currently it did not support negtive theta: "<<copy_pose.getTheta()<<std::endl;

	double theta = copy_pose.getTheta();
	if(theta > M_PI || theta < -M_PI)
		std::cout<<"Error: theta out of range [M_PI, -M_PI], theta: "<<theta<<std::endl;
	if(theta < 0)
		theta += 2*M_PI;

	pose_grid.setTheta(static_cast<unsigned int>(round((theta/(M_PI))*180)));
}


/**
 * @brief getProbFromBeamModel
**/
double str::MeasurementModel::getProbFromBeamModel(const double& measurement, const double& predict_measurement)
{
	double w_p_hit = params_.w_hit * this->prob_hit(measurement, predict_measurement);
	double w_p_short = params_.w_short * this->prob_short(measurement, predict_measurement);
	double w_p_max = params_.w_max * this->prob_max(measurement);
	double w_p_rand = params_.w_rand * this->prob_rand(measurement);

	//std::cout<<"prob: "<<w_p_hit + w_p_short + w_p_max + w_p_rand<<" measure: "<<measurement<<" predict: "<<predict_measurement<<" w_p_hit: "<<w_p_hit<<" w_p_short: "<<w_p_short<<" w_p_max: "<<w_p_max<<" w_p_rand: "<<w_p_rand<<std::endl;

	return w_p_hit + w_p_short + w_p_max + w_p_rand;
}

/**
 * @brief tuneParameters
**/
void str::MeasurementModel::setParameters(tuningParameters& input_param)
{
	params_ = input_param;
}

/**
 * @brief tuneParameters
**/
double str::MeasurementModel::prob_hit(const double& measurement, const double& predict_measurement)
{
	double variance = params_.sigma_hit * params_.sigma_hit;
	double eta = 1/sqrt(2 * M_PI * variance);
	double diff = measurement - predict_measurement;
	double p_hit = eta*exp(-0.5 * diff * diff / variance);
	return (measurement >= 0 && measurement <= z_max_) ? p_hit : 0.0;
}

/**
 * @brief tuneParameters
**/
double str::MeasurementModel::prob_short(const double& measurement, const double& predict_measurement)
{
	double eta  = 1/(1 - exp(-params_.lambda_short * predict_measurement));
	double p_short = eta*params_.lambda_short * exp(-params_.lambda_short * measurement);
	return (measurement >= 0 && measurement <= predict_measurement) ? p_short : 0.0;
}

/**
 * @brief tuneParameters
**/
double str::MeasurementModel::prob_max(const double& measurement)
{
	return (measurement == z_max_) ? 1.0 : 0.0;
}

/**
 * @brief tuneParameters
**/
double str::MeasurementModel::prob_rand(const double& measurement)
{
	return (measurement >= 0 && measurement < z_max_) ? 1/z_max_ : 0.0;
}

/**
 * @brief UnitTest
**/
void str::MeasurementModel::UnitTest()
{
	//1.test select dist table by theta
	std::cout<<"Test get artificial data by theta(degree)"<<std::endl;
	std::vector<double> dist_table_per_grid(MEASUREMENT_PER_GRID);
	std::vector<correspondence> coor_per_grid( MEASUREMENT_PER_GRID, correspondence(0,0,0,0));
	std::iota(dist_table_per_grid.begin(), dist_table_per_grid.end(), 0);
	

	unsigned int theta_in_grid = 90;
	this->selectDistTableByTheta( theta_in_grid, dist_table_per_grid, coor_per_grid);
	for(auto ele:dist_table_per_grid)
	{
		std::cout<<ele<<" ";
	}
	std::cout<<std::endl;

	std::cout<<"size = "<<dist_table_per_grid.size()<<std::endl;





	//2, test coord to grid and get distance table
	std::cout<<"Test get distTable by pose"<<std::endl;
	str::Map<double> map("../data/map/wean.dat");
	cv::Mat im = map.getImage();
	dist_table_ = new str::DistanceTable(map);
	cvtColor( im, im, cv::COLOR_GRAY2BGR );
	str::MeasurementModel measurement(map);
	//test num
	double x = 7500;
	double y = 7000;
	double theta = 2;
	
	str::Pose<double> pose(x, y, theta);
	std::vector<double> predict_dist;
	this->getDistTableByPose(pose, predict_dist);

	str::Pose<double> laser_pose = this->getSensorPose(pose);
	str::Pose<unsigned int> laser_grid;
	this->poseCoordToGrid(laser_pose, laser_grid);
	std::cout<<"laser coord = "<<laser_pose.getX()<<" "<<laser_pose.getY()<<" "<<laser_pose.getTheta()<<std::endl;
	std::cout<<"laser grid = "<<laser_grid.getX()<<" "<<laser_grid.getY()<<" "<<laser_grid.getTheta()<<std::endl;

	cv::Point2i laser_pos = cv::Point2i(laser_grid.getX(), laser_grid.getY());

	for(auto ele:predict_dist)
	{
		std::cout<<ele<<" ";
	}
	std::cout<<std::endl;


	for(auto coord:this->selected_corr_table_)
	{
		//std::cout<<coord.x0<<" "<<coord.y0<<" "<<coord.x1<<" "<<coord.y1<<std::endl;
		cv::Point2i pt1 =  cv::Point2i(coord.x0, coord.y0);
		cv::Point2i pt2 =  cv::Point2i(coord.x1, coord.y1);
		cv::line(im, pt1, pt2, cv::Scalar(255,0,0), 1, 8, 0);
	}
	cv::circle(im, laser_pos, 2, cv::Scalar(0,0,255), -1, 8, 0);


	cv::namedWindow("MAP", cv::WINDOW_NORMAL);
	cv::imshow("MAP", im);
	cv::waitKey(0);
}