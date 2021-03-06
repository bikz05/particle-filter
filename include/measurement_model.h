#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include "../include/pose.h"
#include "../include/laser_reading.h"
#include "../include/odometry_reading.h"
#include "../include/distance_table.h"
#include "../include/map.h"
#include <numeric>


namespace str
{
	class MeasurementModel;
}

namespace str
{
	class MeasurementModel
	{
		public:
			//tuning parameters for sensor model(TBD)
			struct tuningParameters
			{
				double w_hit = 1000; //1000
				double w_short = 10; //10
				double w_max = 0.5;  //0.5
				double w_rand = 0.5; //0.5
				double sigma_hit = 300; //300
				double lambda_short = 0.005; //0.005
			};
			MeasurementModel(){};
			MeasurementModel(str::Map<double>& map);
			virtual ~MeasurementModel();
			double getProbability(const str::LaserReading<double>& laser_reading, const str::Pose<double>& pose);
			void setParameters(tuningParameters& input_param);
			void UnitTest();

		private:
			str::Map<double> map_;
			
			
			void getDistTableByPose(const str::Pose<double>& pose, std::vector<double>& predict_dist);
			void selectDistTableByTheta(const unsigned int& theta_in_grid, std::vector<double>& dist_table_per_grid, std::vector<correspondence>& corr_per_grid);
			void poseCoordToGrid(const str::Pose<double>& pose_coord, str::Pose<unsigned int>& pose_grid);
			double getProbFromBeamModel(const double& measurement, const double& predict_measurement);
			double prob_hit(const double& measurement, const double& predict_measurement);
			double prob_short(const double& measurement, const double& predict_measurement);
			double prob_max(const double& measurement);
			double prob_rand(const double& measurement);
			inline str::Pose<double> getSensorPose(const str::Pose<double>& pose)
				{return str::Pose<double> (pose.getX()+sensor_dist_*std::cos(pose.getTheta()), pose.getY()+sensor_dist_*std::sin(pose.getTheta()), pose.getTheta(), pose.getWeight());}
			

			str::DistanceTable* dist_table_;
			std::vector<double> selected_dist_table_;
			std::vector<correspondence> selected_corr_table_;
			tuningParameters params_;
			double z_max_ = MAX_MEARSUREMENT_DIST;//need to discuss how to get this one.

			const double sensor_dist_ = 25.0;

		
		
	};

}


#endif //MEASUREMENT_MODEL_H
