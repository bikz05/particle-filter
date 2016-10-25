#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include "../include/pose.h"
#include "../include/laser_reading.h"
#include "../include/odometry_reading.h"
#include "../include/distance_table.h"
#include "../include/map.h"

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
				double w_hit = 1.0;
				double w_short = 1.0;
				double w_max = 1.0;
				double w_rand = 1.0;
				double sigma_hit = 1.0;
				double lambda_short = 1.0;
			};
			MeasurementModel(){};
			MeasurementModel(str::Map<double>& map);
			virtual ~MeasurementModel();
			double getProbability(const str::LaserReading<double>& laser_reading, const str::Pose<double>& pose);
			void setParameters(tuningParameters& input_param);
			
			void selectDistTableByTheta(const unsigned int& theta, std::vector<double>& dist_table_per_grid);
			void getDistTableByPose(const str::Pose<double>& pose, std::vector<double>& predict_dist);
			void poseCoordToGrid(const str::Pose<double>& pose_coord, str::Pose<unsigned int>& pose_grid);
			inline str::Pose<double> getSensorPose(const str::Pose<double>& pose)
				{return str::Pose<double> (pose.getX()+sensor_dist_*std::cos(pose.getTheta()), pose.getY()+sensor_dist_*std::sin(pose.getTheta()), pose.getTheta(), 1);}


			str::DistanceTable* dist_table_;
		private:
			str::Map<double> map_;
			
			
			//void getDistTableByPose(const str::Pose<double>& pose, std::vector<double>& predict_dist);
			//void selectDistTableByTheta(const unsigned int& theta, std::vector<double>& dist_table_per_grid);
			//void poseCoordToGrid(const str::Pose<double>& pose_coord, str::Pose<unsigned int>& pose_grid);
			double getProbFromBeamModel(const double& measurement, const double& predict_measurement);
			double prob_hit(const double& measurement, const double& predict_measurement);
			double prob_short(const double& measurement, const double& predict_measurement);
			double prob_max(const double& measurement);
			double prob_rand(const double& measurement);
			// inline str::Pose<double> getSensorPose(const str::Pose<double>& pose)
			// 	{return str::Pose<double> (pose.getX()+sensor_dist_*std::cos(pose.getTheta()), pose.getY()+sensor_dist_*std::sin(pose.getTheta()), pose.getTheta(), 1);}
			


			tuningParameters params_;
			double z_max_;//need to discuss how to get this one.

			const double sensor_dist_ = 25.0;

		
		
	};

}


#endif //MEASUREMENT_MODEL_H
