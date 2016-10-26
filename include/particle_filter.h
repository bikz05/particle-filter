#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <iostream>
#include "pose.h"
#include "odometry_reading.h"
#include "laser_reading.h"
#include "motion_model_odometry.h"
#include "measurement_model.h"
#include "map.h"
#include <random>
#include <utility>
#include <ctime>
#include <algorithm>

namespace str{

template <typename T>
class ParticleFilter{
	private:
		int no_samples_;
		int valid_samples_;
		std::vector<str::Pose<T>> samples_;
		std::vector<str::Pose<T>> samplesTemp_;
		std::vector<T> weights_;
		str::Map<double> map_;
		Motion_Model_Odom motionModel_;
		MeasurementModel measurementModel_;
		std::random_device rd_;
		double w_slow = 0;
		double w_fast = 0;
		// TODO Update these
		double a_slow = .2;
		double a_fast = .5;
		void importanceSampling();
		void lowVarianceSampling();
		void augmentedSampling();

	public:
		ParticleFilter(int no_samples);
		std::vector<str::Pose<T>>& mcl(std::vector<str::Pose<T>>& x_tm1,
			const std::pair<str::OdometryReading<T>, str::OdometryReading<T>>& odoReadingPair,
			const str::LaserReading<T>& z_t);
		std::vector<str::Pose<T>>& predict(std::vector<str::Pose<T>>& x_tm1,
			const std::pair<str::OdometryReading<T>, str::OdometryReading<T>>& odoReadingPair);
		std::vector<str::Pose<T>>& update(const str::LaserReading<T>& z_t, int sampling_type=0);
};

}

#endif //PARTICLE_FILTER_H
