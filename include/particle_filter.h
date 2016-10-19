#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <iostream>
#include "pose.h"
#include "odometry_reading.h"
#include "laser_reading.h"
#include "map.h"

namespace str{

template <typename T>
class ParticleFilter{
	private:

	public:
		ParticleFilter();
		void mcl(std::vector<str::Pose<T>> x_tm1, str::OdometryReading<T> u_t, str::LaserReading<T> z_t, str::Map<T> m);
};

}

#endif //PARTICLE_FILTER_H
