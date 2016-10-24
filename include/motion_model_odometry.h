#ifndef MOTION_MODEL_ODOMETRY_H
#define MOTION_MODEL_ODOMETRY_H

#include <queue>
#include <random>
#include <cmath>
#include "../include/pose.h"
#include "../include/odometry_reading.h"

// namespace str{
// 	template <typename T>
// 	class Motion_Model_Odom;
// }

// namespace str{



// template <typename T>
class Motion_Model_Odom
{
public:
	Motion_Model_Odom();
	~Motion_Model_Odom();

	// typedef str::OdometryReading<T> OdomRdg;
	typedef str::OdometryReading<double> OdomRdg;
	// typedef std::pair<OdomRdg,OdomRdg> pControl;

	void setParam(const double& a1, const double& a2, const double& a3, const double& a4){
		this->a1_ = a1;
		this->a2_ = a2;
		this->a3_ = a3;
		this->a4_ = a4;
	};

	// str::Pose<T> Sample(const std::pair<OdomRdg,OdomRdg>& control_u, const str::Pose<T>& prev_pose);
	str::Pose<double> Sample(const std::pair<OdomRdg,OdomRdg>& control_u, const str::Pose<double>& prev_pose);

private:

	// parameters
	double a1_, a2_, a3_, a4_;

	// make it static just in case someone wants to use this for sampling without instantiate the object
	double SampleNormal(const double& b_squre);

	
};

// }

#endif