#include "../include/particle_filter.h"

template <typename T>
str::ParticleFilter<T>::ParticleFilter(int no_samples):no_samples_(no_samples_), samples_(no_samples, str::Pose<double>()),
	samples_temp_(no_samples, str::Pose<double>()), weights_(std::vector<T>(no_samples, 1)){
}

template <typename T>
std::vector<str::Pose<T>>& str::ParticleFilter<T>::mcl(const std::vector<str::Pose<T>>& x_tm1,
		const std::pair<str::OdometryReading<T>, str::OdometryReading<T>>& odoReadingPair,
		const str::LaserReading<T>& z_t){

	// This is needed in case we use adaptive sampling.
	/**
	this->no_samples_ = x_tm1.size();
	this->samples_temp_ = std::vector<T>(this->no_samples_);
	this->samples_ = std::vector<T>(this->no_samples_);
	**/

	this->predict(x_tm1, odoReadingPair);
	this->update(z_t);
	return this->samples_;
}

template <typename T>
std::vector<str::Pose<T>>&  str::ParticleFilter<T>::predict(const std::vector<str::Pose<T>>& x_tm1,
		const std::pair<str::OdometryReading<T>, str::OdometryReading<T>>& odoReadingPair){
	for(int m = 0; m < this->samples_temp_.size(); m++){
		// Sample from the motion model
		this->samples_temp_[m] = motionModel_.Sample(odoReadingPair, x_tm1[m]);
		// TODO: Sample from the measurement model
		// this->weights_[m] = measurementModel_.getProbFromBeamModel(this->samples_temp_[m]);
	}
	return this->samples_temp_;
}

template <typename T>
std::vector<str::Pose<T>>&  str::ParticleFilter<T>::update(const str::LaserReading<T>& z_t){
	std::mt19937 gen(rd_());
	std::discrete_distribution<> d(this->weights_.begin(), this->weights_.end());
	for(int m = 0; m < this->samples_temp_.size(); m++){
		int sample_id = d(gen);
		this->samples_[m] = this->samples_temp_[sample_id];
	}
	return this->samples_;
}
