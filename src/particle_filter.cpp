#include "../include/particle_filter.h"

template <typename T>
str::ParticleFilter<T>::ParticleFilter(int no_samples):no_samples_(no_samples), samples_(no_samples, str::Pose<double>()),
	samplesTemp_(no_samples, str::Pose<double>()), weights_(std::vector<T>(no_samples, 1)){
}

template <typename T>
std::vector<str::Pose<T>>& str::ParticleFilter<T>::mcl(const std::vector<str::Pose<T>>& x_tm1,
		const std::pair<str::OdometryReading<T>, str::OdometryReading<T>>& odoReadingPair,
		const str::LaserReading<T>& z_t){

	// This is needed in case we use adaptive sampling.
	/**
	this->no_samples_ = x_tm1.size();
	this->samplesTemp_ = std::vector<T>(this->no_samples_);
	this->samples_ = std::vector<T>(this->no_samples_);
	**/

	this->predict(x_tm1, odoReadingPair);
	this->update(z_t, 1);
	return this->samples_;
}

template <typename T>
std::vector<str::Pose<T>>&  str::ParticleFilter<T>::predict(const std::vector<str::Pose<T>>& x_tm1,
		const std::pair<str::OdometryReading<T>, str::OdometryReading<T>>& odoReadingPair){
	for(int m = 0; m < this->samplesTemp_.size(); m++){
		// Sample from the motion model
		this->samplesTemp_[m] = motionModel_.Sample(odoReadingPair, x_tm1[m]);
		// TODO: Sample from the measurement model
		// this->weights_[m] = measurementModel_.getProbFromBeamModel(this->samplesTemp_[m]);
	}
	return this->samplesTemp_;
}

template <typename T>
std::vector<str::Pose<T>>&  str::ParticleFilter<T>::update(const str::LaserReading<T>& z_t, int samplingType){

	// Perform the samlping
	if(samplingType == 0){
		importanceSampling();
	}
	else if(samplingType == 1){
		lowVarianceSampling();
	}
	return this->samples_;
}

template <typename T>
void str::ParticleFilter<T>::importanceSampling(){
	std::mt19937 gen(rd_());
	std::discrete_distribution<> d(this->weights_.begin(), this->weights_.end());
	for(int m = 0; m < this->samplesTemp_.size(); m++){
		int sample_id = d(gen);
		this->samples_[m] = this->samplesTemp_[sample_id];
	}
}

template <typename T>
void str::ParticleFilter<T>::lowVarianceSampling(){
	// Normalize the weights
	double n = std::accumulate(this->weights_.begin(), this->weights_.end(), 0);

	std::srand(std::time(0));
	std::cout << no_samples_;

	double r = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) / this->no_samples_;
	double c = this->weights_[0] / n;
	int i = 0;

	for(int m = 0; m < this->samplesTemp_.size(); m++){
		double U = r + 1.0 * m / this->no_samples_;
		while(U > c){
			i++;
			c += this->weights_[i] / n;
		}
		this->samples_[m] = this->samplesTemp_[i];
	}
}
