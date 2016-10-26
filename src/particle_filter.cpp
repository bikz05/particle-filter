#include "../include/particle_filter.h"

template <typename T>
str::ParticleFilter<T>::ParticleFilter(int no_samples):no_samples_(no_samples), samples_(no_samples, str::Pose<double>()),
	samplesTemp_(no_samples, str::Pose<double>()), weights_(std::vector<T>(no_samples, 1)),
	map_(str::Map<double>("../data/map/wean.dat")), measurementModel_(map_){
	std::cout << "Map Value = " << map_.getLocation(2, 3) << std::endl;
	std::srand(std::time(0));
}

template <typename T>
std::vector<str::Pose<T>>& str::ParticleFilter<T>::mcl(std::vector<str::Pose<T>>& x_tm1,
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
std::vector<str::Pose<T>>&  str::ParticleFilter<T>::predict(std::vector<str::Pose<T>>& x_tm1,
		const std::pair<str::OdometryReading<T>, str::OdometryReading<T>>& odoReadingPair){
	this->valid_samples_ = 0;
	for(int m = 0; m < this->no_samples_; m++){
		if(motionModel_.Sample(odoReadingPair, x_tm1[m])){
			this->samplesTemp_[m] = x_tm1[m];
			this->valid_samples_++;
		}
	}
	return this->samplesTemp_;
}

template <typename T>
std::vector<str::Pose<T>>&  str::ParticleFilter<T>::update(const str::LaserReading<T>& z_t, int samplingType){

	// TODO: Sample from the measurement model
	// this->weights_[m] = measurementModel_.getProbFromBeamModel(this->samplesTemp_[m]);
	for(int m = 0; m < this->valid_samples_; m++){
		std::cout << z_t;
		std::cout << this->samplesTemp_[m];
		std::cout << "there" << std::endl;
		this->weights_[m] = this->measurementModel_.getProbability(z_t, this->samplesTemp_[m]);
		std::cout << "here" << std::endl;
	}

	// Perform the samlping
	if(samplingType == 0){
		this->importanceSampling();
	}
	else if(samplingType == 1){
		this->lowVarianceSampling();
	}
	else if(samplingType == 2){
		this->augmentedSampling();
	}
	return this->samples_;
}

template <typename T>
void str::ParticleFilter<T>::importanceSampling(){
	std::mt19937 gen(rd_());
	std::discrete_distribution<> d(this->weights_.begin(), this->weights_.begin() + this->valid_samples_);
	for(int m = 0; m < this->no_samples_; m++){
		int sample_id = d(gen);
		this->samples_[m] = this->samplesTemp_[sample_id];
	}
}

template <typename T>
void str::ParticleFilter<T>::lowVarianceSampling(){
	// Normalize the weights
	double n = std::accumulate(this->weights_.begin(), this->weights_.begin() + this->valid_samples_, 0);

	double r = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) / this->valid_samples_;
	double c = this->weights_[0] / n;
	int i = 0;

	for(int m = 0; m < this->no_samples_; m++){
		double U = r + 1.0 * m / this->valid_samples_;
		while(U > c){
			i++;
			c += this->weights_[i] / n;
		}
		this->samples_[m] = this->samplesTemp_[i];
	}
}

template <typename T>
void str::ParticleFilter<T>::augmentedSampling(){
	double w_avg = std::accumulate(this->weights_.begin(), this->weights_.begin() + this->valid_samples_, 0) / this->valid_samples_;
	this->w_slow += this->a_slow * (w_avg - this->w_slow);
	this->w_fast += this->a_fast * (w_avg - this->w_fast);

	std::mt19937 gen(rd_());
	std::discrete_distribution<> d(this->weights_.begin(), this->weights_.begin() + this->valid_samples_);

	for(int m = 0; m < this->no_samples_; m++){
		double r = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * std::max(0.0, (1.0 - w_fast / w_slow));
		if(r > .8){
			// Add Random Pose.
			// How to determine random pose
			double x = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*7999;
			double y = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*7999;
			double theta = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*6.283185;

			if(theta > 3.14159)
				theta -= 6.283185;

			int x_grid = std::round(x/10.0) >= 800 ? 799 : std::round(x/10.0);
			int y_grid = std::round(y/10.0) >= 800 ? 799 : std::round(y/10.0);

			this->samples_[m] = str::Pose<double>(x_grid, y_grid, theta);
		}
		int sample_id = d(gen);
		this->samples_[m] = this->samplesTemp_[sample_id];
	}
}
