#ifndef LASER_READING_H
#define LASER_READING_H

#include <iostream>
#include <vector>

namespace str{
	template <typename T>
	class LaserReading;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const str::LaserReading<T>& laserRdg);

namespace str{

template <typename T>
class LaserReading{
	private:
		Pose<T> pose_;
		T timestamp_;
		std::vector<T> ranges_;
	public:
		LaserReading(T x, T y, T theta, std::vector<T> ranges, T timestamp):pose_(x, y, theta), ranges_(ranges), timestamp_(timestamp){
		}

		LaserReading(Pose<T> pose, std::vector<T> ranges, T timestamp):pose_(pose), ranges_(ranges), timestamp_(timestamp){
		}

		void setX(T x){
			this->pose_.x_ = x;
		}

		void setY(T y){
			this->pose_.y_ = y;
		}

		void setTheta(T theta){
			this->pose_.theta_ = theta;
		}

		void setTimestamp(T timestamp){
			this->pose_.timestamp_ = timestamp;
		}

		void setRanges(std::vector<T> ranges){
			this->ranges_ = ranges;
		}

		T getX() const{
			return this->pose_.x_;
		}

		T getY() const{
			return this->pose_.y_;
		}

		T getTheta() const{
			return this->pose_.theta_;
		}

		T getTimestamp() const{
			return this->pose_.timestamp_;
		}

		T getRanges() const{
			return this->ranges_;
		}

		template <typename U>
		friend std::ostream& ::operator<<(std::ostream& out, const str::LaserReading<U>& laserRdg);
};

}

template <typename T>
std::ostream& operator<<(std::ostream& out, const str::LaserReading<T>& laserRdg){
	out << "(LR) [TS = " << laserRdg.timestamp_ << " ] -> ";
	out << laserRdg.pose_ << std::endl;
	int i = laserRdg.ranges_.size();
	for(auto range_i: laserRdg.ranges_){
		if(++i == 20)
			out << std::endl;
		out << range_i << "\t";
	}
	out << std::endl;
    return out;
}

#endif //LASER_READING_H
