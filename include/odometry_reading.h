#ifndef ODOMETRY_READING_H
#define ODOMETRY_READING_H

#include <iostream>

namespace str{
	template <typename T>
	class OdometryReading;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const str::OdometryReading<T>& odoRdg);

namespace str{

template <typename T>
class OdometryReading{
	private:
		Pose<T> pose_;
		T timestamp_;
	public:
		OdometryReading(T x, T y, T theta, T timestamp):pose_(x, y, theta), timestamp_(timestamp){
		}

		OdometryReading(Pose<T> pose, T timestamp):pose_(pose), timestamp_(timestamp){
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

		T getX() const{
			return this->pose_.getX();		// Sam changed
		}

		T getY() const{
			return this->pose_.getY();		// Sam changed
		}

		T getTheta() const{
			return this->pose_.getTheta();	// Sam changed
		}

		T getTimestamp() const{
			return this->timestamp_;		// Sam changed
		}

		template <typename U>
		friend std::ostream& ::operator<<(std::ostream& out, const str::OdometryReading<U>& odoRdg);
};

}

template <typename T>
std::ostream& operator<<(std::ostream& out, const str::OdometryReading<T>& odoRdg){
	out << "(OR) [TS = " << odoRdg.timestamp_ << " ] -> ";
	out << odoRdg.pose_;
    return out;
}

#endif //ODOMETRY_READING_H
