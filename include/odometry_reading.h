#ifndef ODOMETRY_READING_H
#define ODOMETRY_READING_H

#include <iostream>
#include "../include/pose.h"


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
		OdometryReading(){
			this->pose_ = Pose<double>();
			this->timestamp_ = 0;
		}

		OdometryReading(T x, T y, T theta, T timestamp):pose_(x, y, theta), timestamp_(timestamp){
		}

		OdometryReading(Pose<T> pose, T timestamp):pose_(pose), timestamp_(timestamp){
		}

		inline bool operator==(const OdometryReading& rhs) const {return (this->getX()==rhs.getX() && this->getY()==rhs.getY() && this->getTheta()==rhs.getTheta());};

		void setX(T x){
			this->pose_.setX(x); 			// Sam changed
		}

		void setY(T y){
			this->pose_.setY(y); 			// Sam changed
		}

		void setTheta(T theta){
			this->pose_.setTheta(theta); 	// Sam changed
		}

		void setTimestamp(T timestamp){
			this->timestamp_ = timestamp;	// Sam changed
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
