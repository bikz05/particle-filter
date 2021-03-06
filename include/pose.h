#ifndef POSE_H
#define POSE_H

#include <iostream>

namespace str{
	template <typename T>
	class Pose;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const str::Pose<T>& pose);

namespace str{

template <typename T>
class Pose{
	private:
		T x_;
		T y_;
		T theta_;
		T weight_;
	public:
		Pose(){
			this->x_ = 0;
			this->y_ = 0;
			this->theta_ = 0;
			this->weight_ = 0;
		}

		Pose(const Pose<T>& that){
			this->x_ = that.x_;
			this->y_ = that.y_;
			this->theta_ = that.theta_;
			this->weight_ = that.weight_;
		}

		void operator=(const Pose<T>& that){
			this->x_ = that.x_;
			this->y_ = that.y_;
			this->theta_ = that.theta_;
			this->weight_ = that.weight_;
		}

		Pose(T x, T y, T theta, T weight=1):x_(x), y_(y), theta_(theta), weight_(weight){
		}

		void setX(T x){
			this->x_ = x;
		}

		void setY(T y){
			this->y_ = y;
		}

		void setTheta(T theta){
			this->theta_ = theta;
		}

		void setWeight(T weight){
			this->weight_ = weight;
		}

		T getX() const{
			return this->x_;
		}

		T getY() const{
			return this->y_;
		}

		T getTheta() const{
			return this->theta_;
		}

		T getWeight() const{
			return this->weight_;
		}

		template <typename U>
		friend std::ostream& ::operator<<(std::ostream& out, const str::Pose<U>& pose);
};

}

template <typename T>
std::ostream& operator<<(std::ostream& out, const str::Pose<T>& pose){
	out << "( x = " << pose.x_ << ", y = " << pose.y_ << ", theta = " << pose.theta_ << ", weight = " << pose.weight_ << ")" << std::endl;
	return out;
}

#endif //POSE_H
