#include "motion_model_odometry.h"

namespace str{

// template <typename T>
Motion_Model_Odom::Motion_Model_Odom() : 
	a1_(0.005),
	a2_(0.005),
	a3_(0.005),
	a4_(0.005)
{

}

// template <typename T>
Motion_Model_Odom::~Motion_Model_Odom()
{

}


bool Motion_Model_Odom::Sample(const std::pair<OdomRdg,OdomRdg>& control_u, str::Pose<double>& prev_pose)
{
	double x_bar, y_bar, theta_bar, xp_bar, yp_bar, thetap_bar;
	double d_rot_1, d_rot_2, d_trans, d_rot_1_hat, d_rot_2_hat, d_trans_hat;
	double xp, yp, thetap;

	x_bar = control_u.first.getX();
	y_bar = control_u.first.getY();
	theta_bar = control_u.first.getTheta();

	xp_bar = control_u.second.getX();
	yp_bar = control_u.second.getY();
	thetap_bar = control_u.second.getTheta();

	d_rot_1 = std::atan2(yp_bar-y_bar, xp_bar-x_bar) - theta_bar;
	d_trans = std::sqrt((x_bar-xp_bar)*(x_bar-xp_bar) + (y_bar-yp_bar)*(y_bar-yp_bar));
	d_rot_2 = thetap_bar - theta_bar - d_rot_1;

	// TODO
	if (d_trans <= 0.01 && d_rot_1+d_rot_2 <= 0.001){
		//std::cout << "no motion, should not sample" << std::endl;
	}
	
	d_rot_1_hat = d_rot_1 - this->SampleNormal(a1_*d_rot_1*d_rot_1 + a2_*d_trans*d_trans);
	d_trans_hat = d_trans - this->SampleNormal(a3_*d_trans*d_trans + a4_*d_rot_1*d_rot_1 + a4_*d_rot_2*d_rot_2);
	d_rot_2_hat = d_rot_2 - this->SampleNormal(a2_*d_trans*d_trans + a1_*d_rot_2*d_rot_2);

	xp = prev_pose.getX() + d_trans_hat*cos(prev_pose.getTheta()+d_rot_1_hat);
	yp = prev_pose.getY() + d_trans_hat*sin(prev_pose.getTheta()+d_rot_1_hat);
	thetap = prev_pose.getTheta() + d_rot_1_hat + d_rot_2_hat;

	// if(thetap > 6.283185)
	// 	thetap -= 6.283185;
	// if(thetap < -6.283185)
	// 	thetap += 6.283185
	if( std::fabs(thetap) > 6.283185)
		thetap = std::fmod(thetap, 6.283185);
	if(thetap > 3.14159)
		thetap -= 6.283185;
	if(thetap < -3.14159)
		thetap += 6.283185;

	// str::Pose<double> output_pose(xp, yp, thetap, 1);

	// return output_pose;

	bool x_check = xp > 0 && xp < 8000;
	bool y_check = yp > 0 && yp < 8000;

	if (x_check && y_check)
	{
		prev_pose.setX(xp);
		prev_pose.setY(yp);
		prev_pose.setTheta(thetap);
		return true;
	}else{
		return false;
	}

}


// str::Pose<double> Motion_Model_Odom::Sample(const std::pair<OdomRdg,OdomRdg>& control_u, const str::Pose<double>& prev_pose)
// {
// 	double x_bar, y_bar, theta_bar, xp_bar, yp_bar, thetap_bar;
// 	double d_rot_1, d_rot_2, d_trans, d_rot_1_hat, d_rot_2_hat, d_trans_hat;
// 	double xp, yp, thetap;

// 	x_bar = control_u.first.getX();
// 	y_bar = control_u.first.getY();
// 	theta_bar = control_u.first.getTheta();

// 	xp_bar = control_u.second.getX();
// 	yp_bar = control_u.second.getY();
// 	thetap_bar = control_u.second.getTheta();

// 	d_rot_1 = std::atan2(yp_bar-y_bar, xp_bar-x_bar) - theta_bar;
// 	d_trans = std::sqrt((x_bar-xp_bar)*(x_bar-xp_bar) + (y_bar-yp_bar)*(y_bar-yp_bar));
// 	d_rot_2 = thetap_bar - theta_bar - d_rot_1;

// 	// TODO
// 	if (d_trans <= 0.01 && d_rot_1+d_rot_2 <= 0.001){
// 		std::cout << "no motion, should not sample" << std::endl;
// 	}
	
// 	d_rot_1_hat = d_rot_1 - this->SampleNormal(a1_*d_rot_1*d_rot_1 + a2_*d_trans*d_trans);
// 	d_trans_hat = d_trans - this->SampleNormal(a3_*d_trans*d_trans + a4_*d_rot_1*d_rot_1 + a4_*d_rot_2*d_rot_2);
// 	d_rot_2_hat = d_rot_2 - this->SampleNormal(a2_*d_trans*d_trans + a1_*d_rot_2*d_rot_2);

// 	xp = prev_pose.getX() + d_trans_hat*cos(prev_pose.getTheta()+d_rot_1_hat);
// 	yp = prev_pose.getY() + d_trans_hat*sin(prev_pose.getTheta()+d_rot_1_hat);
// 	thetap = prev_pose.getTheta() + d_rot_1_hat + d_rot_2_hat;

// 	str::Pose<double> output_pose(xp, yp, thetap, 1);

// 	return output_pose;
// }

double Motion_Model_Odom::SampleNormal(const double& b_square)
{
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

	double sum = 0;

	for (int i = 0; i < 12; ++i){
		double rand_num = (dis(gen)-0.5) * 2 * std::sqrt(b_square); 
		sum += rand_num;
	}

	return 0.5*sum;
}

}
