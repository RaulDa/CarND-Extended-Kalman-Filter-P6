#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
	    cout << "Error: estimations size zero or estimations and ground_truth different" << endl;
	}
    else{
    	//accumulate squared residuals
	    for(int i=0; i < estimations.size(); ++i){

            VectorXd res = estimations[i]-ground_truth[i];

            res = res.array()*res.array();

            rmse += res;

	    }

	    //calculate the mean
	    rmse = rmse / estimations.size();

	    //calculate the squared root
	    rmse = rmse.array().sqrt();
    }
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	// matrix initialization
    MatrixXd Hj = MatrixXd::Zero(3, 4);

	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// do repeated calculations once and store results
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = c1*c2;

   	// check division by values near from zero
   	if(fabs(c1)<0.0001){
   	    cout << "Error. Division by zero" << endl;
   	}
   	else{
   		// calculate matrix values
   	    Hj(0,0) = px/c2;
   	    Hj(0,1) = py/c2;
   	    Hj(1,0) = -py/c1;
   	    Hj(1,1) = px/c1;
   	    Hj(2,0) = py*(vx*py-vy*px)/c3;
   	    Hj(2,1) = px*(vy*px-vx*py)/c3;
   	    Hj(2,2) = px/c2;
   	    Hj(2,3) = py/c2;
   	}
   	// return result
   	return Hj;
}

VectorXd Tools::CartToPolar(const VectorXd& x_state) {

	// define and initialize output vector
	VectorXd h(3);
	h << 0,0,0;

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// do repeated calculations once and store results
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);

   	//check division by zero
   	if(fabs(c1)<0.0001){
   	    cout << "Error. Division by zero" << endl;
   	}
   	else{
   		// calculate polar coordinates
   	    h(0) = c2;
   	    h(1) = atan2(py,px);
   	    h(2) = (px*vx+py*vy)/c2;
   	}

   	// return result
   	return h;
}

VectorXd Tools::AdjustAngle(const VectorXd& y) {

	// define output vector
	VectorXd y_(3);

	// initialize to received difference between measurement and prediction
	y_(0) = y(0);
	y_(1) = y(1);
	y_(2) = y(2);

	// adjust angle if not between used range
	if(-M_PI <= y_(1) && y_(1) <= M_PI)
	{

	}
	else if(y_(1) > M_PI){
		while(y_(1) > M_PI){
			y_(1) = y_(1) - 2 * M_PI;
		}
	}
	else{
		while(-M_PI > y_(1)){
			y_(1) = y_(1) + 2 * M_PI;
		}
	}

	// return result
   	return y_;
}
