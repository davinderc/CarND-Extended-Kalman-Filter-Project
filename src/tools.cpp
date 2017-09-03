#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check for division by zero
  if(estimations.size() == 0){
    cout << "Error: no estimations provided!" << endl;
    return rmse;
  }
  // Check for equal vector sizes
  if(estimations.size() != ground_truth.size()){
    cout << "Error: Different sizes for estimations/ground_truth vectors!" << endl;
    return rmse;
  }

  // Calc residuals, square and sum up in rmse
  for(int i=0; i < estimations.size(); ++i){
    VectorXd diff(estimations.size());
    diff = estimations[i] - ground_truth[i];
    VectorXd residuals = diff.array()*diff.array();
    rmse += residuals;
  }

  //calculate the mean
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  // State parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Some values that get used more than once
  float pxpy = pow(px,2.0) + pow(py,2.0);
  float pxpy2 = sqrt(pxpy);
  float pxpy32 = pow(pxpy2,3.0);
  float vxpy = vx*py - vy*px;
  float vypx = vy*px - vx*py;

  if(fabs(pxpy < 0.0001)){
    cout << "Error! There was an error with the error system!" << endl;
    cout << "Just kidding, division by zero" << endl;
    return Hj;
  }

  Hj << px/pxpy2, py/pxpy2, 0, 0,
        -1*py/pxpy, px/pxpy, 0, 0,
        py*vxpy/pxpy32, px*vypx/pxpy32, px/pxpy2, py/pxpy2;
  return Hj;
}
