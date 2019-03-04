#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if(estimations.size() == 0){
      cout << "Error: estimation vector is empty";
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()){
      cout << "Error: estimation vector is not the same size as ground_truth vector";
      return rmse;
  }
  // TODO: accumulate squared residuals
  VectorXd temp;
  for (unsigned int i=0; i < estimations.size(); ++i) {
    // ... your code here
    temp = (estimations[i] - ground_truth[i]);
    temp = temp.array()*temp.array();
    rmse += temp;
  }

  // TODO: calculate the mean
  rmse = rmse / estimations.size();
  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  // TODO: YOUR CODE HERE 
  // check division by zero
  if(px == 0 && py == 0){
      std::cout << "Error: division by zero" << std::endl;
      return Hj;
  }
  // compute the Jacobian matrix
  float den = px * px + py * py;
  Hj(0, 0) = px / sqrt(den);
  Hj(0, 1) = py / sqrt(den);
  Hj(0, 2) = 0;
  Hj(0, 3) = 0;
  Hj(1, 0) = -1 * py / den;
  Hj(1, 1) = px / den;
  Hj(1, 2) = 0;
  Hj(1, 3) = 0;
  Hj(2, 0) = Hj(0, 1) / den * (vx * py - vy * px);
  Hj(2, 1) = Hj(0, 0) / den * (vy * px - vx * py);
  Hj(2, 2) = Hj(0, 0);
  Hj(2, 3) = Hj(0, 1); 
  return Hj;
}
