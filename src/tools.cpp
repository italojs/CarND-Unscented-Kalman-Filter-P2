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

  if(estimations.size() == 0){
    cout << "ERROR - CalculateRMSE () - The estimations vector is empty" << endl;
    return rmse;
  }

  if(ground_truth.size() == 0){
    cout << "ERROR - CalculateRMSE () - The ground-truth vector is empty" << endl;
    return rmse;
  }

  unsigned int estimations_size = estimations.size();
  if(estimations_size != ground_truth.size()){
    cout << "ERROR - CalculateRMSE () - The ground-truth and estimations vectors must have the same size." << endl;
    return rmse;
  }

  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd delta = estimations[i] - ground_truth[i];
    delta = delta.array()*delta.array();
    rmse += delta;
  }

  rmse = rmse / estimations_size;
  rmse = rmse.array().sqrt();
  return rmse;
}
