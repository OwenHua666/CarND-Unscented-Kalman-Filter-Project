#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // Data Sanity Check
  if (estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Input size are incorrect."<<endl;
    cout << "Return 0 rmse"<<endl;
    return rmse;
  }

  //Summarize the rmse
  for (int i=0; i < estimations.size(); i++){

    VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();

    rmse += rmse;

  }

  rmse = rmse/estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;
}
