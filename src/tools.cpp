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
    rmse<<0,0,0,0;
    
    if(estimations.size() != ground_truth.size() || estimations.size() == 0){
        std::cout<<"Invalid estimation or ground_truth data "<<std::endl;
        return rmse;
    }
    for(unsigned int i=0;i<estimations.size();++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
    }
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
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //check division by zero
    float c1=sqrt(pow(px, 2) + pow(py,2));
    float c2=pow(px,2)+ pow(py,2);
//    float c3=pow(c1,3);
    float c3=c1 * c2;
    
    if(fabs(c1) < 0.00001){
        std::cout<<"zero error"<<std::endl;
        return Hj;
    }
    Hj<<px/c1, py/c1,0,0,
    -py/c2, px/c2,0,0,
    py*(vx*py - vy*px)/c3, px*(vy*px - vx*py)/c3, px/c1, py/c1;
    return Hj;
}
