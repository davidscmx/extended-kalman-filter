#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  
  x_ = F_ * x_;  
  MatrixXd Ft = F_.transpose();  
  P_ = F_ * P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  

  const double PI  =3.141592653589793238463;
  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  
  long x_size = x_.size();
  
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  std::cout << "begginnign of update " << std::endl;

  //VectorXd z_pred = H_ * x_;
  const double PI  =3.141592653589793238463;
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float phi = atan2(py,px);

  if (z(1)>PI){
     std::cout << "z greater than PI " << std::endl; 
     std::cout << "oroginal phi  " << phi  << std::endl; 
          
     if (phi<0){
      std::cout << "1" << std::endl;      
        phi += 2*PI;

     }
  }

  float rho_dot = (px*vx + py*vy) / rho;

  VectorXd z_pred = VectorXd(3);  
  z_pred << rho,phi,rho_dot;

  std::cout << "size of z: " << z.rows() << " " << z.cols() << std::endl;
  std::cout << "z measured: " << z(0) << " " << z(1) << " " <<  z(2)   <<std::endl;
  std::cout << "z pred : " << z_pred(0) << " " << z_pred(1) << " " <<  z_pred(2)   <<std::endl;

  VectorXd y = z - z_pred;

  std::cout << "2 " << std::endl;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  
  long x_size = x_.size();
  
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  

}
