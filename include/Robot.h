#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Dense>
#include "uncertainty_sampling.h"

#define _USE_MATH_DEFINES	
#include <cmath>

// X_DIM - Dimension of the state vector
// U_DIM - Dimension of the input vector
// Z_DIM - Dimension of the observation vector
// W_DIM - Dimension of the workspace
template <int X_DIM, int U_DIM, int Z_DIM, int W_DIM>
class Robot {

public:
  typedef Eigen::Matrix<float, X_DIM, 1> State;
  typedef Eigen::Matrix<float, U_DIM, 1> Input;
  typedef Eigen::Matrix<float, Z_DIM, 1> Observation;

  // Initialize matrices and other things
  virtual void Setup(void) = 0;

  void set_x(const State& x) {
    x_ = x;
  }  // set_x

  void set_input(const Input& u) {
		u_ = u;
	}
	
  // Update state estimate from input
  void ApplyInput() {
    x_ = RobotG(x_, u_);
		set_z();
		KalmanFilter(u_);
  }  // ApplyInput

  float radius() { return radius_; }
	
protected:
  typedef Eigen::Matrix<float, X_DIM, X_DIM> XXmat;
  typedef Eigen::Matrix<float, Z_DIM, Z_DIM> ZZmat;
  typedef Eigen::Matrix<float, X_DIM, Z_DIM> XZmat;
  typedef Eigen::Matrix<float, Z_DIM, X_DIM> ZXmat;
    
  State x_, x_hat_;
	Input u_;
	
  XXmat M_; // True process noise, could be different than Kalman
	ZZmat N_; // True observation noise, coudl be differen than Kalman
	
  Observation z_;  // Observation vector
  XXmat Q_;        // Process noise for Kalman
  ZZmat R_;        // Measurement noise for Kalman
  XXmat Pm_;       // a-priori covariance
  XXmat P_;        // posterior covariance
  ZXmat H_;        // map state to observation
    
  float dt_;
  float radius_;
    
  // Equations of motion, to be set in derived classes
  virtual State RobotF(const State& x, const Input& u) = 0;
    
  // Solution of state equations for time t
  State RobotG(const State& x, const Input& u) {
    State m = SampleGaussian(State::Zero(), M_);
      
    State k1 = RobotF(x, u) + m;
    State k2 = RobotF(x + 0.5*dt_*k1, u) + m;
    State k3 = RobotF(x + 0.5*dt_*k2, u) + m;
    State k4 = RobotF(x + dt_*k3, u) + m;
      
    State x_new = x + (dt_/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
      
    double theta = (x_new.segment(6,3)).norm();
    double eps = 0.0001;
      
    if (theta > 2.0*M_PI) {
      theta = fmod(theta,2.0*M_PI);
      if (theta < eps && theta > -eps) {
        x_new.segment(6,3) << 0,0,0;
      } else {
    	  x_new.segment(6,3) = theta*(x_new.segment(6,3)).normalized();
      }
    } 	
    return x_new;
  }  // RobotG

  // find jacobian of the state with respect to itself
  XXmat FindStateJacobian(const State& x, const Input& u) {
    XXmat A; // State jacobian 
    double j_step = 0.0009765625;
    State xP, xM, gP, gM;
    for (int i = 0; i < X_DIM; i++) {
      xP = x; xP[i] += j_step;
      xM = x; xM[i] -= j_step;
      gP = RobotG(xP,u);
      gM = RobotG(xM,u);
      A.col(i) = (gP-gM)/(2.0*j_step);
    }
    return A;
  }  // FindStateJacobian

	// Set the observation vector
	virtual void set_z(void) = 0;
	
  // Estimate the new state with a kalman filter for a given input
  void KalmanFilter(const Input& u) {
    State xM = RobotG(x_hat_,u);
    XXmat A = FindStateJacobian(x_hat_,u);
    Pm_ = A*P_*A.transpose() + Q_;
    XZmat K = Pm_*H_.transpose()*(H_*Pm_*H_.transpose() + R_).inverse();
    P_ = (XXmat::Identity() - K*H_)*Pm_;
    x_hat_ = xM + K*(z_ - H_*xM);
  }  // set_z
        

};

#endif
