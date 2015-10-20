#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Dense>

#define _USE_MATH_DEFINES	
#include <cmath>

// X_DIM - Dimension of the state vector
// U_DIM - Dimension of the input vector
// Z_DIM - Dimension of the observation vector
// W_DIM - Dimension of the workspace
template <int X_DIM, int U_DIM, int Z_DIM, int W_DIM>
class Robot {

protected:
  typedef Eigen::Matrix<float, X_DIM, X_DIM> XXmat;
  typedef Eigen::Matrix<float, Z_DIM, Z_DIM> ZZmat;
  typedef Eigen::Matrix<float, X_DIM, Z_DIM> XZmat;
  typedef Eigen::Matrix<float, Z_DIM, X_DIM> ZXmat;

public:
  typedef Eigen::Matrix<float, X_DIM, 1> State;
  typedef Eigen::Matrix<float, U_DIM, 1> Input;
  typedef Eigen::Matrix<float, Z_DIM, 1> Observation;
    
  State x, x_hat;
  XXmat M; // True process noise, could be different than Kalman

  Observation z;  // Observation vector
  XXmat Q;        // Process noise for Kalman
  ZZmat R;        // Measurement noise for Kalman
  XXmat Pm;       // a-priori covariance
  XXmat P;        // posterior covariance
  ZXmat H;        // map state to observation
  XZmat K;        // Kalman gain
    
  double dt;
  double radius;
    
  // Equations of motion, to be set in derived classes
  virtual State robotF(const State& x0, const Input& u0) = 0;
    
  // Solution of state equations for time t
  State robotG(const State& x0, const Input& u0,
	  						const XXmat& M = XXmat::Zero()) {
    State m = sampleGaussian(State::Zero(), M);
      
    State k1 = robotF(x0, u0) + m;
    State k2 = robotF(x0 + 0.5*dt*k1, u0) + m;
    State k3 = robotF(x0 + 0.5*dt*k2, u0) + m;
    State k4 = robotF(x0 + dt*k3, u0) + m;
      
    State x_new = x0 + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
      
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
  } // robot_g

  // find jacobian of the state with respect to itself
  XXmat findStateJacobian(const State& x0, const Input& u0) {
    XXmat A; // State jacobian 
    double j_step = 0.0009765625;
    State xP, xM, gP, gM;
    for (int i = 0; i < X_DIM; i++) {
      xP = x; xP[i] += j_step;
      xM = x; xM[i] -= j_step;
      gP = robotG(xP,u0);
      gM = robotG(xM,u0);
      A.col(i) = (gP-gM)/(2.0*j_step);
    }
    return A;
  }
     
  // Estimate the new state with a kalman filter for a given input
  State kalmanFilter(const Input& u0) {
    State xM = robotG(x_hat,u0);
    XXmat A = findStateJacobian(x_hat,u0);
    Pm = A*P*A.transpose() + Q;
    K = Pm*H.transpose()*(H*Pm*H.transpose() + R).inverse();
    P = (XXmat::Identity() - K*H)*Pm;
    return xM + K*(z-H*xM);
  }
        
public:
  // Initialize matrices and other things
  virtual void setup(void) = 0;
  
  // Sets observation
  void setObservation(Observation z_in) {
    z = z_in;
  }

  void setInitialState(const State& x0) {
    x = x0;
  }

  // Update state estimate from input
  void updateState(const Input& u0) {
    x = robot_g(x, u0, M);
  }
  
  void updateStateEstimate(const Input& u0) {
    x_hat = kalmanFilter(u0);
  }
      
  size_t getTimesteps(const double& t) {
    return (int) (t/dt);
  }
  
  double getRadius() {
    return radius;
  }
};

#endif
