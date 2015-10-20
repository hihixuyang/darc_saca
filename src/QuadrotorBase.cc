#include "QuadrotorBase.h"

QuadrotorBase::State QuadrotorBase::robotF(const QuadrotorBase::State& x0,
										 											 const QuadrotorBase::Input& u0) {
  State x;
	return x;
}

void QuadrotorBase::setup(void) {
	// Process noise
  Q = XXmat::Zero();
  Q.block<3,3>(0,0) = 0.025*0.025*Eigen::Matrix3f::Identity();
  Q.block<3,3>(3,3) = 0.05*0.05*Eigen::Matrix3f::Identity();
  Q.block<3,3>(6,6) = 0.025*0.025*Eigen::Matrix3f::Identity();
  Q.block<3,3>(9,9) = 0.05*0.05*Eigen::Matrix3f::Identity();
    
  // Kalman Sensor noise
  R = ZZmat::Zero();
  R.block<3,3>(0,0) = 0.005*0.005*Eigen::Matrix3f::Identity();
  R.block<3,3>(3,3) = 0.005*0.005*Eigen::Matrix3f::Identity();
      
  // Observation using pixhawk IMU
  //H.block<3,3>(0,6) = Matrix3f::Identity();
  //H.block<3,3>(3,9) = Matrix3f::Identity();
  // Observation using position date (mocap-like)
	H.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
	H.block<3,3>(3,6) = Eigen::Matrix3f::Identity();
	
  // Kalman covariances
  Pm = XXmat::Zero();
  P = XXmat::Zero();
    
  // Robot/integration parameters
  dt = 0.02;
  radius = 0.2;
}

QuadrotorBase::Position QuadrotorBase::true_position(void) {
	return x.head(3);
}

QuadrotorBase::Position QuadrotorBase::est_position(void) {
	return x_hat.head(3);
}
	
Eigen::Quaternionf QuadrotorBase::true_orientation(void) {
	Position r = x.segment(6,3);
	double theta = fmod(r.norm(),2.0*M_PI);
	double eps = 0.0001;
	Eigen::Quaternionf q(cos(0.5*theta), r[0]*sin(0.5*theta),
											 r[1]*sin(0.5*theta), r[2]*sin(0.5*theta));
	return q;
}													 

Eigen::Quaternionf QuadrotorBase::est_orientation(void) {
	Position r = x_hat.segment(6,3);
	double theta = fmod(r.norm(), 2.0*M_PI);
	double eps = 0.0001;
	Eigen::Quaternionf q(cos(0.5*theta), r[0]*sin(0.5*theta),
											 r[1]*sin(0.5*theta), r[2]*sin(0.5*theta));
	return q;
}

float QuadrotorBase::true_speed(void) {
	return x.segment(3,3).norm();
}

float QuadrotorBase::est_speed(void) {
	return x_hat.segment(3,3).norm();
}
