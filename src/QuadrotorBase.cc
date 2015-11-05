#include "QuadrotorBase.h"
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>

QuadrotorBase::State QuadrotorBase::RobotF(const QuadrotorBase::State& x,
										 											 const QuadrotorBase::Input& u) {
	// State = [pos vel orient angular_vel]
	// Input = [roll, pitch, yaw_rate, climb_rate]
	Eigen::Vector3f g(0,0,9.812);
	Eigen::Vector3f vel = x.segment(3,3);
	Eigen::Vector3f r = x.segment(6,3);
	Eigen::Vector3f w = x.segment(9,3);
	float phi   = x[6];
	float theta = x[7];
	float psi   = x[8];
	float cphi = cos(phi);
	float sphi = sin(phi);
	float ctheta = cos(theta);
	float stheta = sin(theta);
	float cpsi = cos(psi);
	float spsi = sin(psi);
	
	Eigen::Matrix3f R;
	R << cpsi*ctheta-sphi*spsi*stheta, -cphi*spsi, cpsi*stheta+ctheta*sphi*spsi,
		ctheta*spsi+cpsi*sphi*stheta, cphi*cpsi, spsi*stheta-cpsi*ctheta*sphi,
		-cphi*stheta, sphi, cphi*ctheta;
	
	Eigen::Matrix3f I;
	I << 0.6, 0,   0,
		   0,   0.6, 0,
		   0,   0,   0.9;

	float max_climb_rate = 0.5;
	float kp1 = 10.0;
	Eigen::Vector3f T(0,
										0,
										g.norm()/(cphi*ctheta) + kp1*(max_climb_rate*u[2]-vel[2]));
	
	float max_angle = 30.0f*M_PI/180.0f;
	float max_yaw_rate = 90.0f*M_PI/180.0f;
	float kp2 = 15.1;
	float kd  = 2.5;
	float kp3 = 5.0;
	Eigen::Vector3f tau = Eigen::Vector3f::Zero();
	// Control about roll, pitch, yaw rate
	tau[0] = kp2*(-max_angle*u[1] - phi) - kd*w[0];
	tau[1] = kp2*(max_angle*u[0] - theta) - kd*w[1];
  tau[2] = kp3*(max_yaw_rate*u[3] - w[2]);
	
	float kdrag = 1.25;
	State x_dot;
	x_dot.segment(0,3) = vel;
	x_dot.segment(3,3) = R*T - g - kdrag*vel;
	x_dot.segment(6,3) = w;
	x_dot.segment(9,3) = I.inverse()*(tau + w.cross(I*w));
	return x_dot;
}  // RobotF

void QuadrotorBase::set_z(void) {
	// Using pixhawk to read angle and angular rates
	//z_ = x_.segment(6,6);
	// Sense position and orientation
	z_.head(3) = x_.head(3);
	z_.segment(3,3) = x_.segment(6,3);
	z_ += SampleGaussian(Observation::Zero(), N_);
}  // set_z

void QuadrotorBase::Setup(void) {
	x_hat_ = State::Zero();
	// True process noise (Does not have to be same as Kalman Q)
	M_ = XXmat::Zero();
	/*
  M_.block<3,3>(0,0) = 0.025*0.025*Eigen::Matrix3f::Identity();
	M_.block<3,3>(3,3) = 0.05*0.05*Eigen::Matrix3f::Identity();
	M_.block<3,3>(6,6) = 0.025*0.025*Eigen::Matrix3f::Identity();
	M_.block<3,3>(9,9) = 0.05*0.05*Eigen::Matrix3f::Identity();
	*/
	// True observation noise (Does not have to be same as Kalman R)
	N_ = ZZmat::Zero();
	/*
	N_.block<3,3>(0,0) = 0.005*0.005*Eigen::Matrix3f::Identity();
	N_.block<3,3>(3,3) = 0.005*0.005*Eigen::Matrix3f::Identity();
	*/
	
	// Kalman process noise
  Q_ = XXmat::Zero();
  Q_.block<3,3>(0,0) = 0.025*0.025*Eigen::Matrix3f::Identity();
  Q_.block<3,3>(3,3) = 0.05*0.05*Eigen::Matrix3f::Identity();
  Q_.block<3,3>(6,6) = 0.025*0.025*Eigen::Matrix3f::Identity();
  Q_.block<3,3>(9,9) = 0.05*0.05*Eigen::Matrix3f::Identity();

  // Kalman observation noise
  R_ = ZZmat::Zero();
  R_.block<3,3>(0,0) = 0.005*0.005*Eigen::Matrix3f::Identity();
  R_.block<3,3>(3,3) = 0.005*0.005*Eigen::Matrix3f::Identity();
	
  // Observation mapping
	H_ = ZXmat::Zero();
	// Using pixhawk to read angles and angular rates
  //H.block<3,3>(0,6) = Matrix3f::Identity();
  //H.block<3,3>(3,9) = Matrix3f::Identity();
  // Observation using position date (mocap-like)
	H_.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
	H_.block<3,3>(3,6) = Eigen::Matrix3f::Identity();
	
  // Kalman initial covariances
  Pm_ = XXmat::Zero();
  P_ = XXmat::Zero();
    
  // Robot/integration parameters
  dt_ = 0.02;
  radius_ = 0.282;
}  // Setup

QuadrotorBase::Position QuadrotorBase::true_position(void) {
	return x_.head(3);
}  // true_position

QuadrotorBase::Position QuadrotorBase::est_position(void) {
  return x_hat_.head(3);
}  // est_position
	
Eigen::Quaternionf QuadrotorBase::true_quaternion(void) {
	float phi   = x_[6];
	float theta = x_[7];
	float psi   = x_[8];
	
	float cphi   = cos(0.5*phi);
	float sphi   = sin(0.5*phi);
	float ctheta = cos(0.5*theta);
	float stheta = sin(0.5*theta);
	float cpsi   = cos(0.5*psi);
	float spsi   = sin(0.5*psi);

	float qw = cphi*ctheta*cpsi + sphi*stheta*spsi;
	float qx = sphi*ctheta*cpsi - cphi*stheta*spsi;
	float qy = cphi*stheta*cpsi + sphi*ctheta*spsi;
	float qz = cphi*ctheta*spsi - sphi*stheta*cpsi;
	
	Eigen::Quaternionf q(qw, qx, qy, qz);
	return q;
}  // true_quaternion								 				 

Eigen::Quaternionf QuadrotorBase::est_quaternion(void) {
	float phi   = x_hat_[6];
	float theta = x_hat_[7];
	float psi   = x_hat_[8];
	
	float cphi   = cos(0.5*phi);
	float sphi   = sin(0.5*phi);
	float ctheta = cos(0.5*theta);
	float stheta = sin(0.5*theta);
	float cpsi   = cos(0.5*psi);
	float spsi   = sin(0.5*psi);

	float qw = cphi*ctheta*cpsi + sphi*stheta*spsi;
	float qx = sphi*stheta*cpsi - cphi*stheta*spsi;
	float qy = cphi*stheta*cpsi + sphi*ctheta*spsi;
	float qz = cphi*ctheta*spsi - sphi*stheta*cpsi;
	
	Eigen::Quaternionf q(qw, qx, qy, qz);
	return q;
}  // est_quaternion

float QuadrotorBase::true_speed(void) {
	return x_.segment(3,3).norm();
}  // true_speed

float QuadrotorBase::est_speed(void) {
	return x_hat_.segment(3,3).norm();
}  // est_speed
