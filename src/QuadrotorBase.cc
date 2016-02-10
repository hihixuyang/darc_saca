
#include "QuadrotorBase.h"
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>

QuadrotorBase::State QuadrotorBase::RobotF(const QuadrotorBase::State& x,
										 											 const QuadrotorBase::Input& u) {
	// State = [pos vel orient angular_vel]
	// Input = [roll, pitch, yaw_rate, climb_rate]
	Eigen::Vector3f g(0,0,9.812); //gravity_);
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

	// Control about roll, pitch, yaw rate
	Input u_sat = u;
	for (int u_index = 0; u_index < 4; ++u_index) {
		if (u[u_index] < -1.0) {
			u_sat[u_index] = -1.0;
		} else if(u[u_index] > 1.0) {
			u_sat[u_index] = 1.0;
		}
	}

	Eigen::Vector3f T(0,
										0,
										(mass_*gravity_ + kp1_*(max_climb_rate_*u[2]-vel[2])) / (cphi * ctheta));

  Eigen::Vector3f w_dot;
	w_dot[0] = kp2_*(-max_angle_*u_sat[1] - phi) - kd_*w[0];
	w_dot[1] = kp2_*( max_angle_*u_sat[0] - theta) - kd_*w[1];
  w_dot[2] = kp3_*( max_yaw_rate_*u_sat[3] - w[2]);

	State x_dot;
	float kdrag = 0.75;
	x_dot.segment(0,3) = vel;
	x_dot.segment(3,3) = R*T - g - kdrag*vel;
	x_dot.segment(6,3) = w;
	x_dot.segment(9,3) = w_dot;
	return x_dot;
}  // RobotF

void QuadrotorBase::ApplyKalman(const float& rx, const float& ry,
                                const float& wx, const float& wy, const float& wz,
                                const float& vx, const float& vy, const float& vz) {
  z_[0] = rx;
  z_[1] = ry;
  z_[2] = wx;
  z_[3] = wy;
  z_[4] = wz;
  z_[5] = vx;
  z_[6] = vy;
  z_[7] = vz;
  KalmanFilter(u_);
}  // ApplyKalman

void QuadrotorBase::set_z(void) {
	// Sense position and orientation
	z_.head(3) = x_.head(3);
	z_.segment(3,3) = x_.segment(6,3);
	z_ += SampleGaussian(Observation::Zero(), N_);
}  // set_z

void QuadrotorBase::Setup(void) {
	x_hat_ = State::Zero();
	// True process noise (Does not have to be same as Kalman Q)
	M_ = XXmat::Zero();
  M_.block<3,3>(0,0) = 0.1*0.1*Eigen::Matrix3f::Identity();
	M_.block<3,3>(3,3) = 0.25*0.25*Eigen::Matrix3f::Identity();
	M_.block<3,3>(6,6) = 0.1*0.1*Eigen::Matrix3f::Identity();
	M_.block<3,3>(9,9) = 0.25*0.25*Eigen::Matrix3f::Identity();
	M_ = XXmat::Zero();

	// True observation noise (Does not have to be same as Kalman R)
	N_ = ZZmat::Zero();
	//N_.block<3,3>(0,0) = 0.0075*0.0075*Eigen::Matrix3f::Identity();
	//N_.block<3,3>(3,3) = 0.0075*0.0075*Eigen::Matrix3f::Identity();
	N_ = ZZmat::Zero();

	// Kalman process noise
  Q_ = XXmat::Zero();
  Q_.block<3,3>(0,0) = 0.1*0.1*Eigen::Matrix3f::Identity();
  Q_.block<3,3>(3,3) = 0.25*0.5*Eigen::Matrix3f::Identity();
  Q_.block<3,3>(6,6) = 0.25*0.25*Eigen::Matrix3f::Identity();
  Q_.block<3,3>(9,9) = 0.5*0.5*Eigen::Matrix3f::Identity();

  // Kalman observation noise
  R_ = ZZmat::Zero();
  //R_.block<3,3>(0,0) = 0.01*0.01*Eigen::Matrix3f::Identity();
  //R_.block<3,3>(3,3) = 0.01*0.01*Eigen::Matrix3f::Identity();
  R_.block<2,2>(0,0) = 0.025*0.025*Eigen::Matrix2f::Identity();
  R_.block<3,3>(2,2) = 0.05*0.05*Eigen::Matrix3f::Identity();
  R_.block<3,3>(5,5) = 0.05*0.05*Eigen::Matrix3f::Identity();

  // Observation mapping
	H_ = ZXmat::Zero();
	// Using pixhawk to read angles and angular rates
  H_.block<2,2>(0,6) = Eigen::Matrix2f::Identity();
  H_.block<3,3>(2,9) = Eigen::Matrix3f::Identity();
  H_.block<3,3>(5,3) = Eigen::Matrix3f::Identity();
  // Observation using position date (mocap-like)
	//H_.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
	//H_.block<3,3>(3,6) = Eigen::Matrix3f::Identity();

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

float QuadrotorBase::true_roll(void) {
	return x_[6];
}  // true_roll

float QuadrotorBase::est_roll(void) {
	return x_hat_[6];
}  // est_roll

float QuadrotorBase::true_pitch(void) {
	return x_[7];
}  // true_pitch

float QuadrotorBase::est_pitch(void) {
	return x_hat_[7];
}  // est_pitch

float QuadrotorBase::true_yaw(void) {
	return x_[8];
}  // true_yaw

float QuadrotorBase::est_yaw(void) {
	return x_hat_[8];
}  // est_yaw
