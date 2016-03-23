#include "QuadrotorBase.h"
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <math.h>
QuadrotorBase::State QuadrotorBase::RobotF(const QuadrotorBase::State& x,
										 											 const QuadrotorBase::Input& u) {
	// Input = [r_x, r_y, v_z, v_w]
	Input u_sat = u;

	// State = [pos vel orient angular_vel]
  static const float kdrag = 0.15;
  static const float mass = 1.42;

  static const float max_climb_rate = 0.5;
  static const float Kp = 5.0;
  float t = Kp * (max_climb_rate * u_sat[2] - x[5]);

  float cx = cos(x[6]);
  float sx = sin(x[6]);
  float cy = cos(x[7]);
  float sy = sin(x[7]);
  float cz = cos(x[8]);
  float sz = sin(x[8]);
  Eigen::Vector3f T(t*(cx*sy*cz + sx*sz), t*(cx*sy*sz - sx*cz), t*(cx*cy));

  Eigen::Vector3f g(0,0,9.812);
  Eigen::Vector3f vel = x.segment(3,3);
  Eigen::Vector3f r = x.segment(6,3);
  Eigen::Vector3f w = x.segment(9,3);

	State x_dot;
	x_dot.segment(0,3) = vel;
	x_dot.segment(3,3) = T;// - g - kdrag*vel / mass;
	x_dot.segment(6,3) = w;
	x_dot[9]  = kpx_*(u_sat[0] - r[0]) - kdx_*w[0];
	x_dot[10] = kpy_*(u_sat[1] - r[1]) - kdy_*w[1];
	x_dot[11] = kpz_*(u_sat[3] - w[2]);
	return x_dot;
}  // RobotF

void QuadrotorBase::ApplyKalman(const Eigen::VectorXf& z) {
  for (int i = 0; i < z.rows(); ++i) {
    z_[i] = z(i);
  }
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
  M_.block<3,3>(0,0) = 0.1*0.1*Eigen::Matrix3f::Identity(); // P
	M_.block<3,3>(3,3) = 0.25*0.25*Eigen::Matrix3f::Identity(); //  V
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
  Q_.block<3,3>(0,0) = 0.6*0.6*Eigen::Matrix3f::Identity();  // px, py, pz
  Q_.block<3,3>(3,3) = 0.5*0.5*Eigen::Matrix3f::Identity();  // vx, vy, vz
  Q_.block<3,3>(6,6) = 0.25*0.25*Eigen::Matrix3f::Identity();  // rx, ry, rz
  Q_.block<3,3>(9,9) = 0.4*0.4*Eigen::Matrix3f::Identity();  // wx, wy, wz

  // Kalman observation noise
  R_ = ZZmat::Zero();
  R_.block<2,2>(0,0) = 0.1*0.1*Eigen::Matrix2f::Identity();  // rx, ry
  R_.block<3,3>(2,2) = 0.2*0.2*Eigen::Matrix3f::Identity();  // wx, wy, wz
  R_.block<3,3>(5,5) = 0.1*0.1*Eigen::Matrix3f::Identity();  // vx, vy, vz

  // Observation mapping
	H_ = ZXmat::Zero();
  H_.block<2,2>(0,6) = Eigen::Matrix2f::Identity(); // rx, ry
  H_.block<3,3>(2,9) = Eigen::Matrix3f::Identity(); // wx, wy, wz
  H_.block<3,3>(5,3) = Eigen::Matrix3f::Identity(); // vx, vy, vz

  // Kalman initial covariances
  Pm_ = XXmat::Zero();
  P_ = XXmat::Zero();

  // Robot/integration parameters
  dt_ = 0.02;
  radius_ = 0.3556;
  voltage_ = 11.1;
}  // Setup

QuadrotorBase::Position QuadrotorBase::true_position(void) {	return x_.head(3); }
QuadrotorBase::Position QuadrotorBase::est_position(void) { return x_hat_.head(3); }
float QuadrotorBase::true_speed(void) {	return x_.segment(3,3).norm(); }
float QuadrotorBase::est_speed(void) { return x_hat_.segment(3,3).norm(); }
float QuadrotorBase::true_roll(void) { return x_[6]; }
float QuadrotorBase::est_roll(void) { return x_hat_[6]; }
float QuadrotorBase::true_pitch(void) { return x_[7]; }
float QuadrotorBase::est_pitch(void) { return x_hat_[7]; }
float QuadrotorBase::true_yaw(void) {	return x_[8]; }
float QuadrotorBase::est_yaw(void) { return x_hat_[8]; }
