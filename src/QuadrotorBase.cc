#include "QuadrotorBase.h"

QuadrotorBase::State QuadrotorBase::RobotF(const QuadrotorBase::State& x,
										 											 const QuadrotorBase::Input& u) {
	// State = [pos vel orient angular_vel]
	// Input = [roll, pitch, yaw_rate, climb_rate]

	float kdrag = 0.15;
	float kd = 0.05;
	float kp1 = 0.5;
	float kp2 = 5.0;
	float kp3 = 0.5;
	float kp4 = 0.05;

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
	I << 1, 0, 0,
		   0, 1, 0,
		   0, 0, 1;
	
	Eigen::Vector3f T(0,0,kp1*(u[3]-vel[2]));
	Eigen::Vector3f tau;
	tau << kp2*(u[0] - r[0]) - kd*w[0],
		     kp2*(u[1] - r[1]) - kd*w[1],
      	 kp3*(u[2] - w[2]);
	
	State x_dot;
	x_dot.segment(0,3) = vel;
	x_dot.segment(3,3) = -kdrag*vel + R*T;
	x_dot.segment(6,3) = w;
	x_dot.segment(9,3) = I.inverse()*(tau + w.cross(I*w));
	return x_dot + SampleGaussian(State::Zero(), M_);
}  // RobotF

void QuadrotorBase::set_z(void) {
	// Sense position and orientation
	z_.head(3) = x_.head(3);
	z_.segment(3,3) = x_.segment(6,3);
	z_ += SampleGaussian(Observation::Zero(), N_);
}  // set_z

void QuadrotorBase::Setup(void) {
	// True process noise (Does not have to be same as Kalman Q)
	M_ = XXmat::Zero();
  M_.block<3,3>(0,0) = 0.025*0.025*Eigen::Matrix3f::Identity();
	M_.block<3,3>(3,3) = 0.05*0.05*Eigen::Matrix3f::Identity();
	M_.block<3,3>(6,6) = 0.025*0.025*Eigen::Matrix3f::Identity();
	M_.block<3,3>(9,9) = 0.05*0.05*Eigen::Matrix3f::Identity();
	
	// True observation noise (Does not have to be same as Kalman R)
	N_ = ZZmat::Zero();
	N_.block<3,3>(0,0) = 0.005*0.005*Eigen::Matrix3f::Identity();
	N_.block<3,3>(3,3) = 0.005*0.005*Eigen::Matrix3f::Identity();
	
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
  radius_ = 0.2;
}  // Setup

QuadrotorBase::Position QuadrotorBase::true_position(void) {
	return x_.head(3);
}  // true_position

QuadrotorBase::Position QuadrotorBase::est_position(void) {
	return x_hat_.head(3);
}  // est_position
	
Eigen::Quaternionf QuadrotorBase::true_quaternion(void) {
	// TODO: update to be RPY
	Position r = x_.segment(6,3);
	double theta = fmod(r.norm(), 2.0*M_PI);
	double eps = 0.0001;
	Eigen::Quaternionf q(cos(0.5*theta), r[0]*sin(0.5*theta),
											 r[1]*sin(0.5*theta), r[2]*sin(0.5*theta));
	return q;
}  // true_quaternion								 				 

Eigen::Quaternionf QuadrotorBase::est_quaternion(void) {
	// TODO: update to be RPY
	Position r = x_hat_.segment(6,3);
	double theta = fmod(r.norm(), 2.0*M_PI);
	double eps = 0.0001;
	Eigen::Quaternionf q(cos(0.5*theta), r[0]*sin(0.5*theta),
											 r[1]*sin(0.5*theta), r[2]*sin(0.5*theta));
	return q;
}  // est_quaternion

float QuadrotorBase::true_speed(void) {
	return x_.segment(3,3).norm();
}  // true_speed

float QuadrotorBase::est_speed(void) {
	return x_hat_.segment(3,3).norm();
}  // est_speed
