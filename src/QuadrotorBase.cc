#include "QuadrotorBase.h"

QuadrotorBase::State QuadrotorBase::robotF(const QuadrotorBase::State& x0,
										 											 const QuadrotorBase::Input& u0) {
	// State = [pos vel orient angular_vel]
	// Input = [roll, pitch, yaw_rate, climb_rate]

	float kdrag = 0.15;
	float kd = 0.05;
	float kp1 = 0.5;
	float kp2 = 5.0;
	float kp3 = 0.5;
	float kp4 = 0.05;

	Eigen::Vector3f g(0,0,9.812);
	Eigen::Vector3f vel = x0.segment(3,3);
	Eigen::Vector3f r = x0.segment(6,3);
	Eigen::Vector3f w = x0.segment(9,3);
	float phi   = x0[6];
	float theta = x0[7];
	float psi   = x0[8];
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
	
	Eigen::Vector3f T(0,0,kp1*(u0[3]-vel[2]));
	Eigen::Vector3f tau;
	tau << kp2*(u0[0] - r[0]) - kd*w[0],
		     kp2*(u0[1] - r[1]) - kd*w[1],
      	 kp3*(u0[2] - w[2]);
	
	State x_dot;
	x_dot.segment(0,3) = vel;
	x_dot.segment(3,3) = -kdrag*vel + R*T;
	x_dot.segment(6,3) = w;
	x_dot.segment(9,3) = I.inverse()*(tau + w.cross(I*w));
	return x_dot;
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
	// TODO: update to be RPY
	Position r= x.segment(6,3);
	double theta = fmod(r.norm(), 2.0*M_PI);
	double eps = 0.0001;
	Eigen::Quaternionf q(cos(0.5*theta), r[0]*sin(0.5*theta),
											 r[1]*sin(0.5*theta), r[2]*sin(0.5*theta));
	return q;
}													 

Eigen::Quaternionf QuadrotorBase::est_orientation(void) {
	// TODO: update to be RPY
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
