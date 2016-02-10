#ifndef QUADROTOR_BASE_H
#define QUADROTOR_BASE_H

#include "Robot.h"

#include <Eigen/Geometry>

class QuadrotorBase : public Robot<12,4,8,3> {

public:
	typedef Eigen::Matrix<float, 3, 1> Position;

  void Setup(void);
 
	Position true_position(void);
	Position est_position(void);
	
	Eigen::Quaternionf true_quaternion(void);
	Eigen::Quaternionf est_quaternion(void);

	float true_speed(void);
	float est_speed(void);

	float true_roll(void);
	float est_roll(void);

	float true_pitch(void);
	float est_pitch(void);

	float true_yaw(void);
	float est_yaw(void);

  void ApplyKalman(const float&, const float&,  // rx, ry
                   const float&, const float&, const float&,  // wx, wy, wz
                   const float&, const float&, const float&);  // vx, vy, vz

protected:
  static const float mass_ = 1.42;  // kg
  static const float gravity_ = 9.812;  // m/s^2

  static const float max_climb_rate_ = 1.0;
  static const float max_angle_ = 20.0*M_PI / 180.0;
  static const float max_yaw_rate_ = 45.0*M_PI / 180.0;

  static const float kp1_ = 10.0;
  static const float kp2_ = 15.0;
  static const float kd_ = 2.5;
  static const float kdrag_ = 0.75;
  static const float kp3_ = 5.0;

 	State RobotF(const State& x0, const Input& u0);
	void set_z(void);

};

#endif
