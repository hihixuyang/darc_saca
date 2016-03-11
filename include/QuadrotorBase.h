#ifndef QUADROTOR_BASE_H
#define QUADROTOR_BASE_H

#include "Robot.h"

#include <Eigen/Geometry>

class QuadrotorBase : public Robot<12,4,6,3> {

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
	
protected:
  static const float mass_ = 1.42;
  static const float max_angle_ = 15.0* M_PI / 180.0;
  static const float max_yaw_rate_ = 60.0 * M_PI / 180.0;
  static const float kpx_ = 145;
  static const float kdx_ = 5;
  static const float kpy_ = 145;
  static const float kdy_ = 5;
  static const float kpz_ = 5;
	State RobotF(const State& x0, const Input& u0);
	void set_z(void);
};

#endif
