#ifndef QUADROTOR_BASE_H
#define QUADROTOR_BASE_H

#include "Robot.h"

#include <Eigen/Geometry>
// Robot<X_DIM,U_DIM,Z_DIM,P_DIM>
class QuadrotorBase : public Robot<12,4,8,3> {

public:
	typedef Eigen::Matrix<float, 3, 1> Position;

  void Setup(void);

	Position true_position(void);
	Position est_position(void);

	float true_speed(void);
	float est_speed(void);

  float true_roll(void);
  float est_roll(void);

 	float true_pitch(void);
	float est_pitch(void);

	float true_yaw(void);
	float est_yaw(void);

  void ApplyKalman(const Eigen::VectorXf& z);

protected:
  static const float mass_ = 1.42;  // kg
  static const float max_angle_ = 20.0 * M_PI / 180.0; // rad
  static const float max_yaw_rate_ = 60.0 * M_PI / 180.0;  // rad/s

  static const float kpx_ = 15.0;
  static const float kdx_ = 2.5;
  static const float kpy_ = 15.0;
  static const float kdy_ = 2.5;
  static const float kpz_ = 5.0;

 	State RobotF(const State& x0, const Input& u0);
	void set_z(void);

};

#endif
