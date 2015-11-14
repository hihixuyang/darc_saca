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

protected:
	State RobotF(const State& x0, const Input& u0);
	void set_z(void);
};

#endif
