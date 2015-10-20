#ifndef QUADROTOR_BASE_H
#define QUADROTOR_BASE_H

#include "Robot.h"

#include <Eigen/Geometry>

class QuadrotorBase : public Robot<12,4,6,3> {
protected:
	State robotF(const State& x0, const Input& u0);

public:
  typedef Eigen::Matrix<float, 3, 1> Position;
	
  // Initialize matrices and other things
  void setup(void);

	Position true_position(void);
	Position est_position(void);
	
	Eigen::Quaternionf true_orientation(void);
	Eigen::Quaternionf est_orientation(void);

	float true_speed(void);
	float est_speed(void);
};

#endif
