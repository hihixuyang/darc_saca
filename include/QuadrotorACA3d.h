#ifndef QUADROTOR_ACA_3D_H
#define QUADROTOR_ACA_3D_H

#include "QuadrotorACABase.h"
#include <vector>
#include <Eigen/Dense>

class QuadrotorACA3d : public QuadrotorACABase {
// Put linear programming and algorithm stuff here
public:
  void SetupNoise(void);  // Set the values of Z, M, and initialize Mtau
	
private:
	std::vector<Eigen::Vector3f> p_star_;
	std::vector<Eigen::Matrix3f> J_;

	// Solve for p_star_ and J_ for a given number of steps
	void Linearize(const State& x, const Input& u, const size_t steps);  

	// Solve the forward prediction of the trajectory with initial position of 0.
	void ForwardPrediction(const Input& u, const float& time_horizon);
	
	// Return desired position of current input
	Eigen::VectorXf desired_position(void);

	// Return position at some point on trajectory
	Eigen::VectorXf trajectory_position(size_t time_step);
};

#endif
