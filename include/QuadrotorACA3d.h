#ifndef QUADROTOR_ACA_3D_H
#define QUADROTOR_ACA_3D_H

#include "QuadrotorACABase.h"
#include "Obstacle3d.h"
#include "linear_programming_3d.h"

#include <vector>
#include <Eigen/Dense>

class QuadrotorACA3d : public QuadrotorACABase {
// Put linear programming and algorithm stuff here
public:
	QuadrotorACA3d(void);
	QuadrotorACA3d(float time_horizon);
	
  void SetupNoise(void);  // Set the values of Z, M, and initialize Mtau
	void AvoidCollisions(const Input& desired_input,
											 std::vector<Obstacle3d>& obstacle_list);
	
private:
	std::vector<Plane> halfplanes_;
	std::vector<Eigen::Vector3f> p_star_;
	std::vector<Eigen::Matrix3f> J_;

	// Solve for p_star_ and J_ for a given number of steps
	void Linearize(const State& x, const Input& u);  

	// Solve the forward prediction of the trajectory with initial position of 0.
	void ForwardPrediction();
	
	// Return desired position of current input
	Eigen::VectorXf desired_position(void);

	// Return position at some point on trajectory
	Eigen::VectorXf trajectory_position(size_t time_step);

	void CreateHalfplane(const Eigen::VectorXf pos_colliding,
											 const Eigen::VectorXf normal);
	
	void ClearHalfplanes(void);
	
	// Find which obstacles can potentially be colliding
	std::vector<int>
		FindPotentialCollidingPlanes(std::vector<Obstacle3d>& obstacle_list);

	bool CheckForCollision(std::vector<Obstacle3d>& obstacle_list,
												 std::vector<int>& index_list);

	void CalculateDeltaU(void);
};

#endif
