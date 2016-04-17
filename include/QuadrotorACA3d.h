#ifndef QUADROTOR_ACA_3D_H
#define QUADROTOR_ACA_3D_H

#include "QuadrotorBase.h"
#include "Obstacle3d.h"
#include "linear_programming_3d.h"

#include <vector>
#include <Eigen/Dense>

class QuadrotorACA3d : public QuadrotorBase {
public:
	QuadrotorACA3d(void);
	QuadrotorACA3d(float time_horizon);

  void SetupNoise(void);  // Set the values of Z, M, and initialize Mtau
	void set_time_horizon(float time_horizon);
	Eigen::Vector3f sensing_noise(void);
	bool AvoidCollisions(const Input& desired_input,
											 std::vector<Obstacle3d>& obstacle_list, int flag);
  void SetVoltage(float);
  
  std::vector<Eigen::Vector3f> InitialDesiredTrajectory(void);
  std::vector<Eigen::Vector3f> FinalDesiredTrajectory(void);
  
private:
	float time_horizon_;  // Time horizon to look ahead for collisions in seconds
	Input delta_u_;  // The change in input calculated from the algorithm
	Input desired_u_;  // Desired input from user or some other high-level planner

	State g_; // Solved state used for feedforward trajectory estimate
	std::vector<State> gm_, gp_;  // Solved states with perturbation for Jacobian
	XXmat Mtau_;  // Position uncertainty at time tau
	Eigen::Matrix3f Z_;  // Obstacle sensing uncertainty

	std::vector<Plane> halfplanes_;  // Set of halfplanes representing collisions
	std::vector<Position> p_star_, p_star_init_;  // Desired positions from trajectory
	std::vector<Eigen::Matrix3f> J_;  // Set of jacobians along trajectory

	// Set the desired input from high level controller or user
	void set_desired_u(const Input& desired_u);

	// Set the change in input to zero for a given timestep
	void ResetDeltaU(void);

	// Solve for Pdot = AP + PA^T + M
	XXmat MotionVarianceDerivative(const XXmat& Mtau);

	// Update Mtau_ for one timestep
	void MotionVarianceIntegration(void);

	// Solves for projection of variance ellipsoid A onto vector b
	float VarianceProjection(const Eigen::Matrix3f& A, const Position& b);

	// Uncertainty bound methods
  Position PositionUncertaintyProjection(const Position& normal);
  Position SensingUncertaintyProjection(const Position& normal);
	float sigma(const Position& normal);

	// Solve for p_star_ and J_ for a given number of steps
	void Linearize(const State& x, const Input& u);

	// Solve the forward prediction of the trajectory with initial position of 0.
	void ForwardPrediction();

	// Return desired position of current input
	Position desired_position(void);

	// Return position at some point on trajectory
	Position trajectory_position(size_t time_step);

	// Create a halfplane for a given collision that is useable by the RVO lib
	void CreateHalfplane(const Eigen::Vector3f& pos_colliding,
											 const Eigen::Vector3f& normal);

	// Clear the halfplane list after the algorithm has completed
	void ClearHalfplanes(void);

	// Find which obstacles can potentially be colliding
	std::vector<int>
		FindPotentialCollidingPlanes(std::vector<Obstacle3d>& obstacle_list);

	// Check if a collision has occured between the trajectory and an obstacle
	bool IsThereACollision(std::vector<Obstacle3d>& obstacle_list,
												 std::vector<int>& index_list);

	// Run the linear programming to calcualte a change in input
	void CalculateDeltaU(void);
};

#endif
