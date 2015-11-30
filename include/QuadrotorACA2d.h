#ifndef QUADROTOR_ACA_2D_H
#define QUADROTOR_ACA_2D_H

#include "QuadrotorBase.h"
#include "Obstacle2d.h"
#include "linear_programming_2d.h"

#include <vector>
#include <Eigen/Dense>

class QuadrotorACA2d : public QuadrotorBase {
public:
	QuadrotorACA2d(void);
	QuadrotorACA2d(float time_horizon);

	void SetupNoise(void);
	void set_time_horizon(float time_horizon);
	Eigen::Vector2f sensing_noise(void);
	void AvoidCollisions(const Input& desired_input,
											 std::vector<Obstacle2d>& obstacle_list);
	std::vector<Eigen::Vector2f> InitialDesiredTrajectory(void);
	std::vector<Eigen::Vector2f> FinalDesiredTrajectory(void);

private:
	float time_horizon_;  // Time horizon to look ahead for collisions in seconds
	Input delta_u_;  // The change in input calculated from the algorithm
	Input desired_u_;  // Desired input from user or some other high-level planner

	State g_; // Solved state used for feedforward trajectory estimate
	std::vector<State> gm_, gp_;  // Solved states with perturbation for Jacobian
	XXmat Mtau_;  // Position uncertainty at time tau
	Eigen::Matrix2f Z_;  // Obstacle sensing uncertainty
	
	std::vector<Line> halfplanes_;  // Set of halfplanes representing collisions
	std::vector<Eigen::Vector2f> p_star_;  // Desired positions from trajectory
	std::vector<Eigen::Vector2f> p_star_initial_;
	std::vector<Eigen::Matrix2f> J_;  // Set of jacobians along trajectory
	
	// Set the desired input from high level controller or user
	void set_desired_u(const Input& desired_u);

	// Set the change in input to zero for a given timestep
	void ResetDeltaU(void);

	// Solve for Pdot = AP + PA^T + M
	XXmat MotionVarianceDerivative(const XXmat& Mtau);

	// Update Mtau_ for one timestep
	void MotionVarianceIntegration(void);

	// Solves for projection of variance ellipsoid A onto vector b
	float VarianceProjection(const Eigen::Matrix2f& A, const Eigen::Vector2f& b);

	// Uncertainty bound methods
	Eigen::Vector2f PositionUncertaintyProjection(const Eigen::Vector2f& normal);
	Eigen::Vector2f SensingUncertaintyProjection(const Eigen::Vector2f& normal);
	float sigma(const Eigen::Vector2f& normal);

	// Solve for p_star_ and J_ for a given number of steps
	void Linearize(const State& x, const Input& u);  

	// Solve the forward prediction of the trajectory with initial position of 0.
	void ForwardPrediction();
	
	// Return desired position of current input
	Eigen::Vector2f desired_position(void);

	// Return position at some point on trajectory
	Eigen::Vector2f trajectory_position(size_t time_step);

	// Return the distance to a colliding plane from a point
	float Projection(Obstacle2d& obstacle, int index);
	
	// Return the position on the trajectory with maximum penetration depth
	Eigen::Vector2f FindMaximumPenetration(Obstacle2d& obstacle, int starting_index);
	
	// Create a halfplane for a given collision that is useable by the RVO lib
	void CreateHalfplane(const Eigen::Vector2f& pos_colliding,
											 const Eigen::Vector2f& pos_desired,
											 const Eigen::Vector2f& normal);

	// Clear the halfplane list after the algorithm has completed
	void ClearHalfplanes(void);

	// Check if a collision has occured between the trajectory and an obstacle
	bool IsThereACollision(std::vector<Obstacle2d>& obstacle_list);

	// Run the linear programming to calcualte a change in input
	void CalculateDeltaU(void);
};
#endif
