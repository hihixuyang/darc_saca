#ifndef QUADROTOR_ACA_BASE_H
#define QUADROTOR_ACA_BASE_H

#include "QuadrotorBase.h"
#include <vector>
#include <Eigen/Dense>

class QuadrotorACABase : public QuadrotorBase {
// Put linear programming and algorithm stuff here
public:
  virtual void SetupNoise(void) = 0;  // Set the values of Z, M, and initialize Mtau
	
protected:
	struct halfplane {
		Eigen::VectorXf pos_colliding;
		Eigen::VectorXf normal;
	};

	Input delta_u_;  // Change in input calculated from algorithm
	
	XXmat A_;  // Used to store State jacobian
	XXmat M_, Mtau_;
	Eigen::MatrixXf Z_;
	
	std::vector<halfplane> halfplanes_;
	
  // Solve for Pdot = AP+PA^T + M
	XXmat MotionVarianceDerivative(const XXmat& Mtau);

	// Update Mtau_ for one timestep
	void MotionVarianceIntegration(void);

	// Solves for projection of variance ellipsoid A onto a vector b
	float VarianceProjection(const Eigen::MatrixXf& A,
													 const Eigen::VectorXf& b);
	
	// Uncertainty bound methods
	Eigen::VectorXf PositionUncertaintyProjection(const Eigen::VectorXf& normal);
	Eigen::VectorXf SensingUncertaintyProjection(const Eigen::VectorXf& normal);
	float sigma(const Eigen::VectorXf& normal);

	// Save a constraint if a collision is found
	void CreateHalfplane(const Eigen::VectorXf pos_colliding,
											 const Eigen::VectorXf normal);

	// Clear saved constraints for next timestep of algorithm
	void ClearHalfplanes(void);

	// Solve for p_star_ and J_ for a given number of steps
	virtual void Linearize(const State& x, const Input& u, const size_t steps) = 0;

	// Solve the forward prediction of the trajectory with initial position of 0.
	virtual void ForwardPrediction(const Input& u, const float& time_horizon) = 0;

	virtual Eigen::VectorXf desired_position() = 0;
	virtual Eigen::VectorXf trajectory_position(size_t time_step) = 0;
};

#endif
