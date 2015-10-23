#include "QuadrotorACA3d.h"
#include "linear_programming_3d.h"

void QuadrotorACA3d::SetupNoise(void) {
	Z_ = Eigen::Matrix3f::Zero();
	float scale = 1.0;
	Z_.block<3,3>(0,0) = scale*scale*0.1*0.1*Eigen::Matrix3f::Identity();

	M_ = XXmat::Zero();
	double vv = 0.2;
	M_.block<3,3>(0,0) = scale*scale*vv*vv*Eigen::Matrix3f::Identity();
	double aa = 0.1;
	M_.block<3,3>(3,3) = scale*scale*aa*aa*Eigen::Matrix3f::Identity();
	double rr = 0.05;
	M_.block<3,3>(6,6) = scale*scale*rr*rr*Eigen::Matrix3f::Identity();
	double ww = 0.025;
	M_.block<3,3>(9,9) = scale*scale*ww*ww*Eigen::Matrix3f::Identity();
}  // SetupNoise

void QuadrotorACA3d::Linearize(const State& x, const Input& u,
															 const size_t max_steps) {
	float j_step = 0.0009765625;
	p_star_.clear();
	J_.clear();
	p_star_.push_back(x.head(3));
	Mtau_ = XXmat::Zero();

	static State g;
	static std::vector<State> gP(3), gM(3);
	static Input uP, uM;
	static Eigen::Matrix3f tmp_J;

	// Loop over all timesteps
	for (size_t time_step = 1; time_step < max_steps; time_step++) {
		if (time_step == 1) {
			g = RobotG(x, u);
			A_ = FindStateJacobian(x, u);
		} else {
			g = RobotG(g, u);
			A_ = FindStateJacobian(g, u);
		}
		p_star_.push_back(g.head(3));  // Save position with no input change
	  MotionVarianceIntegration();  // Update Mtau_

    for (int dim = 0; dim < 3; dim++) {  // Loop over input dimension
			uP = u; uP[dim] += j_step;
			uM = u; uM[dim] -= j_step;
			if (time_step == 1) {
				gP[dim] = RobotG(x, uP);
				gM[dim] = RobotG(x, uM);
			} else {
				gP[dim] = RobotG(gP[dim], uP);
				gM[dim] = RobotG(gM[dim], uM);
			}
			tmp_J.col(dim) = (gP[dim].head(3) - gM[dim].head(3)) / (2.0*j_step);
		}
		J_.push_back(tmp_J);
	}		
}  // Linearize

void QuadrotorACA3d::ForwardPrediction(const Input& u,
																				const float& time_horizon) {
	State x_tilde = x_hat_;
	x_tilde.head(3) = Position::Zero();
	Linearize(x_tilde, u, static_cast<int>(time_horizon/dt_));
}  // ForwardPrediction

Eigen::VectorXf QuadrotorACA3d::desired_position(void) {
	return p_star_.back();
}  // desired_position

Eigen::VectorXf QuadrotorACA3d::trajectory_position(size_t time_step) {
	return p_star_[time_step];
}

