#include "QuadrotorACA3d.h"
#include "linear_programming_3d.h"
#include <stdio.h>

QuadrotorACA3d::QuadrotorACA3d(void) {
	time_horizon_ = 1.0;

	desired_u_ = Input::Zero();
	delta_u_ = Input::Zero();
	g_ = State::Zero();
	gp_.resize(3);
	gm_.resize(3);
	this->Setup();
	this->SetupNoise();

	p_star_.resize(static_cast<int>(time_horizon_/dt_));
	J_.resize(static_cast<int>(time_horizon_/dt_));
};  // QuadrotorACA3d

QuadrotorACA3d::QuadrotorACA3d(float time_horizon) {
	time_horizon_ = time_horizon;

	desired_u_ = Input::Zero();
	delta_u_ = Input::Zero();
	g_ = State::Zero();
	gp_.resize(3);
	gm_.resize(3);

	this->Setup();
	this->SetupNoise();
	
	p_star_.resize(static_cast<int>(time_horizon_/dt_));
	J_.resize(static_cast<int>(time_horizon_/dt_));
}  // QuadrotorACA3d

void QuadrotorACA3d::SetupNoise(void) {
	Z_ = Eigen::Matrix3f::Zero();
	float scale = 1.0;
	Z_.block<3,3>(0,0) = scale*scale*0.1*0.1*Eigen::Matrix3f::Identity();
}  // SetupNoise

void QuadrotorACA3d::set_time_horizon(float time_horizon) {
	time_horizon_ = time_horizon;
}  // set_time_horizon

void QuadrotorACA3d::AvoidCollisions(const Input& desired_input,
																		 std::vector<Obstacle3d>& obstacle_list) {
	set_desired_u(desired_input);
	ResetDeltaU();
	ClearHalfplanes();
	bool found_collision;
	for (int loop_index = 0; loop_index < 10; ++loop_index) {
		ForwardPrediction();
		std::vector<int> potential_colliding_planes =
			FindPotentialCollidingPlanes(obstacle_list);
		found_collision = IsThereACollision(obstacle_list,
																				potential_colliding_planes);
		if (!found_collision)
			break;
		CalculateDeltaU();
	}
	u_ = desired_u_ + delta_u_;
}  // AvoidCollision

void QuadrotorACA3d::set_desired_u(const Input& desired_u) {
	desired_u_ = desired_u;
}  // set_desired_u

void QuadrotorACA3d::ResetDeltaU(void) {
	delta_u_ = Input::Zero();
}  // ResetDeltaU

QuadrotorACA3d::XXmat
QuadrotorACA3d::MotionVarianceDerivative(const XXmat& Mtau) {
	return A_*Mtau + Mtau*A_.transpose() + M_;
}  // MotionVarianceDerivative

void QuadrotorACA3d::MotionVarianceIntegration(void) {
	XXmat M1 = MotionVarianceDerivative(Mtau_);
	XXmat M2 = MotionVarianceDerivative(Mtau_ + 0.5*dt_*M1);
	XXmat M3 = MotionVarianceDerivative(Mtau_ + 0.5*dt_*M2);
	XXmat M4 = MotionVarianceDerivative(Mtau_ + dt_*M3);
	Mtau_ =  Mtau_ + (dt_/6.0)*(M1 + 2.0*M2 + 2.0*M3 + M4);
}  // MotionVarianceIntegration

float QuadrotorACA3d::VarianceProjection(const Eigen::Matrix3f& A,
																				 const Position& b) {
  float c = 3.841;
	Eigen::LLT<Eigen::MatrixXf> lltOfA(A);
	Eigen::MatrixXf L = lltOfA.matrixL();
	return c*b.transpose()*L*b;
}  // VarianceProjection

QuadrotorACA3d::Position
QuadrotorACA3d::PositionUncertaintyProjection(const Position& normal) {
  return normal*VarianceProjection(Mtau_.block(0,0,3,3), normal);
}  // PositionUncertaintyProjection

QuadrotorACA3d::Position
QuadrotorACA3d::SensingUncertaintyProjection(const Position& normal) {
  return normal*VarianceProjection(Z_, normal);
}  // SensingUncertaintyProjection

float QuadrotorACA3d::sigma(const Position& normal) {
  return VarianceProjection(Mtau_.block(0,0,3,3) + Z_, normal);
}  // sigma

void QuadrotorACA3d::Linearize(const State& x, const Input& u) {
	float j_step = 0.0009765625;
	p_star_[0] = x.head(3);
	Mtau_ = XXmat::Zero();

	// Loop over all timesteps
	for (size_t time_step = 1; time_step < static_cast<size_t>(time_horizon_/dt_);
			 time_step++) {
		if (time_step == 1) {
			g_ = RobotG(x, u);
			FindStateJacobian(x, u);
		} else {
			g_ = RobotG(g_, u);
			FindStateJacobian(g_, u);
		}
		p_star_[time_step] = g_.head(3);
	  MotionVarianceIntegration();  // Update Mtau_

		// Loop over input dimension and calculate numerical Jacobian
    for (int dim = 0; dim < 3; dim++) {  
			Input up = u;
			up[dim] += j_step;
			Input um = u;
			um[dim] -= j_step;
			if (time_step == 1) {
				gp_[dim] = RobotG(x, up);
				gm_[dim] = RobotG(x, um);
			} else {
				gp_[dim] = RobotG(gp_[dim], up);
				gm_[dim] = RobotG(gm_[dim], um);
			}
			J_[time_step].col(dim) = (gp_[dim].head(3) - gm_[dim].head(3))
				/ (2.0f*j_step);
		}
	}
}  // Linearize

void QuadrotorACA3d::ForwardPrediction() {
	//State x_tilde = x_hat_;
	//x_tilde.head(3) = Position::Zero();  // For relative obstacle definition
	//Linearize(x_tilde, desired_u_ + delta_u_);
	Linearize(x_hat_, desired_u_ + delta_u_);
}  // ForwardPrediction

QuadrotorACA3d::Position QuadrotorACA3d::desired_position(void) {
	return p_star_.back();
}  // desired_position

QuadrotorACA3d::Position QuadrotorACA3d::trajectory_position(size_t time_step) {
	return p_star_[time_step];
}  // trajectory_position

void QuadrotorACA3d::CreateHalfplane(const Eigen::VectorXf& pos_colliding,
																		 const Eigen::VectorXf& normal) {
	Eigen::Vector3f a;
	a.transpose() = normal.transpose()*J_.back();
	float b = (normal.transpose() * (pos_colliding - desired_position()) + 0.0002)
	  / a.norm();
	a.normalize();
	Plane tmp_plane;
  tmp_plane.point.x(b*a[0]);
	tmp_plane.point.y(b*a[1]);
	tmp_plane.point.z(b*a[2]);
	tmp_plane.normal.x(a[0]);
	tmp_plane.normal.y(a[1]);
	tmp_plane.normal.z(a[2]);
	halfplanes_.push_back(tmp_plane);
}  // CreateHalfplane

void QuadrotorACA3d::ClearHalfplanes(void) {
	halfplanes_.clear();
}  // ClearHalfplanes

std::vector<int> QuadrotorACA3d::FindPotentialCollidingPlanes(
	std::vector<Obstacle3d>& obstacle_list) {
	// Iterate over obstacles
	std::vector<int> potential_colliding_obstacle_indices;
  for (int obstacle_index = 0; obstacle_index < obstacle_list.size();
			++obstacle_index) {
		if (!obstacle_list[obstacle_index].IsTranslatedSeeable(desired_position())
				&& obstacle_list[obstacle_index].IsTrueSeeable(est_position())) {
			potential_colliding_obstacle_indices.push_back(obstacle_index);
		}
	}
	return potential_colliding_obstacle_indices;
}  // FindPotentialCollidingPlanes

bool QuadrotorACA3d::IsThereACollision(std::vector<Obstacle3d>& obstacle_list,
																			 std::vector<int>& index_list) {
	// First loop over the trajectory, piecewise, and check for collisions
	int trajectory_index = 1;
	for (; trajectory_index < static_cast<int>(time_horizon_/dt_);
			 ++trajectory_index) {
		// Loop over all the possible colliding planes to check for collision
		// against that single trajectory segment
		int plane_index = 0;
		for (; plane_index < index_list.size(); ++plane_index) {
			// Check the segment for a collisions
			if (obstacle_list[plane_index].IsIntersecting(trajectory_position(trajectory_index - 1),
																									  trajectory_position(trajectory_index))) {
				// If one is found, create the halfplane for that collision
				// and stop checking for collisions
			  CreateHalfplane(
					obstacle_list[plane_index].IntersectionPoint(trajectory_position(trajectory_index - 1),
																											 trajectory_position(trajectory_index)),
					obstacle_list[plane_index].normal());
				break;
			}
		}
		// If a collision was found, plane_index should be < index_list.size() and
		// the method should stop looping over the trajectory
		if (plane_index < index_list.size())
			break;
	}
	// If a collision was found, the loop was exited early and
	// trajectory_index < static_cast<int>(time_horizon_/dt_) and the
	// method should return true that a collision was found,
	// otherwise the for loop exited on the condition. 
	return trajectory_index < static_cast<int>(time_horizon_/dt_); 
}

void QuadrotorACA3d::CalculateDeltaU(void) {
	float max_speed = 5.0;
	Vector3 pref_v(-delta_u_[0], -delta_u_[1], -delta_u_[2]);
	Vector3 new_v;
	size_t plane_fail = linearProgram3(halfplanes_, max_speed,
																		 pref_v, false, new_v);
	if (plane_fail < halfplanes_.size())
		linearProgram4(halfplanes_, plane_fail, max_speed, new_v);

	delta_u_[0] += new_v.x();
	delta_u_[1] += new_v.y();
	delta_u_[2] += new_v.z();
}  // CalculateDeltaU


