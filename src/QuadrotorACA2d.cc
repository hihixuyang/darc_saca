#include "QuadrotorACA2d.h"
//#include "linear_programming_2d.h"
#include <stdio.h>

QuadrotorACA2d::QuadrotorACA2d(void) {
	time_horizon_ = 1.0;
	desired_u_ = Input::Zero();
	delta_u_ = Input::Zero();
	g_ = State::Zero();
	gp_.resize(2);
	gm_.resize(2);
	this->Setup();
	this->SetupNoise();
	p_star_.resize(static_cast<int>(time_horizon_/dt_));
	J_.resize(static_cast<int>(time_horizon_/dt_));
}  // QuadrotorACA2d

QuadrotorACA2d::QuadrotorACA2d(float time_horizon) {
	time_horizon_ = time_horizon;
	desired_u_ = Input::Zero();
	delta_u_ = Input::Zero();
	g_ = State::Zero();
	gp_.resize(2);
	gm_.resize(2);
	this->Setup();
	this->SetupNoise();
	p_star_.resize(static_cast<int>(time_horizon_/dt_));
	J_.resize(static_cast<int>(time_horizon_/dt_));
}  // QuadrotorACA2d

void QuadrotorACA2d::SetupNoise(void) {
	Z_ = 0.01*0.01*Eigen::Matrix2f::Identity();
	//Z_ = Eigen::Matrix2f::Zero();
}  // SetupNoise

void QuadrotorACA2d::set_time_horizon(float time_horizon) {
	time_horizon_ = time_horizon;
}  // set_time_horizon

Eigen::Vector2f QuadrotorACA2d::sensing_noise(void) {
	return SampleGaussian(Eigen::Vector2f::Zero(), Z_);
}  // sensing_noise

void QuadrotorACA2d::AvoidCollisions(const Input& desired_input,
																		 std::vector<Obstacle2d>& obstacle_list) {
	set_desired_u(desired_input);
	ResetDeltaU();
	bool found_collision;
	for (int loop_index = 0; loop_index < 6; ++loop_index) {
		ForwardPrediction();
		if (loop_index == 0) {
			p_star_initial_ = p_star_;
		}
		found_collision = IsThereACollision(obstacle_list);
		if (!found_collision)
			break;
		CalculateDeltaU();
		ClearHalfplanes();
	}
	u_ = desired_u_ + delta_u_;
}  // AvoidCollisions

std::vector<Eigen::Vector2f> QuadrotorACA2d::InitialDesiredTrajectory(void) {
	return p_star_initial_;
}

std::vector<Eigen::Vector2f> QuadrotorACA2d::FinalDesiredTrajectory(void) {
	return p_star_;
}

std::vector<Eigen::Vector2f>
QuadrotorACA2d::ProjectLidar(std::vector<Eigen::Vector2f>& points) {
	std::vector<Eigen::Vector2f> projected_points(points.size());
	for (int i = 0; i < points.size(); ++i) {
		projected_points[i] = points[i];
	}
	
	return projected_points;
}  // ProjectLidar

void QuadrotorACA2d::set_desired_u(const Input& desired_u) {
	desired_u_ = desired_u;
}  // set_desired_u

void QuadrotorACA2d::ResetDeltaU(void) {
	delta_u_= Input::Zero();
}  // ResetDeltaU

QuadrotorACA2d::XXmat QuadrotorACA2d::MotionVarianceDerivative(const XXmat& Mtau) {
	return A_*Mtau + Mtau*A_.transpose() + M_;
}  // MotionVarianceDerivative

void QuadrotorACA2d::MotionVarianceIntegration(void) {
	XXmat M1 = MotionVarianceDerivative(Mtau_);
	XXmat M2 = MotionVarianceDerivative(Mtau_ + 0.5*dt_*M1);
	XXmat M3 = MotionVarianceDerivative(Mtau_ + 0.5*dt_*M2);
	XXmat M4 = MotionVarianceDerivative(Mtau_ + 0.5*dt_*M3);
	Mtau_ = Mtau_ + (dt_/6.0) * (M1 + 2.0*M2 + 2.0*M3 + M4);
}  // MotionVarianceIntegration

float QuadrotorACA2d::VarianceProjection(const Eigen::Matrix2f& A,
																				 const Eigen::Vector2f& b) {
	//float c = 3.841;  // 95%
	//float c = 5.024;  // 97.5%
	//float c = 6.635;  // 99%
	float c = 7.879;  // 99.5%
	Eigen::LLT<Eigen::MatrixXf> lltOfA(A);
	Eigen::MatrixXf L = lltOfA.matrixL();
	return c*b.transpose()*L*b;
}  // VarianceProjection

Eigen::Vector2f
QuadrotorACA2d::PositionUncertaintyProjection(const Eigen::Vector2f& normal) {
	return normal*VarianceProjection(Mtau_.block(0,0,2,2), normal);
}  // PositionUncertaintyProjection

Eigen::Vector2f
QuadrotorACA2d::SensingUncertaintyProjection(const Eigen::Vector2f& normal) {
	return normal*VarianceProjection(Z_, normal);
}  // SensingUncertaintyProjection

float QuadrotorACA2d::sigma(const Eigen::Vector2f& normal) {
	return VarianceProjection(Mtau_.block(0,0,2,2) + Z_, normal);
}  // sigma

void QuadrotorACA2d::Linearize(const State& x, const Input& u) {
	float j_step = 0.0009765625;
	p_star_[0] = x.head(2);
	Mtau_ = XXmat::Zero();
	// Loop over all timesteps
	for (size_t time_step = 1; time_step < static_cast<size_t>(time_horizon_/dt_);
			 ++time_step) {
		if (time_step == 1) {
			g_ = RobotG(x, u);
			FindStateJacobian(x,u);
		} else {
			g_ = RobotG(g_, u);
			FindStateJacobian(g_, u);
		}
		p_star_[time_step] = g_.head(2);
		MotionVarianceIntegration();  // Update Mtau_
		// Loop over input dimension and calculate numerical jacobian
		for (int dim = 0; dim < 2; ++dim) {
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
			J_[time_step].col(dim) = (gp_[dim].head(2) - gm_[dim].head(2))
				/ (2.0f*j_step);
		}
	}
}  // Linearize

void QuadrotorACA2d::ForwardPrediction(void) {
	State x_tilde = x_hat_;
	x_tilde.head(2) = Eigen::Vector2f::Zero(); // For relative obstacle definition
	Linearize(x_tilde, desired_u_ + delta_u_);
}  // ForwardPrediction

Eigen::Vector2f QuadrotorACA2d::desired_position(void) {
	return p_star_.back();
}  // desired_position

Eigen::Vector2f QuadrotorACA2d::trajectory_position(size_t time_step) {
	return p_star_[time_step];
}  // trajectory_position

void QuadrotorACA2d::CreateHalfplane(const Eigen::Vector2f& pos_colliding,
																		 const Eigen::Vector2f& pos_desired,
																		 const Eigen::Vector2f& normal) {
	Eigen::Vector2f a;
	a.transpose() = normal.transpose()*J_.back();
	float b = static_cast<float>((normal.transpose() *
																(pos_colliding - pos_desired +
																 sigma(normal)*normal)) + 0.0002f) / a.norm();
	a.normalize();
	
	Line tmp_line;
	tmp_line.point.x(b*a[0]);
	tmp_line.point.y(b*a[1]);
	tmp_line.direction.x(a[1]);
	tmp_line.direction.y(-a[0]);
	halfplanes_.push_back(tmp_line);
}  // CreateHalfplane

void QuadrotorACA2d::ClearHalfplanes(void) {
	halfplanes_.clear();
}  // ClearHalfplanes

bool QuadrotorACA2d::IsThereACollision(std::vector<Obstacle2d>& obstacle_list) {
	// First loop over the trajectory, piecewise, and check for collisions
	int trajectory_index = 1;
	Eigen::Vector2f current_position, desired_position;
	for (; trajectory_index < static_cast<int>(time_horizon_/dt_);
			 ++trajectory_index) {
		current_position = trajectory_position(trajectory_index - 1);
		desired_position = trajectory_position(trajectory_index);
		// Loop over all the possible colliding planes to check for collision
		// against that single trajectory segment
		int plane_index = 0;
		for (; plane_index < obstacle_list.size(); ++plane_index) {
			// First check for the segments dot products for a simpler computation,
			// avoiding matrix inverse of solving intersection
			bool p0 = obstacle_list[plane_index].normal().transpose() *
				(current_position - obstacle_list[plane_index].true_vertices(0)) > 0.0;
			bool p1 = obstacle_list[plane_index].normal().transpose() *
				(desired_position - obstacle_list[plane_index].translated_vertices(0)) < 0.0;
			if (p0 && p1) {
				// Check the segment for a collisions
				if (obstacle_list[plane_index].IsTranslatedIntersecting(
							current_position,
							desired_position)) {
				  // If one is found, create the halfplane for that collision
  				// and stop checking for collisions
  			  CreateHalfplane(
  					obstacle_list[plane_index].TranslatedIntersectionPoint(
							current_position,
							desired_position),
						this->desired_position(),
						obstacle_list[plane_index].normal());
					break;
				}
			}
		}
		// If a collision was found, plane_index should be < index_list.size() and
		// the method should stop looping over the trajectory
		if (plane_index < obstacle_list.size())
			break;
	}
	// If a collision was found, the loop was exited early and
	// trajectory_index < static_cast<int>(time_horizon_/dt_) and the
	// method should return true that a collision was found,
	// otherwise the for loop exited on the condition. 
	return trajectory_index < static_cast<int>(time_horizon_/dt_); 
}

void QuadrotorACA2d::CalculateDeltaU(void) {
	float max_speed = 5.0;
	Vector2 pref_v(0.0, 0.0);
	Vector2 new_v;
	
	size_t line_fail = linearProgram2(halfplanes_, max_speed,
																	  pref_v, false, new_v);
	if (line_fail < halfplanes_.size())
		linearProgram3(halfplanes_, 0, line_fail, max_speed, new_v);

	delta_u_[0] += new_v.x();
	delta_u_[1] += new_v.y();
}  // CalculateDeltaU
