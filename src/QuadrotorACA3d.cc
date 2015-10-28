#include "QuadrotorACA3d.h"
#include "linear_programming_3d.h"
#include <iostream>

QuadrotorACA3d::QuadrotorACA3d(void) {
	time_horizon_ = 1.0;
	this->Setup();
	this->SetupNoise();
};

QuadrotorACA3d::QuadrotorACA3d(float time_horizon) {
	time_horizon_ = time_horizon;
	this->Setup();
	this->SetupNoise();
}

void QuadrotorACA3d::SetupNoise(void) {
	Z_ = Eigen::Matrix3f::Zero();
	float scale = 1.0;
	Z_.block<3,3>(0,0) = scale*scale*0.1*0.1*Eigen::Matrix3f::Identity();
}  // SetupNoise

void QuadrotorACA3d::Linearize(const State& x, const Input& u) {
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
	for (size_t time_step = 1; time_step < static_cast<size_t>(time_horizon_/dt_);
			 time_step++) {
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

void QuadrotorACA3d::ForwardPrediction() {
	// TODO: Find out why x_hat is going to NaN!
	//State x_tilde = x_hat_;
	State x_tilde = x_;
	x_tilde.head(3) = Position::Zero();
	Linearize(x_tilde, desired_u_ + delta_u_);
}  // ForwardPrediction

Eigen::VectorXf QuadrotorACA3d::desired_position(void) {
	return p_star_.back();
}  // desired_position

Eigen::VectorXf QuadrotorACA3d::trajectory_position(size_t time_step) {
	return p_star_[time_step];
}

std::vector<int> QuadrotorACA3d::FindPotentialCollidingPlanes(
	std::vector<Obstacle3d>& obstacle_list) {
	
	// Iterate over obstacles
	std::vector<int> potential_colliding_obstacle_indices;
  for (int obstacle_index; obstacle_index < obstacle_list.size();
			++obstacle_index) {
		if (!obstacle_list[obstacle_index].IsTranslatedSeeable(desired_position())
				&& obstacle_list[obstacle_index].IsTrueSeeable(true_position()))  //&& obstacle_list[obstacle_index].IsTrueSeeable(est_position()))
			potential_colliding_obstacle_indices.push_back(obstacle_index);
	}
	return potential_colliding_obstacle_indices;
}

bool QuadrotorACA3d::CheckForCollision(std::vector<Obstacle3d>& obstacle_list,
											 std::vector<int>& index_list) {
	// First loop over the trajectory, piecewise, and check for collisions
	int trajectory_index = 1;
	for (; trajectory_index < static_cast<int>(time_horizon_/dt_);
			 ++trajectory_index) {
		// For a single trajectory segment, save the two endpoint positions
		Eigen::Vector3f current_position = trajectory_position(trajectory_index - 1);
		Eigen::Vector3f desired_position = trajectory_position(trajectory_index);
		
		// Loop over all the possible colliding planes to check for collision
		// against that single trajectory segment
		std::vector<int>::iterator plane_index;
		for (plane_index = index_list.begin();
				 plane_index != index_list.end(); ++plane_index) {
			// Check the segment for a collisions
			if (obstacle_list[*plane_index].IsIntersecting(current_position,
																										 desired_position)) {
				// If one is found, create the halfplane for that collision
			  CreateHalfplane(obstacle_list[*plane_index].IntersectionPoint(current_position, desired_position),
												obstacle_list[*plane_index].normal());
				// Then stop checking, collision is found so no need to keep going
				break;
			}
		}
		// If a collision was found, plane_index should != index_list.end() and
		// the method should stop looping over the trajectory
		if (plane_index != index_list.end()) break;
	}
	// If a collision was found, the loop was exited early and
	// trajectory_index < static_cast<int>(time_horizon_/dt_) and the
	// method should return true that a collision was found,
	// otherwise the for loop exited on the condition. 
	return trajectory_index < static_cast<int>(time_horizon_/dt_); 
}

void QuadrotorACA3d::CalculateDeltaU(void) {
	// Convert halfplanes from [pos_colliding, normal] to ``Plane''
	std::vector<Plane> halfplanes_converted;
	for (int halfplane_index = 0; halfplane_index < halfplanes_.size();
			 ++halfplane_index) {
		Eigen::Vector3f a;
		a.transpose() = halfplanes_[halfplane_index].normal_.transpose()*J_.back();
		float b = (halfplanes_[halfplane_index].normal_.transpose()
							 * (halfplanes_[halfplane_index].pos_colliding_ - desired_position()) + 0.0002) / a.norm();
		a.normalize();
		
		Plane tmp_halfplane_converted;
		tmp_halfplane_converted.point.x(b*a[0]);
		tmp_halfplane_converted.point.y(b*a[1]);
		tmp_halfplane_converted.point.z(b*a[2]);
		tmp_halfplane_converted.normal.x(a[0]);
		tmp_halfplane_converted.normal.y(a[1]);
		tmp_halfplane_converted.normal.z(a[2]);
		halfplanes_converted.push_back(tmp_halfplane_converted);
	}
	
	float max_speed = 3.0;
	Vector3 pref_v(0.0, 0.0, 0.0);
	Vector3 new_v;
	size_t plane_fail = linearProgram3(halfplanes_converted, max_speed, pref_v,
																		 false, new_v);
	if (plane_fail < halfplanes_converted.size()) {
		linearProgram4(halfplanes_converted, plane_fail, max_speed, new_v);
	}
	delta_u_ << new_v.x(), new_v.y(), new_v.z(), 0.0;
}

void QuadrotorACA3d::AvoidCollisions(const Input& desired_input,
																		 std::vector<Obstacle3d>& obstacle_list) {
	set_desired_u(desired_input);
	ResetDeltaU();
	ClearHalfplanes();
 
	bool found_collision;
	for (int loop_index = 0; loop_index < 20; ++loop_index) {
		ForwardPrediction();
		std::cout << "Desired z: " << desired_position()[2] << std::endl;
		std::vector<int> potential_colliding_planes =
			FindPotentialCollidingPlanes(obstacle_list);
		found_collision = CheckForCollision(obstacle_list,
																				potential_colliding_planes);
	  if (found_collision) {
			CalculateDeltaU();
			std::cout << halfplanes_.size() << std::endl;
			ClearHalfplanes();
		} else {
			break;
		}
	}
	u_ = desired_u_ + delta_u_;
}
