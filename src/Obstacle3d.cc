#include "Obstacle3d.h"

Obstacle3d::Obstacle3d(void) { ; }
Obstacle3d::Obstacle3d(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
											 const Eigen::Vector3f& v2, const Eigen::Vector3f& normal,
											 float radius) {
	true_vertices_.push_back(v0);
	true_vertices_.push_back(v1);
	true_vertices_.push_back(v2);

	normal_ = normal;
	normal_.normalize();

	// Naive Minkowksi sum
	Eigen::Vector3f v0_tilde, v1_tilde, v2_tilde;
	v0_tilde = v0 + radius*normal_;
	v1_tilde = v1 + radius*normal_;
	v2_tilde = v2 + radius*normal_;

	translated_vertices_.push_back(v0_tilde);
	translated_vertices_.push_back(v1_tilde);
	translated_vertices_.push_back(v2_tilde);

	noise_vertices_.push_back(v0_tilde);
  noise_vertices_.push_back(v1_tilde);
	noise_vertices_.push_back(v2_tilde);
}

bool Obstacle3d::IsTrueSeeable(const Eigen::VectorXf& position) {
	return (normal_.transpose()*(position - true_vertices_[0]) > 0.0)
		? true : false;
}

bool Obstacle3d::IsTranslatedSeeable(const Eigen::VectorXf& position) {
	return (normal_.transpose()*(position - translated_vertices_[0]) > 0.0)
		? true : false;
}

void Obstacle3d::SolveForIntersection(const Eigen::VectorXf& current_position,
																			const Eigen::VectorXf& desired_position) {
	Eigen::Matrix3f A;
	Eigen::Vector3f x, b;
	A.col(0) = current_position - desired_position;
	A.col(1) = noise_vertices_[1] - noise_vertices_[0];
	A.col(2) = noise_vertices_[2] - noise_vertices_[0];
	b = current_position - noise_vertices_[0];
	x = A.fullPivHouseholderQr().solve(b);
	
	t = x[0];
	u = x[1];
	v = x[2];
}

bool Obstacle3d::IsIntersecting(const Eigen::VectorXf& current_position,
																const Eigen::VectorXf& desired_position) {
	SolveForIntersection(current_position, desired_position);
	if ((t >= 0 && t <= 1) && (u >= 0 && u <=1)
			&& (v >= 0 && v <= 0) && (u + v <= 1))
		return true;
	
	if ((t <= 0) && (u >= 0 && u <= 1) && (v >= 0 && v <= 1) && ( u + v <= 1)
			&& (current_position - noise_vertices_[0]).dot(normal_) < 0.0)
		return true;
	
	return false;
}

Eigen::VectorXf
  Obstacle3d::IntersectionPoint(const Eigen::VectorXf& current_position,
																const Eigen::VectorXf& desired_position) {
	return (1.0 - t)*current_position + t*desired_position;
}

	
	
