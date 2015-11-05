#include "Obstacle3d.h"

Obstacle3d::Obstacle3d(void) { ; }

Obstacle3d::Obstacle3d(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
											 const Eigen::Vector3f& v2, const Eigen::Vector3f& normal,
											 float radius) {
	true_vertices_.push_back(v0);
	true_vertices_.push_back(v1);
	true_vertices_.push_back(v2);

	normal_ = normal.normalized();

	// Naive Minkowksi sum
	Eigen::Vector3f v0_tilde = v0 + radius*normal_;
	Eigen::Vector3f v1_tilde = v1 + radius*normal_;
	Eigen::Vector3f v2_tilde = v2 + radius*normal_;

	translated_vertices_.push_back(v0_tilde);
	translated_vertices_.push_back(v1_tilde);
	translated_vertices_.push_back(v2_tilde);

	noise_vertices_.push_back(v0_tilde);
  noise_vertices_.push_back(v1_tilde);
	noise_vertices_.push_back(v2_tilde);
}  // Obstacle3d

Eigen::Vector3f Obstacle3d::true_vertices(int index) {
	return true_vertices_[index];
}  // true_vertices

Eigen::Vector3f Obstacle3d::translated_vertices(int index) {
	return translated_vertices_[index];
}  // translated_vertices

Eigen::Vector3f Obstacle3d::noise_vertices(int index) {
	return noise_vertices_[index];
}  // noise_vertices

Eigen::Vector3f Obstacle3d::normal(void) {
	return normal_;
}  // normal

bool Obstacle3d::IsIntersecting(const Eigen::Vector3f& current_position,
																const Eigen::Vector3f& desired_position) {
	SolveForIntersection(current_position, desired_position);

	return (0 <= u_ && u_ <= 1) && (0 <= v_ && v_ <= 1) && (u_ + v_ <= 1) &&
		((0 <= t_ && t_ <= 1) || (t_ <= 0 &&  (current_position - noise_vertices_[0]).dot(normal_) < 0.0));
}  // IsInterecting

Eigen::Vector3f
  Obstacle3d::IntersectionPoint(const Eigen::Vector3f& current_position,
																const Eigen::Vector3f& desired_position) {
	return (1.0 - t_)*current_position + t_*desired_position;
}  // IntersectionPoint

bool Obstacle3d::IsTrueSeeable(const Eigen::Vector3f& position) {
	return normal_.transpose()*(position - true_vertices_[0]) > 0.0;
}  // IsTrueSeeable 

bool Obstacle3d::IsTranslatedSeeable(const Eigen::Vector3f& position) {
	return normal_.transpose()*(position - translated_vertices_[0]) > 0.0;
}  // IsTranslatedSeeable

void Obstacle3d::SolveForIntersection(const Eigen::Vector3f& current_position,
																			const Eigen::Vector3f& desired_position) {
	Eigen::Matrix3f A;
	A.col(0) = current_position - desired_position;
	A.col(1) = noise_vertices_[1] - noise_vertices_[0];
	A.col(2) = noise_vertices_[2] - noise_vertices_[0];
	Eigen::Vector3f b = current_position - noise_vertices_[0];
	Eigen::Vector3f x = A.fullPivHouseholderQr().solve(b);
	
	t_ = x[0];
	u_ = x[1];
	v_ = x[2];
}  // SolveForIntersection

	
	
