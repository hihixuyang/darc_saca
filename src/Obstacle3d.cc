#include "Obstacle3d.h"

Obstacle3d::Obstacle3d(void) {
	true_intersection_calculated_ = translated_intersection_calculated_ =
		noise_intersection_calculated_ = false;
}  // Obstacle3d

Obstacle3d::Obstacle3d(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
											 const Eigen::Vector3f& v2, const Eigen::Vector3f& normal,
											 float radius) {
	true_intersection_calculated_ = translated_intersection_calculated_ =
		noise_intersection_calculated_ = false;
	
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

bool Obstacle3d::IsTrueIntersecting(const Eigen::Vector3f& current_position,
																		const Eigen::Vector3f& desired_position) {
	SolveForTrueIntersection(current_position, desired_position);
	return (0 <= true_u_ && true_u_ <= 1) &&
		(0 <= true_v_ && true_v_ <= 1)	&&
		(true_u_ + true_v_ <= 1) &&
		((0 <= true_t_ && true_t_ <= 1) ||
		 (true_t_ <= 0 &&
			(normal_.transpose()*(current_position - true_vertices_[0]) < 0.0)));
}  // IsTrueIntersecting

bool Obstacle3d::IsTranslatedIntersecting(const Eigen::Vector3f& current_position,
																					const Eigen::Vector3f& desired_position) {
	SolveForTranslatedIntersection(current_position, desired_position);
	return (0 <= translated_u_ && translated_u_ <= 1) &&
		(0 <= translated_v_ && translated_v_ <= 1) &&
		(translated_u_ + translated_v_ <= 1) &&
		((0 <= translated_t_ && translated_t_ <= 1) ||
		 (translated_t_ <= 0 &&
			(normal_.transpose()*(current_position - translated_vertices_[0]) < 0.0)));
}  //IsTranslatedIntersecting

bool Obstacle3d::IsNoiseIntersecting(const Eigen::Vector3f& current_position,
																		 const Eigen::Vector3f& desired_position) {
	SolveForNoiseIntersection(current_position, desired_position);
	return (0 <= noise_u_ && noise_u_ <= 1) &&
		(0 <= noise_v_ && noise_v_ <= 1) &&
		(noise_u_ + noise_v_ <= 1) &&
		((0 <= noise_t_ && translated_t_ <= 1) ||
		 (translated_t_ <= 0 &&
			(normal_.transpose()*(current_position - noise_vertices_[0]) < 0.0)));
}  // IsNoiseIntersecting

Eigen::Vector3f
Obstacle3d::TrueIntersectionPoint(const Eigen::Vector3f& current_position,
																	const Eigen::Vector3f& desired_position) {
	if (!true_intersection_calculated_)
		SolveForTrueIntersection(current_position, desired_position);
	return (1.0 - true_t_)*current_position + true_t_*desired_position;
}  // TrueIntersectionPoint

Eigen::Vector3f
Obstacle3d::TranslatedIntersectionPoint(const Eigen::Vector3f& current_position,
																				const Eigen::Vector3f& desired_position) {
	if (!translated_intersection_calculated_)
		SolveForTranslatedIntersection(current_position, desired_position);
	return (1.0 - translated_t_)*current_position + translated_t_*desired_position;
}  // TranslatedIntersectionPoint

Eigen::Vector3f
Obstacle3d::NoiseIntersectionPoint(const Eigen::Vector3f& current_position,
																	 const Eigen::Vector3f& desired_position) {
	if (!noise_intersection_calculated_)
		SolveForNoiseIntersection(current_position, desired_position);
	return (1.0 - noise_t_)*current_position + noise_t_*desired_position;
}  // NoiseIntersectionPoint

bool Obstacle3d::IsTrueSeeable(const Eigen::Vector3f& position) {
	return normal_.transpose()*(position - true_vertices_[0]) > 0.0;
}  // IsTrueSeeable 

bool Obstacle3d::IsTranslatedSeeable(const Eigen::Vector3f& position) {
	return normal_.transpose()*(position - translated_vertices_[0]) > 0.0;
}  // IsTranslatedSeeable

void Obstacle3d::SolveForTrueIntersection(const Eigen::Vector3f& current_position,
																			const Eigen::Vector3f& desired_position) {
	Eigen::Matrix3f A;
	A.col(0) = current_position - desired_position;
	A.col(1) = true_vertices_[1] - true_vertices_[0];
	A.col(2) = true_vertices_[2] - true_vertices_[0];
	Eigen::Vector3f b = current_position - true_vertices_[0];
	Eigen::Vector3f x = A.fullPivHouseholderQr().solve(b);
	true_t_ = x[0];
	true_u_ = x[1];
	true_v_ = x[2];
	true_intersection_calculated_ = true;
}  // SolveForTrueIntersection

void
Obstacle3d::SolveForTranslatedIntersection(const Eigen::Vector3f& current_position,
																					 const Eigen::Vector3f& desired_position) {
	Eigen::Matrix3f A;
	A.col(0) = current_position - desired_position;
	A.col(1) = translated_vertices_[1] - translated_vertices_[0];
	A.col(2) = translated_vertices_[2] - translated_vertices_[0];
	Eigen::Vector3f b = current_position - translated_vertices_[0];
	Eigen::Vector3f x = A.fullPivHouseholderQr().solve(b);
	translated_t_ = x[0];
	translated_u_ = x[1];
	translated_v_ = x[2];

	translated_intersection_calculated_ = true;
}  // SolveForTranslatedIntersection

void Obstacle3d::SolveForNoiseIntersection(const Eigen::Vector3f& current_position,
																					 const Eigen::Vector3f& desired_position) {
	Eigen::Matrix3f A;
	A.col(0) = current_position - desired_position;
	A.col(1) = noise_vertices_[1] - noise_vertices_[0];
	A.col(2) = noise_vertices_[2] - noise_vertices_[0];
	Eigen::Vector3f b = current_position - noise_vertices_[0];
	Eigen::Vector3f x = A.fullPivHouseholderQr().solve(b);
	noise_t_ = x[0];
	noise_u_ = x[1];
	noise_v_ = x[2];

	noise_intersection_calculated_ = true;
}  // SolveForNoiseIntersection

	
	
